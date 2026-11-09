"""Velocity-only bus contract and delayed simulated speed plant, no hardware."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m,fixtures=app.m,controls.fixtures


def speed_model(clock,positions,encoder_polarity=(-1,-1),speed_gain=1.,response_s=.06):
    class Model(fixtures.MechanicalMotorModel):
        def __init__(self):
            super().__init__(positions)
            self.rpm={1:0,2:0};self.speed={1:0.,2:0.};self.stamp=clock.now
        def integrate(self):
            now=clock.now;dt=now-self.stamp;self.stamp=now
            for mid in (1,2):
                demand=self.rpm[mid]*6*encoder_polarity[mid-1]*speed_gain if self.enabled[mid] else 0.
                self.speed[mid]+=(demand-self.speed[mid])*min(1.,dt/response_s)
                if not self.stuck:self.physical[mid]+=self.speed[mid]*dt
        def write(self,data):
            self.integrate()
            result=super().write(data)
            if data[2]==1:self.rpm[data[1]]=int.from_bytes(data[3:5],'big',signed=True)
            return result
        def read(self,*args,**kwargs):
            self.integrate();return super().read(*args,**kwargs)
    return Model()


class VelocityDriveTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    enable=controls.SeparateControlTests.enable

    def driver(self,pose=(0.,0.),**settings):
        bus=speed_model(self.clock,pose)
        d=m.VelocityMotorService(dict(m.CONFIG,startup_auto_home=False,**settings),lambda:bus)
        d.serial=bus;d.stop_event=fixtures.Event(self.clock)
        d.sent_target=d.manual_target=d._initialize();d.ready=True;d.phase='DISABLED'
        self.clock.pump=lambda:d.input(0.,0.)
        d.input(0.,0.);d.confirm_reference();self.enable(d)
        return d,bus

    def tick(self,d,count=1):
        for _ in range(count):
            self.clock.sleep(.03);d._measure();d._velocity_tick()

    def test_start_enable_has_zero_speed_and_only_mode_zero(self):
        d,b=self.driver((12.,-8.))
        self.assertTrue(d.armed)
        self.assertEqual(b.rpm,{1:0,2:0})
        self.assertFalse(any(p[2] in (2,3,8,9,10) for p in b.writes))
        self.assertTrue(all(int.from_bytes(p[3:5],'big')==0 for p in b.writes if p[2]==0))
        for i,p in enumerate(b.writes):
            if p[2]==6:self.assertTrue(any(x[1]==p[1] and x[2]==1 and x[3:5]==b'\0\0' for x in b.writes[:i]))

    def test_tracking_error_produces_signed_speed_without_position_packets(self):
        d,b=self.driver();d.set_response_context('TRACK')
        for _ in range(20):
            d.offer_tracking_goal((-20.,10.),self.clock.now,(True,True),velocity=(-10.,0.))
            self.tick(d)
        self.assertLess(b.physical[1],-2.);self.assertGreater(b.physical[2],2.)
        self.assertFalse(any(p[2] in (2,3) for p in b.writes))

    def test_prediction_and_feedforward_drive_an_initially_centered_moving_target(self):
        d,b=self.driver();d.set_response_context('TRACK')
        for _ in range(8):
            d.offer_tracking_goal(tuple(d.angles),self.clock.now-.08,(True,False),velocity=(40.,0.))
            self.tick(d)
        self.assertGreater(d.speed_rpm[0],0);self.assertGreater(b.physical[1],0.)

    def test_manual_direction_mapping_is_applied_once(self):
        d,b=self.driver();d.set_response_context('MANUAL')
        self.clock.pump=lambda:d.input(1.,0.)
        d.input(1.,0.);self.tick(d,15)
        self.assertLess(b.physical[1],0.);self.assertLess(d.speed_rpm[0],0);self.assertGreater(b.rpm[1],0)

    def test_motor_sign_maps_mechanical_speed_to_raw_motor_direction(self):
        d,b=self.driver(motor_signs=(-1,1));d.set_response_context('TRACK')
        for _ in range(10):
            d.offer_tracking_goal((20.,0.),self.clock.now,(True,False));self.tick(d)
        self.assertGreater(b.rpm[1],0);self.assertGreater(d.angles[0],0.)

    def test_small_hold_disturbance_returns_to_original_anchor(self):
        d,b=self.driver();d.set_response_context('HOLD');anchor=d.velocity_hold
        b.physical[1]=2.3;self.tick(d,130)
        self.assertTrue(d.armed);self.assertEqual(d.velocity_hold,anchor)
        self.assertLess(abs(b.physical[1]),1.)

    def test_manual_release_brakes_then_latches_a_single_hold(self):
        d,b=self.driver();d.set_response_context('MANUAL')
        self.clock.pump=lambda:d.input(.25,0.)
        self.tick(d,20)
        self.clock.pump=lambda:d.input(0.,0.);d.input(0.,0.);d.set_response_context('HOLD')
        self.tick(d,100);anchor=d.velocity_hold
        self.assertIsNotNone(anchor);self.assertEqual(d.speed_rpm,[0,0])
        self.tick(d,30);self.assertEqual(d.velocity_hold,anchor)

    def test_missing_observations_brake_without_continuing_old_feedforward(self):
        d,b=self.driver();d.set_response_context('TRACK')
        for _ in range(8):
            d.offer_tracking_goal((20.,0.),self.clock.now,(True,False),velocity=(40.,0.));self.tick(d)
        d.offer_tracking_goal(None,self.clock.now,(False,False));d.set_response_context('LOST')
        self.tick(d,100)
        self.assertEqual(d.speed_rpm,[0,0]);self.assertTrue(d.armed)

    def test_degraded_feedback_sends_zero_while_retaining_enable(self):
        d,b=self.driver();d.set_response_context('TRACK')
        for _ in range(8):
            d.offer_tracking_goal((20.,0.),self.clock.now,(True,False));self.tick(d)
        d.feedback_issues[1]='timeout';d.input(0.,0.);d._velocity_tick()
        self.assertEqual(b.rpm,{1:0,2:0});self.assertTrue(d.armed)

    def test_disable_sends_zero_before_disable_even_after_expired_heartbeat(self):
        d,b=self.driver();self.clock.pump=None;self.clock.sleep(2.)
        before=len(b.writes);d._disable_now();tail=b.writes[before:]
        self.assertFalse(d.armed);self.assertEqual(b.rpm,{1:0,2:0})
        for mid in (1,2):
            zero=next(i for i,p in enumerate(tail) if p[1]==mid and p[2]==1)
            off=next(i for i,p in enumerate(tail) if p[1]==mid and p[2]==5)
            self.assertLess(zero,off)

    def test_pitch_outward_speed_is_zero_at_limit_and_inward_still_works(self):
        self.assertEqual(m.velocity_limit(100.,30.,(-100.,30.),500.,.1,.8),0.)
        self.assertLess(m.velocity_limit(-100.,30.,(-100.,30.),500.,.1,.8),0.)
        d,b=self.driver((0.,29.));d.set_response_context('MANUAL')
        self.clock.pump=lambda:d.input(0.,-1.)
        self.tick(d,45)
        self.assertLessEqual(b.physical[2],30.)

    def test_home_uses_nearest_yaw_zero_and_keeps_speed_mode(self):
        d,b=self.driver((350.,-20.));d.set_response_context('HOLD');d.request_home()
        for _ in range(250):
            self.tick(d)
            if not d.home_requested:break
        self.assertFalse(d.home_requested);self.assertTrue(d.startup_completed)
        self.assertLess(abs(m.signed_mechanical(b.physical[1])),1.)
        self.assertLess(abs(b.physical[2]),1.)
        self.assertFalse(any(p[2] in (2,3) for p in b.writes))

    def test_recheck_does_not_enable_or_send_position(self):
        d,b=self.driver();d.request_arm(False);d._disable_now();d.phase='DISABLED'
        before=len(b.writes);d.request_recheck();d._perform_recheck()
        self.assertFalse(d.armed or d.arm_requested)
        self.assertFalse(any(p[2] in (2,3,6) for p in b.writes[before:]))

    def test_legacy_position_packets_and_modes_are_rejected(self):
        d,b=self.driver();before=len(b.writes)
        for p in (m.set_single_turn_position(1,10.),m.select_mode(1,2)):
            with self.assertRaises(RuntimeError):d._write(p)
        self.assertEqual(len(b.writes),before)

    def test_angle_target_is_executed_via_speed_servo(self):
        d,b=self.driver();d.request_angle_move(15.,-10.,60.)
        for _ in range(220):
            self.tick(d)
            if not d.angle_move.active:break
        self.assertEqual(d.angle_move.state,'COMPLETE')
        self.assertFalse(any(p[2] in (2,3) for p in b.writes))

    def test_braking_cap_is_lower_near_limit_and_never_rounds_outward(self):
        distant=m.velocity_limit(300.,0.,(-100.,30.),500.,.1,.8)
        close=m.velocity_limit(300.,28.,(-100.,30.),500.,.1,.8)
        self.assertLess(close,distant);self.assertGreaterEqual(close,0.)
        self.assertEqual(m.velocity_limit(-300.,0.,(float('-inf'),float('inf')),500.,.1,.8),-300.)

    def test_local_hub_drives_velocity_without_browser_or_position_packets(self):
        d,b=self.driver();hub=m.TrackingHub(d,d.c,self.clock.monotonic)
        frame=hub.offer_frame([app.track()],640,480,self.clock.now,tuple(d.angles))
        hub.local_action(click=(frame['id'],.625,.375))
        for _ in range(15):
            hub.offer_frame([app.track()],640,480,self.clock.now,tuple(d.angles));hub.tick();self.tick(d)
        self.assertIsNone(hub.token);self.assertTrue(hub.local_owner)
        self.assertTrue(any(p[2]==1 and p[3:5]!=b'\0\0' for p in b.writes))
        self.assertFalse(any(p[2] in (2,3) for p in b.writes))

    def test_home_from_outside_pitch_limits_moves_inward_in_speed_mode(self):
        d,b=self.driver((0.,65.));d.home_requested=True
        for _ in range(250):
            self.tick(d)
            if not d.home_requested:break
        self.assertFalse(d.home_requested);self.assertLess(abs(b.physical[2]),1.)
        self.assertFalse(any(p[2] in (2,3) for p in b.writes))

    def test_hard_watchdog_fault_zeros_and_disables_both_axes(self):
        d,b=self.driver();self.clock.pump=None;self.clock.sleep(2.)
        with self.assertRaises(RuntimeError) as caught:d._velocity_tick()
        d._record_fault(caught.exception)
        self.assertFalse(d.armed);self.assertFalse(any(b.enabled.values()))
        self.assertEqual(b.rpm,{1:0,2:0})

    def test_one_axis_zero_failure_does_not_skip_other_axis_disable(self):
        d,b=self.driver();original=b.write
        def fail_one(data):
            if data[1]==1 and data[2]==1:raise OSError('simulated UART failure')
            return original(data)
        b.write=fail_one;before=len(b.writes)
        errors=d._disable_all()
        self.assertTrue(errors)
        self.assertTrue(all(any(p[1]==mid and p[2]==5 for p in b.writes[before:]) for mid in (1,2)))

    def test_reported_positive_125_6_pitch_home_sends_positive_raw_rpm_and_returns(self):
        # Independent hardware convention: positive raw speed decreases encoder angle.
        d,b=self.driver((-4.3,125.6));d.home_requested=True
        for _ in range(260):
            self.tick(d)
            if not d.home_requested:break
        pitch_packets=[int.from_bytes(p[3:5],'big',signed=True) for p in b.writes if p[1]==2 and p[2]==1]
        first=next(v for v in pitch_packets if v)
        self.assertGreater(first,0)
        self.assertFalse(d.home_requested)
        self.assertLess(abs(b.physical[2]),1.)
        self.assertTrue(all(d.home_direction_checked))

    def test_wrong_pitch_speed_polarity_is_stopped_before_old_recovery_escape(self):
        d,b=self.driver((0.,125.6),velocity_command_signs=(-1,1));d.home_requested=True
        with self.assertRaisesRegex(m.MotionProtectionFault,'SPEED_DIRECTION_MISMATCH'):
            for _ in range(60):self.tick(d)
        self.assertEqual(b.rpm,{1:0,2:0})
        self.assertLess(b.physical[2]-125.6,3.)

    def test_wrong_yaw_speed_polarity_is_stopped_as_well(self):
        d,b=self.driver((-4.3,0.),velocity_command_signs=(1,-1));d.home_requested=True
        with self.assertRaisesRegex(m.MotionProtectionFault,'SPEED_DIRECTION_MISMATCH'):
            for _ in range(60):self.tick(d)
        self.assertEqual(b.rpm,{1:0,2:0})

    def test_speed_mapping_cannot_change_encoder_or_joystick_signs(self):
        d,b=self.driver()
        self.assertEqual(d.c['motor_signs'],(1,1));self.assertEqual(d.c['joystick_signs'],(-1,-1))
        for signs in ((0,-1),(1,), (True,-1)):
            with self.assertRaises(ValueError):m.validate_visual_config(dict(m.CONFIG,velocity_command_signs=signs))

    def test_moving_integral_builds_only_in_lag_direction_and_is_bounded(self):
        value=0.
        for _ in range(200):value=m.moving_integral(value,3.,40.,40.,.04,0,m.CONFIG)
        self.assertEqual(value,m.CONFIG['velocity_tracking_i_cap'][0])
        self.assertEqual(m.moving_integral(value,0.,40.,40.,.04,0,m.CONFIG),value)
        self.assertLess(m.moving_integral(value,-1.,40.,40.,.04,0,m.CONFIG),value)
        self.assertLess(m.moving_integral(0.,-3.,-40.,-40.,.04,0,m.CONFIG),0.)

    def test_stop_reversal_and_slowdown_clear_moving_integral(self):
        for ff,previous in ((0.,40.),(-40.,40.),(10.,40.)):
            with self.subTest(ff=ff):self.assertEqual(m.moving_integral(12.,3.,ff,previous,.04,0,m.CONFIG),0.)
        self.assertEqual(m.moving_integral(12.,3.,40.,40.,.04,0,m.CONFIG,False),0.)

    def test_real_driver_does_not_keep_integral_after_target_stop(self):
        d,b=self.driver();d.set_response_context('TRACK')
        d.velocity_context='TRACK';d.speed_integral=[12.,0.];d.tracking_previous_ff=[40.,0.]
        d.offer_tracking_goal(tuple(d.angles),self.clock.now,(False,False),velocity=(0.,0.))
        self.tick(d);self.assertEqual(d.speed_integral,[0.,0.])

    def test_coast_geometry_and_stale_observation_do_not_accumulate_integral(self):
        d,b=self.driver();d.set_response_context('TRACK');d.velocity_context='TRACK'
        for extra in ({'extrapolate':False},{'age':.3}):
            d.speed_integral=[12.,0.];d.tracking_previous_ff=[40.,0.]
            d.offer_tracking_goal((20.,0.),self.clock.now-extra.get('age',0),(True,False),velocity=(40.,0.),extrapolate=extra.get('extrapolate',True))
            self.tick(d);self.assertEqual(d.speed_integral,[0.,0.])
