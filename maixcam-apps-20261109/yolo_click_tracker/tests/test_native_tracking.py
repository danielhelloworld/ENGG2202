"""Absolute aim, native replanning, coordinate mapping and mode handoff."""
import unittest
import test_tracker as app
import test_separate_controls as controls
from native_model import make_native_model

m,fixtures=app.m,controls.fixtures


class NativeTrackingTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    enable=controls.SeparateControlTests.enable

    def driver(self,pose=(0.,0.),**settings):
        settings.setdefault('native_velocity_damping_s',0.)  # Existing mode/mailbox regressions.
        settings.setdefault('continuous_yaw',False)
        bus=make_native_model(fixtures.MechanicalMotorModel,self.clock,pose)
        d=m.VisualMotorService(dict(m.CONFIG,startup_auto_home=False,**settings),lambda:bus)
        d.serial=bus;d.stop_event=fixtures.Event(self.clock)
        d.sent_target=d.manual_target=d._initialize();d.ready=True;d.phase='DISABLED'
        self.clock.pump=lambda:d.input(0.,0.);d.input(0.,0.);d.confirm_reference();self.enable(d)
        d.set_response_context('TRACK');d.phase='MANUAL'
        return d,bus

    def tick(self,d):
        self.clock.sleep(.02);d._measure();d._advance(d.manual_target,d.c['max_rate'])

    def test_latest_absolute_target_wins_and_is_not_a_sequence_of_small_waypoints(self):
        d,bus=self.driver();before=len(bus.writes)
        d.offer_tracking_goal((20.,10.),self.clock.now,(True,True))
        d.offer_tracking_goal((-20.,-10.),self.clock.now,(True,True))
        self.assertEqual(len(bus.writes),before)
        d.input(.1,.1);self.tick(d)
        self.assertEqual(d.sent_target,(-20.,-10.))
        self.assertEqual(d.native_tracker.engaged,[True,True])
        self.assertEqual(d.programmed_rpm,[60,60])
        self.assertFalse(any(p[2] in (5,6,8,9,10) for p in bus.writes[before:]))
        self.assertLessEqual(len([p for p in bus.writes[before:] if p[2]==3]),4)

    def test_center_entry_updates_reference_once_then_holds_despite_noisy_goal(self):
        d,bus=self.driver();d.offer_tracking_goal((-10.,-6.),self.clock.now,(True,True));self.tick(d)
        before=len([p for p in bus.writes if p[2]==3])
        d.offer_tracking_goal((1.,1.),self.clock.now,(False,False));self.tick(d)
        reference=d.sent_target
        self.assertEqual(reference,(1.,1.))
        self.assertEqual(len([p for p in bus.writes if p[2]==3]),before+2)
        before+=2
        d.offer_tracking_goal((.5,.6),self.clock.now,(False,False));self.tick(d)
        self.assertEqual(d.sent_target,reference)
        self.assertEqual(len([p for p in bus.writes if p[2]==3]),before)

    def test_new_aim_during_mode_setup_replaces_the_original_before_first_move(self):
        d,bus=self.driver()
        original_write=bus.write
        def write(packet):
            result=original_write(packet)
            if packet[2]==0 and int.from_bytes(packet[3:5],'big')==2:
                d.offer_tracking_goal((-18.,-9.),self.clock.now,(True,True))
            return result
        bus.write=write
        d.offer_tracking_goal((18.,9.),self.clock.now,(True,True))
        self.tick(d)
        self.assertEqual(d.sent_target,(-18.,-9.))

    def test_uart_drain_is_nonblocking_and_response_read_uses_frame_length(self):
        d,bus=self.driver()
        reads=[]
        original_read=bus.read
        def read(size,timeout):
            reads.append((size,timeout))
            return original_read(size,timeout)
        bus.read=read
        d._feedback_once(1,m.FEEDBACK_MECHANICAL_ANGLE)
        self.assertEqual(reads[0],(512,0))
        self.assertIn((9,2),reads)

    def test_no_separate_tracking_cap_follows_driver_speed_without_changing_manual(self):
        c=dict(m.CONFIG,motor_rpm=60)
        m.validate_visual_config(c)
        self.assertEqual(m.native_tracking_rates(c),(360.,360.))
        self.assertEqual(c['manual_rate'],(240.,180.))
        c['native_tracking_rate']=(120.,90.)
        self.assertEqual(m.native_tracking_rates(c),(120.,90.))
        c['native_tracking_rate']=(361.,90.)
        with self.assertRaises(ValueError):m.validate_visual_config(c)

    def test_clear_target_keeps_native_hold_without_mode_change_or_disable(self):
        d,bus=self.driver();d.offer_tracking_goal((-10.,-6.),self.clock.now,(True,True));self.tick(d)
        before=len(bus.writes);d.offer_tracking_goal(None,self.clock.now,(False,False));d.set_response_context('HOLD');self.tick(d)
        self.assertTrue(all(d.native_tracker.engaged));self.assertTrue(d.armed)
        self.assertTrue(d._fixed_hold_active())
        modes=[(p[1],int.from_bytes(p[3:5],'big')) for p in bus.writes[before:] if p[2]==0]
        self.assertEqual(modes,[])
        self.assertFalse(any(p[2] in (5,6) for p in bus.writes[before:]))

    def test_mode_handoff_brakes_before_switching_a_moving_axis(self):
        d,bus=self.driver();d.offer_tracking_goal((-10.,-6.),self.clock.now,(True,True));self.tick(d)
        before=len(bus.writes);d.set_response_context('MANUAL');d.response.velocity=(40.,0.)
        self.assertFalse(d.native_tracker.restore())
        self.assertFalse(any(p[2]==0 for p in bus.writes[before:]))
        d.response.velocity=(0.,0.)
        self.assertTrue(d.native_tracker.restore())
        self.assertFalse(any(d.native_tracker.engaged))
        self.assertTrue(d.armed)

    def test_one_frame_aim_jump_is_not_chased_until_a_distinct_frame_confirms_it(self):
        d,bus=self.driver();d.offer_tracking_goal((-10.,-6.),self.clock.now,(True,True));self.tick(d)
        self.clock.sleep(.03)
        stamp=self.clock.now
        d.offer_tracking_goal((50.,-6.),stamp,(True,True))
        self.assertEqual(d.native_tracker.message[0],(-10.,-6.))
        d.offer_tracking_goal((50.,-6.),stamp,(True,True))
        self.assertEqual(d.native_tracker.aim_rejections,1)
        self.assertEqual(d.native_tracker.message[0],(-10.,-6.))
        self.clock.sleep(.03)
        d.offer_tracking_goal((49.,-6.),self.clock.now,(True,True))
        self.assertEqual(d.native_tracker.message[0],(49.,-6.))

    def test_stale_aim_does_not_emit_old_rate_stream(self):
        d,bus=self.driver();d.offer_tracking_goal((-10.,-6.),self.clock.now-1.,(True,True));d.input(1.,0.)
        before=len([p for p in bus.writes if p[2]==3]);self.tick(d)
        self.assertFalse(any(d.native_tracker.engaged))
        self.assertEqual(len([p for p in bus.writes if p[2]==3]),before)

    def test_brief_detection_gap_holds_without_mode_churn_then_resumes_latest_aim(self):
        d,bus=self.driver();d.offer_tracking_goal((-10.,-6.),self.clock.now,(True,True));self.tick(d)
        before=len(bus.writes)
        d.offer_tracking_goal(None,self.clock.now,(False,False));d.set_response_context('LOST');self.tick(d)
        self.assertEqual(d.native_tracker.engaged,[True,True])
        self.assertEqual(d.stream_velocity,(0.,0.))
        self.assertTrue(d._fixed_hold_active())
        held=d.sent_target
        d.offer_tracking_goal(None,self.clock.now,(False,False));self.tick(d)
        self.assertEqual(d.sent_target,held)
        d.set_response_context('TRACK');d.offer_tracking_goal((-16.,-8.),self.clock.now,(True,True));self.tick(d)
        self.assertEqual(d.sent_target,(-16.,-8.))
        self.assertEqual(d.native_tracker.setup_count,2)
        self.assertFalse(any(p[2] in (0,1,5,6,7) for p in bus.writes[before:]))

    def test_detection_gap_never_keeps_mode_active_after_vision_feedback_expires(self):
        d,bus=self.driver();d.offer_tracking_goal((-10.,-6.),self.clock.now,(True,True));self.tick(d)
        d.offer_tracking_goal(None,self.clock.now,(False,False));d.set_response_context('LOST');self.tick(d)
        self.clock.sleep(d.c['vision_max_age']+.01)
        d.input(0.,0.);d._measure();d._advance(d.manual_target,d.c['max_rate'])
        self.assertEqual(d.native_tracker.engaged,[False,False])
        self.assertTrue(d.armed)

    def test_targets_are_limited_to_work_range_and_explicit_stop_restores_modes(self):
        d,bus=self.driver((140.,25.));d.offer_tracking_goal((999.,999.),self.clock.now,(True,True));self.tick(d)
        self.assertEqual(d.sent_target,(150.,30.))
        d._disable_now();self.assertFalse(d.armed);self.assertFalse(any(d.native_tracker.engaged))
        self.assertFalse(any(bus.enabled.values()))

    def test_capture_pose_compensation_builds_one_world_aim_without_double_counting(self):
        core=m.ClickTracker(m.CONFIG);tr=app.track();core.select(tr['key'],[tr],1.)
        core.step([tr],1.1,pose=(0.,0.),current_pose=(-5.,0.))
        self.assertAlmostEqual(core.aim_goal[0],-.125*87.)
        self.assertAlmostEqual(core.aim_goal[1],-.125*49.)

    def test_native_center_corrects_small_visible_offset_without_old_large_hysteresis(self):
        core=m.ClickTracker(dict(m.CONFIG,prediction_enabled=False))
        tr=app.track();core.select(tr['key'],[tr],1.)
        tr.update(cx=.5,cy=.5)
        core.step([tr],1.1,pose=(0.,0.),current_pose=(0.,0.))
        self.assertEqual(core.settled,[True,True])
        tr.update(cx=.512,cy=.5)  # 7.7px at 640px; old settled mode ignored this.
        core.step([tr],1.2,pose=(0.,0.),current_pose=(0.,0.))
        self.assertFalse(core.settled[0])
        self.assertLess(core.aim_goal[0],-.9)

    def test_live_velocity_brakes_reference_and_center_does_not_freeze_brake_offset(self):
        d,bus=self.driver(native_velocity_damping_s=.035)
        d.offer_tracking_goal((20.,0.),self.clock.now,(True,False));self.tick(d)
        self.tick(d)
        self.assertGreater(d.native_tracker.velocity[0],0.)
        self.assertGreater(d.native_tracker.damping[0],0.)
        self.assertLess(d.sent_target[0],20.)
        d.offer_tracking_goal((20.,0.),self.clock.now,(False,False));self.tick(d)
        self.assertFalse(d.native_tracker.centered[0])
        d.native_tracker.velocity=[0.,0.]
        d.native_tracker.previous_pose=list(zip(d.angles,d.mechanical_feedback_times))
        self.clock.sleep(.03);d.input(0.,0.);d.native_tracker.advance()
        self.assertEqual(d.sent_target[0],20.)
        self.assertTrue(d.native_tracker.centered[0])

    def test_relative_velocity_damping_keeps_up_with_equal_speed_target(self):
        c=dict(m.CONFIG)
        self.assertEqual(m.velocity_damped_goal(20.,80.,80.,c),(20.,0.))
        q,correction=m.velocity_damped_goal(20.,80.,0.,c)
        self.assertLess(q,20.);self.assertGreater(correction,0.)
        q,correction=m.velocity_damped_goal(-20.,-80.,0.,c)
        self.assertGreater(q,-20.);self.assertLess(correction,0.)

    def test_prediction_marker_matches_capture_image_not_current_rotated_pose(self):
        core=m.ClickTracker(dict(m.CONFIG));tr=app.track();core.select(tr['key'],[tr],1.)
        core.predictor.samples=3;core.predictor.velocity=(.1,0.)
        core.predictor.observe=lambda *args:None
        core.step([tr],1.1,pose=(0.,0.),current_pose=(-5.,0.),latency=.04)
        self.assertAlmostEqual(core.prediction_point[0],.625+.008)
