"""Speed-mode UART commands and transitions using a rate-integrating model."""
import math
import unittest
import test_native_tracking as native

m = native.m


class SpeedTrackingTests(unittest.TestCase):
    setUp = native.NativeTrackingTests.setUp
    enable = native.NativeTrackingTests.enable
    tick = native.NativeTrackingTests.tick

    def driver(self, pose=(0., 0.), **settings):
        return native.NativeTrackingTests.driver(self, pose, tracking_motor_control='speed', **settings)

    def aim(self, d, goal=(20., -15.), active=(True, True), velocity=(0., 0.)):
        d.offer_tracking_goal(goal, self.clock.now, active, velocity)
        self.tick(d)

    def test_default_speed_mode_and_signed_rpm_without_tracking_position_packets(self):
        self.assertEqual(m.CONFIG['tracking_motor_control'], 'speed')
        d, bus = self.driver()
        before = len(bus.writes)
        for _ in range(12): self.aim(d)
        packets = bus.writes[before:]
        self.assertEqual(bus.modes, {1: 0, 2: 0})
        self.assertFalse(any(p[2] == 3 for p in packets))
        self.assertLess(d.native_tracker.command_rpm[0], 0)
        self.assertGreater(d.native_tracker.command_rpm[1], 0)
        self.assertGreater(bus.physical[1], 0)
        self.assertLess(bus.physical[2], 0)
        self.assertTrue(all(abs(v) <= d.c['motor_rpm'] for v in d.native_tracker.command_rpm))

    def test_negative_motor_sign_is_applied_once(self):
        d, bus = self.driver(motor_signs=(-1, 1))
        for _ in range(8): self.aim(d)
        self.assertGreater(d.native_tracker.command_rpm[0], 0)
        self.assertGreater(d.angles[0], 0)

    def test_acceleration_and_center_deadzone(self):
        d, bus = self.driver()
        old = (0., 0.)
        for _ in range(8):
            self.aim(d)
            for v, previous, accel in zip(d.native_tracker.command_velocity, old, d.c['tracking_accel_deg_s2']):
                self.assertLessEqual(abs(v-previous), accel*.1+1e-8)
            old = tuple(d.native_tracker.command_velocity)
        for _ in range(30): self.aim(d, tuple(d.angles), (False, False))
        self.assertEqual(d.native_tracker.command_rpm, [0, 0])

    def test_limits_block_outward_speed_but_allow_inward(self):
        d, bus = self.driver((0., 29.5))
        for _ in range(10): self.aim(d, (0., 100.))
        self.assertEqual(d.native_tracker.command_rpm[1], 0)
        for _ in range(15): self.aim(d, (0., 0.))
        self.assertGreater(d.native_tracker.command_rpm[1], 0)
        self.assertGreaterEqual(bus.physical[2], -90.)

    def test_feedback_pause_writes_zero_even_with_expired_heartbeat(self):
        d, bus = self.driver()
        for _ in range(10): self.aim(d)
        d.feedback_issues[0] = 'Missing feedback'
        d.command_time = self.clock.now-2.
        d.native_tracker.stop()
        self.assertEqual(bus.rates, {1: 0, 2: 0})
        self.assertEqual(bus.modes, {1: 0, 2: 0})
        with self.assertRaises(RuntimeError): d._write(m.set_speed(1, 1))

    def test_sync_speed_limit_cannot_accidentally_command_motion(self):
        d, bus = self.driver()
        self.aim(d)
        before = len(bus.writes)
        d._sync_driver_speed()
        self.assertEqual(len(bus.writes), before)

    def test_lost_target_stops_then_restores_position_hold(self):
        d, bus = self.driver()
        for _ in range(10): self.aim(d)
        d.set_response_context('LOST')
        d.offer_tracking_goal(None, self.clock.now, (False, False))
        for _ in range(20): self.tick(d)
        self.assertFalse(any(d.native_tracker.engaged))
        self.assertEqual(bus.modes, {1: 4, 2: 4})
        self.assertTrue(d.armed)
        held = tuple(bus.physical.values())
        for _ in range(10): self.tick(d)
        self.assertEqual(tuple(bus.physical.values()), held)

    def test_manual_home_and_disarm_mode_handoffs(self):
        for context in ('MANUAL', 'HOME', 'DISARM'):
            with self.subTest(context=context):
                d, bus = self.driver()
                for _ in range(6): self.aim(d)
                if context == 'DISARM':
                    d._disable_now()
                    self.assertFalse(any(bus.enabled.values()))
                elif context == 'HOME':
                    d.request_home(); d._begin_home()
                    self.assertTrue(d.recovery_active)
                else:
                    d.set_response_context('MANUAL')
                    for _ in range(20): self.tick(d)
                self.assertFalse(any(d.native_tracker.engaged))
                self.assertEqual(bus.modes, {1: 4, 2: 4})

    def test_stale_visual_aim_stops_instead_of_repeating_last_speed(self):
        d, bus = self.driver()
        for _ in range(12): self.aim(d)
        self.clock.sleep(d.c['vision_max_age']+.01)
        d.input(0.,0.)
        self.tick(d)
        self.assertEqual(d.native_tracker.command_rpm, [0,0])
        for _ in range(25): self.tick(d)
        self.assertFalse(any(d.native_tracker.engaged))

    def test_actual_feedback_pause_sends_zero_during_grace_period(self):
        d, bus = self.driver()
        for _ in range(12): self.aim(d)
        bus.drop_feedback.add(1)
        d._measure()
        self.assertTrue(d.motion_paused)
        self.assertEqual(bus.rates, {1:0,2:0})
        self.assertTrue(d.armed)

    def test_continuous_yaw_crosses_180_without_reversing_speed(self):
        d, bus = self.driver((179.,0.), continuous_yaw=True)
        for _ in range(30): self.aim(d, (200.,0.))
        self.assertGreater(d.angles[0],180.)
        self.assertLessEqual(d.native_tracker.command_rpm[0],0)
        self.assertIsNone(d.fault)

    def test_latest_goal_is_used_after_speed_mode_setup(self):
        d, bus = self.driver()
        original = bus.write
        def write(packet):
            result = original(packet)
            if packet[2] == m.FUNCTION_MODE and int.from_bytes(packet[3:5],'big') == 0:
                d.offer_tracking_goal((-20.,-10.),self.clock.now,(True,True))
            return result
        bus.write = write
        self.aim(d)
        self.assertEqual(d.sent_target,(-20.,-10.))
        for _ in range(10): self.aim(d,(-20.,-10.))
        self.assertGreater(d.native_tracker.command_rpm[0],0)

    def test_guard_checks_physical_direction_after_speed_sign_conversion(self):
        for pitch, outward_rpm in ((29.5,-1),(-89.5,1)):
            with self.subTest(pitch=pitch):
                d, bus = self.driver((0.,pitch))
                self.aim(d,(0.,pitch))  # Enter speed mode at rest.
                before = len(bus.writes)
                with self.assertRaisesRegex(RuntimeError,'Unsafe tracking speed'):
                    d._write(m.set_speed(2,outward_rpm))
                self.assertEqual(len(bus.writes),before)
                d._write(m.set_speed(2,-outward_rpm))  # The opposite raw RPM moves inward.

    def test_both_axes_move_toward_positive_and_negative_goals(self):
        for goal in ((20.,15.),(-20.,-15.)):
            for signs in ((1,1),(-1,1),(1,-1),(-1,-1)):
                with self.subTest(goal=goal,encoder_signs=signs):
                    d,bus=self.driver(motor_signs=signs)
                    initial=sum(abs(q) for q in goal)
                    for _ in range(30): self.aim(d,goal)
                    self.assertLess(sum(abs(q-a) for q,a in zip(goal,d.angles)),initial)
                    for q,a,v in zip(goal,d.angles,d.stream_velocity):
                        self.assertGreater(q*a,0.)
                        self.assertGreaterEqual(q*v,0.)

    def test_alternative_speed_polarities_are_axis_specific(self):
        d,bus=self.driver(speed_mode_signs=(1,-1))
        bus.speed_angle_signs={1:1,2:-1}  # A different explicitly modeled installation.
        for _ in range(20): self.aim(d,(20.,15.))
        self.assertGreater(d.native_tracker.command_rpm[0],0)
        self.assertLess(d.native_tracker.command_rpm[1],0)
        self.assertTrue(all(q>0 for q in d.angles))

    def test_invalid_speed_parameters_rejected(self):
        for settings in ({'tracking_motor_control': 'invalid'}, {'speed_tracking_kp': (math.nan, 1.)},
                         {'speed_tracking_margin_deg': -1.}, {'speed_mode_signs': (0,1)},
                         {'speed_mode_signs': (True,-1)}, {'speed_mode_signs': (-1,)}):
            with self.assertRaises(ValueError): m.validate_visual_config(dict(m.CONFIG, **settings))


if __name__ == '__main__': unittest.main()
