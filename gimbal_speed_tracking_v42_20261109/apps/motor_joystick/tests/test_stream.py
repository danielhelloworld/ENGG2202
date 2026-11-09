"""Command-stream behavior in a physical-position MODEL, not motor dynamics."""
import unittest
import test_demo as fixtures

m = fixtures.m


class StreamTests(unittest.TestCase):
    setUp = fixtures.MechanicalOnlyTests.setUp
    initialize = fixtures.MechanicalOnlyTests.initialize
    arm = fixtures.MechanicalOnlyTests.arm
    commands = fixtures.MechanicalOnlyTests.commands

    def step(self, goal, dt=.02):
        self.clock.sleep(dt)
        self.s._measure()
        self.s._advance(goal, self.c["max_rate"])

    def test_inverted_yaw_input_preserves_pitch_and_mechanical_coordinates(self):
        self.initialize()
        self.s.input(1, .5)
        self.assertEqual(self.s.joystick, (-1, .5))
        self.assertEqual(self.s.angles, (93.9, -1.1))
        self.s.input(-1, -.5)
        self.assertEqual(self.s.joystick, (1, -.5))

    def test_new_pitch_boundary_accepts_30_and_rejects_30_point_1(self):
        self.bus.physical[2] = 30
        self.initialize()
        self.bus.physical[2] = 30.1
        with self.assertRaisesRegex(RuntimeError, "allowed -100.0..30.0"):
            self.s._measure()
        self.assertFalse(self.commands(6))

    def test_outward_goal_is_clamped_and_every_pitch_packet_respects_new_limit(self):
        self.bus.physical[2] = 29.5
        self.arm()
        for _ in range(100):
            self.step((93.9, 80))
        targets = [m.signed_mechanical(int.from_bytes(d[3:5], "big")/10)
                   for d in self.commands(3) if d[1] == 2]
        self.assertTrue(targets)
        self.assertTrue(all(-100 <= q <= 30 for q in targets))
        self.assertLessEqual(self.bus.physical[2], 30)
        self.assertGreaterEqual(self.bus.physical[2], 29.5)

    def test_hold_does_not_retransmit_identical_position_packets(self):
        self.arm()
        before = len(self.commands(3))
        for _ in range(100):
            self.step(self.s.angles)
        self.assertEqual(len(self.commands(3)), before)

    def test_position_stream_is_rate_limited_and_software_velocity_ramps(self):
        self.arm()
        times, original = [], self.bus.write

        def tracked(data):
            if data[2] == 3 and data[1] == 1:
                times.append(self.clock.now)
            return original(data)

        self.bus.write = tracked
        prior_v, prior_t = 0., self.s.last_send
        for _ in range(50):
            self.step((120, -1.1))
            now = self.s.last_send  # Planner timestamp, not completion of UART writes.
            velocity = self.s.stream_velocity[0]
            self.assertLessEqual(abs(velocity-prior_v), self.c["command_accel_deg_s2"][0]*(now-prior_t)+1e-7)
            self.assertLessEqual(abs(velocity), self.c["max_rate"][0])
            prior_v, prior_t = velocity, now
        self.assertGreater(len(times), 3)
        self.assertTrue(all(b-a >= self.c["position_period_s"]-1e-7 for a,b in zip(times,times[1:])))

    def test_release_holds_once_and_does_not_chase_feedback_noise(self):
        self.arm()
        for _ in range(25):
            self.step((120, -1.1))
        self.s._release_hold()
        hold = self.s.manual_target
        self.step(hold)
        self.assertEqual(self.s.stream_velocity, (0, 0))
        before = len(self.commands(3))
        position = self.bus.physical[1]
        for i in range(20):
            self.bus.physical[1] = position + (.1 if i%2 else -.1)
            self.step(hold)
        self.assertEqual(len(self.commands(3)), before)

    def test_no_resend_path_still_checks_expired_feedback(self):
        self.arm()
        self.s.feedback_time = self.clock.now-1
        with self.assertRaisesRegex(RuntimeError, "expired"):
            self.s._advance(self.s.angles, self.c["max_rate"])
        self.assertFalse(any(self.bus.enabled.values()))

    def test_direct_mode_gate_still_rejects_multiturn_mode(self):
        self.arm()
        before = len(self.commands(0))
        with self.assertRaisesRegex(RuntimeError, "configured single-turn"):
            self.s._write(m.select_mode(1, 3))
        self.assertEqual(len(self.commands(0)), before)

    def test_worker_inverted_input_moves_left_then_release_stays_fixed(self):
        vector, ticks, released = [(0, 0)], [0], []
        self.clock.pump = lambda: self.s.input(*vector[0])

        def schedule():
            ticks[0] += 1
            if self.s.phase == "DISABLED" and self.s.ready:
                self.s.confirm_reference()
                self.s.request_arm(True)
            elif self.s.phase in ("ARMED", "MANUAL"):
                vector[0] = (1, .3) if ticks[0] < 60 else (0, 0)
                if ticks[0] > 70:
                    released.append(tuple(self.bus.physical.values()))
            if ticks[0] > 95 or self.s.phase == "FAULT":
                self.s.stop_event.set()

        self.s.stop_event.callback = schedule
        self.s._run()
        self.assertIsNone(self.s.fault)
        self.assertTrue(released)
        self.assertLess(released[0][0], 93.9)
        self.assertGreater(released[0][1], -1.1)
        self.assertEqual(released[-1], released[-2])
        self.assertFalse(any(self.bus.enabled.values()))

    def test_faster_profile_increases_worker_motion_without_changing_limits(self):
        def run_profile(config):
            bus = fixtures.MechanicalMotorModel((0, 0))
            service = m.MechanicalOnlyService(config, lambda: bus)
            service.stop_event = fixtures.Event(self.clock)
            vector, started = [(0, 0)], [None]
            self.clock.pump = lambda: service.input(*vector[0])

            def schedule():
                if service.phase == "DISABLED" and service.ready:
                    service.confirm_reference()
                    service.request_arm(True)
                elif service.phase in ("ARMED", "MANUAL"):
                    started[0] = self.clock.now if started[0] is None else started[0]
                    vector[0] = (1, 1)
                    if self.clock.now-started[0] >= 1.5:
                        service.stop_event.set()
                elif service.phase == "FAULT":
                    service.stop_event.set()

            service.stop_event.callback = schedule
            service._run()
            self.assertIsNone(service.fault)
            self.assertFalse(any(bus.enabled.values()))
            return tuple(abs(bus.physical[mid]) for mid in (1, 2))

        old = dict(self.c, motor_rpm=3, motor_acceleration=6,
                   max_rate=(12, 8), home_rate=(6, 5), manual_lead_deg=1.5,
                   command_accel_deg_s2=(40, 30), position_period_s=.05)
        before = run_profile(old)
        after = run_profile(dict(self.c))
        for slow, fast in zip(before, after):
            self.assertGreater(fast, slow*1.4)
        self.assertLessEqual(after[1], 30)
        self.assertEqual(self.c["limits"], ((-150., 150.), (-100., 30.)))


if __name__ == "__main__":
    unittest.main()
