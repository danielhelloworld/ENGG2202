"""Single-owner recovery and HOME holding tests using the motor model only."""
import unittest
import test_demo as fixtures

m = fixtures.m


class RecheckTests(unittest.TestCase):
    setUp = fixtures.MechanicalOnlyTests.setUp
    initialize = fixtures.MechanicalOnlyTests.initialize
    arm = fixtures.MechanicalOnlyTests.arm
    commands = fixtures.MechanicalOnlyTests.commands

    def check(self):
        self.s.request_recheck()
        self.s._perform_recheck()

    def test_request_only_enqueues_and_success_stays_disabled(self):
        self.initialize()
        before = len(self.bus.writes)
        self.s.request_recheck()
        self.assertEqual(len(self.bus.writes), before)
        self.s._perform_recheck()
        self.assertTrue(self.s.ready)
        self.assertTrue(self.s.reference_valid)
        self.assertFalse(self.s.armed)
        self.assertFalse(self.s.zero_confirmed)
        self.assertFalse(self.commands(6))
        self.assertTrue(all(d[2] in (5, 0x0E) for d in self.bus.writes[before:]))

    def test_out_of_range_check_fails_and_can_be_retried_in_same_worker(self):
        self.bus.physical[2] = 116.2
        requested = [False]
        observed = []

        def schedule():
            if self.s.phase == "FAULT" and not requested[0]:
                requested[0] = True
                self.bus.physical[2] = 10
                self.s.request_recheck()
            if self.s.recheck_status == "PASSED":
                observed.append(self.s.snapshot())
                self.s.stop_event.set()

        self.s.stop_event.callback = schedule
        self.s._run()
        self.assertTrue(observed)
        self.assertIsNone(observed[0]["fault"])
        self.assertEqual(observed[0]["angles"][1], 10)
        self.assertFalse(observed[0]["armed"])
        self.assertFalse(self.commands(6))

    def test_still_outside_limits_is_not_cleared(self):
        self.initialize()
        self.bus.physical[2] = 31
        with self.assertRaisesRegex(RuntimeError, "outside calibrated limits"):
            self.check()
        self.assertFalse(self.s.reference_valid)
        self.assertEqual(self.s.recheck_status, "FAILED")
        self.assertIsNotNone(self.s.fault)

    def test_moving_or_missing_feedback_cannot_pass(self):
        self.initialize()
        self.bus.nonzero_speed = True
        with self.assertRaisesRegex(RuntimeError, "still moving"):
            self.check()
        self.assertFalse(self.s.ready)
        self.s.phase = "FAULT"
        self.bus.nonzero_speed = False
        self.bus.drop_feedback = {2}
        with self.assertRaisesRegex(RuntimeError, "No valid feedback"):
            self.check()
        self.assertFalse(self.s.ready)
        self.assertFalse(self.commands(6))

    def test_initial_disable_failure_prevents_recheck_success(self):
        self.initialize()
        original = self.bus.write
        self.bus.write = lambda d: 0 if d[2] == 5 and d[1] == 1 else original(d)
        with self.assertRaisesRegex(RuntimeError, "disable request failed"):
            self.check()
        self.assertFalse(self.s.ready)

    def test_arm_and_duplicate_checks_are_blocked_while_pending(self):
        self.initialize()
        self.s.request_recheck()
        with self.assertRaises(RuntimeError):
            self.s.request_recheck()
        with self.assertRaises(RuntimeError):
            self.s.request_arm(True)
        self.assertFalse(self.commands(6))

    def test_disarm_cancels_recheck_before_success_commit(self):
        self.initialize()
        self.s.request_recheck()
        self.s.stop_event.callback = lambda: self.s.request_arm(False)
        with self.assertRaisesRegex(m.UserDisarm, "cancelled"):
            self.s._perform_recheck()
        self.assertFalse(self.s.ready)
        self.assertFalse(self.s.reference_valid)
        self.assertFalse(self.commands(6))

    def test_open_failure_consumes_request_and_does_not_auto_retry(self):
        self.s.phase = "FAULT"
        self.s.serial = None
        attempts = []

        def fail():
            attempts.append(1)
            raise RuntimeError("UART unavailable")

        self.s.factory = fail
        self.s.request_recheck()
        with self.assertRaisesRegex(RuntimeError, "UART unavailable"):
            self.s._perform_recheck()
        self.assertFalse(self.s._recheck_requested())
        self.assertEqual(len(attempts), 1)

    def test_home_keeps_both_axes_enabled_and_holds_zero(self):
        self.arm()
        count = len(self.commands(5))
        self.s.request_home()
        self.s._home_and_clear()
        self.assertEqual(self.s.phase, "HOME_HOLD")
        self.assertEqual(self.s._manual_phase(False), "HOME_HOLD")
        self.assertTrue(all(self.bus.enabled.values()))
        self.assertTrue(self.s.armed)
        self.assertFalse(self.s.home_requested)
        self.assertEqual(self.s.manual_target, (0, 0))
        self.assertEqual(len(self.commands(5)), count)
        self.assertEqual(self.s._manual_phase(True), "MANUAL")

    def test_home_hold_still_faults_and_disables_when_heartbeat_expires(self):
        self.arm()
        self.s.request_home()
        self.s._home_and_clear()
        self.clock.pump = None
        self.s.command_time = self.clock.now-self.c["watchdog_s"]-.1
        self.s.stop_event.callback = lambda: self.s.stop_event.set() if self.s.phase == "FAULT" else None
        # The normal loop enforces the same heartbeat while HOME is holding.
        with self.assertRaisesRegex(RuntimeError, "heartbeat expired"):
            self.s._control_loop()
        self.s._record_fault(RuntimeError("heartbeat expired"))
        self.assertFalse(any(self.bus.enabled.values()))


if __name__ == "__main__":
    unittest.main()
