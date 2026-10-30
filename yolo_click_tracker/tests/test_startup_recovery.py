"""Out-of-work-range startup is home-only, using the generated driver + MODEL."""
import contextlib
import io
import unittest
from unittest.mock import patch
import test_adapter as adapters
import test_tracker as app

m, fixtures = app.m, adapters.fixtures


class StartupRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.clock = fixtures.Clock()
        self.bus = fixtures.MechanicalMotorModel((165., 116.2))
        self.driver = m.VisualMotorService(dict(m.CONFIG), lambda:self.bus)
        self.driver.stop_event = fixtures.Event(self.clock)
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(m.time, "monotonic", self.clock.monotonic))
        self.stack.enter_context(patch.object(m.time, "sleep", self.clock.sleep))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.clock.pump = lambda:self.driver.input(0, 0)

    def run_startup(self):
        held, count = [], [0]

        def schedule():
            count[0] += 1
            if self.driver.phase == "HOME_HOLD":
                held.append(self.driver.snapshot())
                self.driver.stop_event.set()
            if self.driver.phase == "FAULT" or count[0] > 6000:
                self.driver.stop_event.set()

        self.driver.stop_event.callback = schedule
        self.driver._run()  # No ZERO OK / ARM call: initialization itself homes.
        return held

    def prepare_recovery(self):
        self.driver.serial = self.bus
        self.driver._initialize()
        self.driver.ready, self.driver.phase = True, "DISABLED"
        self.driver.last_send = self.clock.now
        self.driver.sent_target = self.driver._arm_current()
        self.driver.armed = True

    def test_startup_outside_both_limits_homes_without_operator_arm(self):
        held = self.run_startup()
        self.assertIsNone(self.driver.fault)
        self.assertTrue(held)
        self.assertTrue(held[0]["startup_completed"])
        self.assertTrue(held[0]["armed"])
        self.assertFalse(held[0]["recovery_active"])
        self.assertTrue(all(abs(v) <= .5 for v in held[0]["angles"]))
        self.assertEqual(self.driver.c["limits"], ((-150., 150.), (-100., 30.)))
        for mid, start in ((1, 165.), (2, 116.2)):
            targets = [m.signed_mechanical(int.from_bytes(d[3:5], "big")/10.)
                       for d in self.bus.writes if d[2] == 3 and d[1] == mid]
            self.assertAlmostEqual(targets[0], start)
            self.assertTrue(all(-.001 <= q <= start+.001 for q in targets))
            lead = self.driver.c["home_position_lead_deg"][mid-1]
            self.assertTrue(all(-.001 <= a-b <= lead+.051 for a,b in zip(targets,targets[1:])))

    def test_stuck_outside_limits_disables_and_does_not_complete_startup(self):
        self.bus.stuck = True
        self.assertFalse(self.run_startup())
        self.assertIn("SUSPECTED_BLOCK", self.driver.fault)
        self.assertFalse(any(self.bus.enabled.values()))
        self.assertFalse(self.driver.startup_completed)
        self.assertFalse(self.driver.arm_requested)

    def test_negative_outside_startup_also_returns_toward_zero(self):
        self.bus.physical = {1: -165., 2: -120.}
        held = self.run_startup()
        self.assertIsNone(self.driver.fault)
        self.assertTrue(held)
        self.assertTrue(all(abs(q) <= .5 for q in held[0]["angles"]))
        for mid in (1, 2):
            targets = [m.signed_mechanical(int.from_bytes(d[3:5], "big")/10.)
                       for d in self.bus.writes if d[2] == 3 and d[1] == mid]
            lead = self.driver.c["home_position_lead_deg"][mid-1]
            self.assertTrue(all(-.001 <= b-a <= lead+.051 for a,b in zip(targets,targets[1:])))

    def test_recovery_final_gate_rejects_outward_jump_and_zero_crossing(self):
        self.prepare_recovery()
        before = len(self.bus.writes)
        for q in (117., 116.2-self.driver.c["home_position_lead_deg"][1]-.2, -1.):
            with self.assertRaisesRegex(RuntimeError, "Unsafe recovery step"):
                self.driver._write(m.set_single_turn_position(2, q))
        self.assertEqual(len(self.bus.writes), before)
        self.driver._disable_all()

    def test_normal_target_gate_is_restored_after_home(self):
        self.prepare_recovery()
        self.driver._home_and_clear()
        self.assertFalse(self.driver.recovery_active)
        self.driver.angles = (0., 29.9)
        before = len(self.bus.writes)
        with self.assertRaisesRegex(RuntimeError, "Unsafe single-turn step"):
            self.driver._write(m.set_single_turn_position(2, 30.1))
        self.assertEqual(len(self.bus.writes), before)
        self.driver._disable_all()

    def test_half_turn_representation_can_return_locally_without_360_jump(self):
        self.bus.physical = {1: 0., 2: 180.}
        held = self.run_startup()
        self.assertIsNone(self.driver.fault)
        self.assertTrue(held)
        self.assertAlmostEqual(held[0]["angles"][1], 0., delta=.5)
        targets = [m.signed_mechanical(int.from_bytes(d[3:5], "big")/10.)
                   for d in self.bus.writes if d[2] == 3 and d[1] == 2]
        self.assertAlmostEqual(targets[0], -180.)
        self.assertTrue(all(-.001 <= b-a <= self.driver.c["home_position_lead_deg"][1]+.051 for a,b in zip(targets,targets[1:])))

    def test_disarm_cancels_startup_and_does_not_automatically_retry(self):
        count, requested = [0], [False]

        def schedule():
            count[0] += 1
            if self.driver.arm_homing:
                requested[0] = True
                self.driver.request_arm(False)
            elif requested[0] and self.driver.phase == "DISABLED":
                self.driver.stop_event.set()
            if count[0] > 100:
                self.driver.stop_event.set()

        self.driver.stop_event.callback = schedule
        self.driver._run()
        self.assertTrue(requested[0])
        self.assertFalse(self.driver.arm_requested)
        self.assertFalse(self.driver.startup_completed)
        self.assertFalse(any(self.bus.enabled.values()))

    def test_missing_feedback_never_enables_startup_motion(self):
        self.bus.drop_feedback = {2}
        self.assertFalse(self.run_startup())
        self.assertIsNotNone(self.driver.fault)
        self.assertFalse(any(d[2] == 6 for d in self.bus.writes))

    def test_outside_position_can_be_rechecked_without_motion(self):
        self.driver.serial = self.bus
        self.driver.phase = "FAULT"
        self.driver.fault = "previous fault"
        self.driver.request_recheck()
        self.driver._perform_recheck()
        state = self.driver.snapshot()
        self.assertTrue(state["ready"])
        self.assertTrue(state["outside_work_limits"])
        self.assertIsNone(state["fault"])
        self.assertFalse(self.driver.arm_requested)
        self.assertFalse(any(d[2] in (3, 6) for d in self.bus.writes))

