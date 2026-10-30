"""ARM/home and obstruction behavior on the actual generated worker + model."""
import contextlib
import io
import unittest
from unittest.mock import patch
import test_adapter as adapters
import test_tracker as app

m = app.m
fixtures = adapters.fixtures


class ArmingGuardTests(unittest.TestCase):
    def setUp(self):
        self.clock = fixtures.Clock()
        self.bus = fixtures.MechanicalMotorModel((12., -8.))
        self.driver = m.VisualMotorService(dict(m.CONFIG, startup_auto_home=False), lambda: self.bus)
        self.driver.stop_event = fixtures.Event(self.clock)
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(m.time, "monotonic", self.clock.monotonic))
        self.stack.enter_context(patch.object(m.time, "sleep", self.clock.sleep))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.vector = (0, 0)
        self.clock.pump = lambda:self.driver.input(*self.vector)

    def run_worker(self, on_home=None):
        count, armed, homing, held = [0], [False], [False], []

        def schedule():
            count[0] += 1
            if self.driver.phase == "DISABLED" and self.driver.ready and not armed[0]:
                armed[0] = True
                self.assertFalse(any(d[2] in (3, 6) for d in self.bus.writes))
                self.driver.confirm_reference()
                self.driver.request_arm(True)
            elif self.driver.armed and self.driver.phase == "ARMED" and not homing[0]:
                homing[0] = True
                self.driver.request_home()
            elif self.driver.phase == "HOME_HOLD":
                held.append(self.driver.snapshot())
                if on_home:
                    on_home()
                else:
                    self.driver.stop_event.set()
            if self.driver.phase == "FAULT" or count[0] > 1800:
                self.driver.stop_event.set()

        self.driver.stop_event.callback = schedule
        self.driver._run()
        return held

    def assert_both_disabled(self):
        self.assertFalse(any(self.bus.enabled.values()))
        disables = [d[1] for d in self.bus.writes if d[2] == 5]
        self.assertEqual(set(disables[-2:]), {1, 2})

    def test_arm_preloads_pose_then_homes_and_holds_zero(self):
        held = self.run_worker()
        self.assertIsNone(self.driver.fault)
        self.assertTrue(held)
        self.assertTrue(held[0]["armed"])
        self.assertFalse(held[0]["arm_homing"])
        self.assertTrue(all(abs(v) <= .5 for v in held[0]["angles"]))
        positions = [d for d in self.bus.writes if d[2] == 3]
        self.assertEqual([int.from_bytes(d[3:5], "big") for d in positions[:2]], [120, 3520])
        self.assert_both_disabled()  # Program exit still disables, HOME itself does not.

    def test_pitch_input_is_inverted_without_changing_mechanical_coordinates(self):
        self.driver.serial = self.bus
        self.driver._initialize()
        self.driver.input(0, 1)
        self.assertEqual(self.driver.joystick, (0, -1))
        self.assertEqual(self.driver.angles, (12, -8))

    def test_stuck_during_arm_homing_latches_and_disables(self):
        self.bus.stuck = True
        held = self.run_worker()
        self.assertFalse(held)
        self.assertIn("SUSPECTED_BLOCK", self.driver.fault)
        self.assertIsNotNone(self.driver.guard.last_event)
        self.assertFalse(self.driver.arm_requested)
        self.assert_both_disabled()

    def test_stuck_during_manual_or_tracking_commands_disables(self):
        self.bus.physical = {1: 0, 2: 0}

        def block():
            self.bus.stuck = True
            self.vector = (1, 0)

        held = self.run_worker(block)
        self.assertTrue(held)
        self.assertIn("SUSPECTED_BLOCK", self.driver.fault)
        self.assert_both_disabled()

    def test_disarm_cancels_arm_homing(self):
        count = [0]

        def schedule():
            count[0] += 1
            if self.driver.phase == "DISABLED" and self.driver.ready and count[0] == 1:
                self.driver.confirm_reference()
                self.driver.request_arm(True)
            elif self.driver.armed and self.driver.phase == "ARMED" and not self.driver.home_requested:
                self.driver.request_home()
            elif self.driver.arm_homing:
                self.driver.request_arm(False)
            elif not self.driver.armed and count[0] > 3:
                self.driver.stop_event.set()
            if count[0] > 100:
                self.driver.stop_event.set()

        self.driver.stop_event.callback = schedule
        self.driver._run()
        self.assertIsNone(self.driver.fault)
        self.assert_both_disabled()
        self.assertFalse(self.driver.arm_homing)

    def test_reverse_progress_is_a_suspected_block(self):
        guard = m.TrackingMotionGuard(dict(m.CONFIG))
        guard.update(1., (0., 0.), (3., 0.))
        guard.update(2.6, (-2.2, 0.), (3., 0.))
        self.assertIsNotNone(guard.warning)
        with self.assertRaisesRegex(RuntimeError, "SUSPECTED_BLOCK.*opposite"):
            guard.update(3.1, (-2.2, 0.), (3., 0.))
        self.assertEqual(guard.last_event["actual"], (-2.2, 0.))

    def test_fault_before_home_loop_clears_arming_flags_and_disables(self):
        self.driver.serial = self.bus
        self.driver.armed = self.driver.arm_requested = True
        self.driver.arm_homing = self.driver.home_requested = True
        self.driver._latch_fault("feedback unavailable", reference_lost=True)
        state = self.driver.snapshot()
        self.assertIsNotNone(state["fault"])
        self.assertFalse(state["arm_homing"])
        self.assertFalse(state["home_requested"])
        self.assertFalse(state["armed"])
        self.assert_both_disabled()

    def test_new_tracking_response_exceeds_previous_response_and_stays_bounded(self):
        old = dict(m.CONFIG, ema=(.06, .10), step_cap=(.8, 1), tracking_rate=(12., 8.))
        before, after = m.ClickTracker(old), m.ClickTracker(dict(m.CONFIG))
        tracks = [app.track()]
        for core in (before, after):
            core.select((0, 1), tracks, 1)
            core.step(tracks, 1.1)
        for slow, fast in zip(before.vector, after.vector):
            self.assertGreater(fast, slow*2)
        self.assertLessEqual(m.math.hypot(*after.vector), 1)

