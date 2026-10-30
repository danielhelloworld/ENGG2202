"""PC protocol/physical-state MODELS only, not real motors or Maix hardware."""
import contextlib
import copy
import importlib.util
import io
import sys
import types
from pathlib import Path
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("motor_joystick_bundle", ROOT / "main.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Clock:
    def __init__(self):
        self.now = 1.0
        self.pump = None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.pump:
            self.pump()


class Event:
    def __init__(self, clock):
        self.clock = clock
        self.stopped = False
        self.callback = None

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, seconds):
        self.clock.sleep(seconds)
        if self.callback:
            self.callback()
        return self.stopped


class MotorModel:
    """Position and cumulative origin are independent; clear changes ONLY origin."""
    def __init__(self, positions=(2.0, -3.0)):
        self.physical = dict(zip((1, 2), positions))
        self.origin = dict(self.physical)
        self.enabled = {1: False, 2: False}
        self.target = {1: 0.0, 2: 0.0}
        self.pending = b""
        self.writes = []
        self.stuck = False
        self.reverse = False
        self.ignore_reset = set()
        self.drop_feedback = set()
        self.nonzero_speed = False
        self.drift_on_disable = False
        self.closed = False

    def write(self, data):
        self.writes.append(bytes(data))
        self.assert_frame(data)
        mid, function = data[1:3]
        if function == m.FUNCTION_DISABLE:
            if self.enabled[mid] and self.drift_on_disable:
                self.physical[mid] += 2
            self.enabled[mid] = False
        elif function == m.FUNCTION_ENABLE:
            self.enabled[mid] = True
        elif function == m.FUNCTION_MULTI_TURN_POSITION:
            self.target[mid] = int.from_bytes(data[3:7], "big", signed=True) / 10.0
            if self.enabled[mid] and not self.stuck:
                goal = self.origin[mid] + self.target[mid]
                self.physical[mid] = 2*self.physical[mid]-goal if self.reverse else goal
        elif function == 0x09 and mid not in self.ignore_reset:
            if self.enabled[mid]:
                raise AssertionError("Cumulative reset while enabled")
            self.origin[mid] = self.physical[mid]
        elif function == m.FUNCTION_FEEDBACK and mid not in self.drop_feedback:
            kind = data[3]
            value = (self.physical[mid]-self.origin[mid] if kind == 1 else
                     self.physical[mid] % 360 if kind == 2 else
                     10.0 if self.nonzero_speed else 0.0)
            raw = int(round(value if kind == 0 else value*10))
            self.pending = m._frame(mid, kind, raw.to_bytes(4, "big", signed=True))
        return len(data)

    @staticmethod
    def assert_frame(data):
        if data[0] != 0x7A or data[-1] != 0x7B or m.bcc_xor(data[:-2]) != data[-2]:
            raise AssertionError("Invalid command frame")

    def read(self, size, timeout):
        value, self.pending = self.pending, b""
        return value

    def close(self):
        self.closed = True


class MotorTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(m.time, "monotonic", self.clock.monotonic))
        self.stack.enter_context(patch.object(m.time, "sleep", self.clock.sleep))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.c = copy.deepcopy(m.CONFIG)
        self.c["motor_reference"] = "mechanical"  # Legacy cumulative-mode regressions.
        self.bus = MotorModel()
        self.s = m.JoystickMotorService(self.c, lambda: self.bus)
        self.s.serial = self.bus
        self.s.stop_event = Event(self.clock)

    def initialize(self):
        self.s.sent_target = self.s.manual_target = self.s._initialize()
        self.s.ready = True
        self.s.phase = "DISABLED"
        self.s.last_send = self.clock.now
        self.s.input(0, 0)

    def arm(self):
        self.initialize()
        self.s.confirm_reference()
        self.s.request_arm(True)
        self.s.sent_target = self.s.manual_target = self.s._arm_current()
        self.s.armed = True
        self.s.phase = "ARMED"
        self.s.last_send = self.clock.now
        self.clock.pump = lambda: self.s.input(0, 0)

    def commands(self, function):
        return [data for data in self.bus.writes if data[2] == function]

    def test_connection_only_disables_and_reads_without_redefining_mechanical_zero(self):
        self.initialize()
        self.assertFalse(self.s.armed)
        self.assertEqual(self.s.angles, (2, -3))
        self.assertTrue(self.s.reference_valid)
        self.assertEqual(set(data[2] for data in self.bus.writes), {0x05, 0x0E})

    def test_arm_requires_user_confirmation_and_preloads_current_pose(self):
        self.initialize()
        with self.assertRaises(RuntimeError):
            self.s.request_arm(True)
        self.s.confirm_reference()
        self.s.request_arm(True)
        self.s._arm_current()
        functions = [data[2] for data in self.bus.writes]
        first_enable = functions.index(0x06)
        self.assertEqual(sum(f == 0x02 for f in functions[:first_enable]), 2)
        self.assertEqual(tuple(self.bus.physical.values()), (2, -3))

    def test_home_clears_only_after_both_disabled_and_rebuilds_reference(self):
        self.arm()
        self.s._home_and_clear()
        self.assertEqual(self.s.phase, "HOME_DONE_DISABLED")
        self.assertTrue(self.s.reference_valid)
        self.assertFalse(self.s.armed or self.s.arm_requested)
        self.assertTrue(all(abs(v) <= 0.2 for v in self.s.raw_feedback))
        self.assertTrue(all(abs(v) <= 0.5 for v in self.bus.physical.values()))
        functions = [data[2] for data in self.bus.writes]
        first_reset = functions.index(0x09)
        self.assertEqual(len(self.commands(0x09)), 2)
        self.assertFalse(any(f in (0x02, 0x06, 0x0A, 0x08) for f in functions[first_reset:]))
        before = self.bus.writes[:first_reset]
        for mid in (1, 2):
            last_nonquery = [d for d in before if d[1] == mid and d[2] != 0x0E][-1]
            self.assertEqual(last_nonquery[2], 0x05)

    def test_ignored_first_reset_blocks_second_axis_and_new_motion(self):
        self.arm()
        self.bus.ignore_reset = {1}
        with self.assertRaisesRegex(RuntimeError, "readback failed"):
            self.s._home_and_clear()
        self.assertFalse(self.s.reference_valid)
        self.assertEqual([d[1] for d in self.commands(0x09)], [1])
        with self.assertRaises(RuntimeError):
            self.s._write(m.enable(1))

    def test_stall_home_timeout_does_not_clear(self):
        self.arm()
        self.bus.stuck = True
        self.c["home_timeout_s"] = 0.3
        with self.assertRaisesRegex(RuntimeError, "Home timeout"):
            self.s._home_and_clear()
        self.assertFalse(self.commands(0x09))

    def test_unsettled_motor_does_not_clear(self):
        self.arm()
        self.bus.nonzero_speed = True
        self.c["home_timeout_s"] = 1.0
        with self.assertRaisesRegex(RuntimeError, "Home timeout"):
            self.s._home_and_clear()
        self.assertFalse(self.commands(0x09))

    def test_gravity_movement_after_disable_blocks_clear(self):
        self.arm()
        self.bus.drift_on_disable = True
        with self.assertRaises(RuntimeError):
            self.s._home_and_clear()
        self.assertFalse(self.commands(0x09))

    def test_ui_deadman_and_user_stop_block_motion_and_reset(self):
        self.arm()
        self.clock.pump = None
        self.clock.sleep(1)
        with self.assertRaisesRegex(RuntimeError, "heartbeat"):
            self.s._advance((0, 0), self.c["home_rate"])
        self.s.request_arm(False)
        with self.assertRaises(m.UserDisarm):
            self.s._write(m.enable(1))
        self.assertFalse(self.commands(0x09))

    def test_zero_discontinuity_latches_and_attempts_both_axis_disable(self):
        self.arm()
        self.bus.origin[1] += 360
        with self.assertRaisesRegex(RuntimeError, "ZERO_REFERENCE_LOST"):
            self.s._measure()
        self.assertFalse(self.s.reference_valid)
        self.assertFalse(self.s.armed)
        self.assertEqual([d[1] for d in self.commands(0x05)][-2:], [1, 2])

    def test_reset_opcode_not_available_outside_home_transaction(self):
        self.initialize()
        with self.assertRaises(RuntimeError):
            self.s._write(m._frame(1, 0x09))
        self.assertFalse(self.commands(0x09))

    def test_cannot_rearm_during_disabled_reset_stage(self):
        self.initialize()
        self.s.confirm_reference()
        for phase in ("CHECK_DISABLED", "CLEARING", "VERIFYING"):
            self.s.phase = phase
            with self.assertRaises(RuntimeError):
                self.s.request_arm(True)

    def test_full_worker_model_arm_home_clear_and_close(self):
        steps = [0]
        self.clock.pump = lambda: self.s.input(0, 0)

        def schedule():
            steps[0] += 1
            if self.s.phase == "DISABLED" and self.s.ready:
                self.s.confirm_reference()
                self.s.request_arm(True)
            elif self.s.phase == "ARMED":
                self.s.request_home()
            elif self.s.phase in ("HOME_DONE_DISABLED", "FAULT") or steps[0] > 1000:
                self.s.stop_event.set()

        self.s.stop_event.callback = schedule
        self.s._run()
        self.assertEqual(self.s.phase, "HOME_DONE_DISABLED")
        self.assertIsNone(self.s.fault)
        self.assertTrue(self.bus.closed)
        self.assertEqual(len(self.commands(0x09)), 2)
        self.assertFalse(any(self.bus.enabled.values()))

    def test_stop_during_clear_invalidates_reference_and_blocks_second_clear(self):
        self.arm()
        original = self.bus.write

        def interrupted(data):
            result = original(data)
            if data[2] == 0x09:
                self.s.request_arm(False)
            return result

        self.bus.write = interrupted
        with self.assertRaises(m.UserDisarm):
            self.s._home_and_clear()
        self.assertFalse(self.s.reference_valid)
        self.assertEqual(len(self.commands(0x09)), 1)

    def test_both_disable_attempted_even_when_first_write_fails(self):
        self.initialize()
        original = self.bus.write
        attempts = []

        def fail_first(data):
            attempts.append((data[1], data[2]))
            if data[1] == 1 and data[2] == 0x05:
                return 0
            return original(data)

        self.bus.write = fail_first
        errors = self.s._disable_all()
        self.assertEqual(attempts, [(1, 0x05), (2, 0x05)])
        self.assertEqual(len(errors), 1)

    def test_manual_worker_clamps_at_bounds_and_release_holds(self):
        self.bus.physical = {1: 149.8, 2: 29.8}
        self.bus.origin = dict(self.bus.physical)
        vector = [(0, 0)]
        steps = [0]
        released = []
        self.clock.pump = lambda: self.s.input(*vector[0])

        def schedule():
            steps[0] += 1
            if self.s.phase == "DISABLED" and self.s.ready:
                self.s.confirm_reference()
                self.s.request_arm(True)
            elif self.s.phase in ("ARMED", "MANUAL"):
                if steps[0] < 25:
                    vector[0] = (1, 1)
                else:
                    vector[0] = (0, 0)
                    released.append(tuple(self.bus.physical.values()))
            if steps[0] >= 40 or self.s.phase == "FAULT":
                self.s.stop_event.set()

        self.s.stop_event.callback = schedule
        self.s._run()
        self.assertIsNone(self.s.fault)
        self.assertTrue(released)
        for axis, lim in enumerate(self.c["limits"]):
            self.assertTrue(all(lim[0] <= p[axis] <= lim[1] for p in released))
        self.assertEqual(released[-1], released[-2])
        self.assertFalse(self.commands(0x09))

    def test_full_worker_heartbeat_failure_latches_and_disables(self):
        self.initialize()
        self.s.zero_confirmed = True
        self.s.arm_requested = True
        # No UI heartbeat; let the worker's clock advance until it faults.
        self.s.stop_event.callback = lambda: self.s.stop_event.set() if self.s.phase == "FAULT" else None
        self.s._run()
        self.assertIn("heartbeat", self.s.fault)
        self.assertFalse(any(self.bus.enabled.values()))
        self.assertFalse(self.commands(0x09))


class MechanicalMotorModel(MotorModel):
    def write(self, data):
        if data[2] in (0x02, 0x08, 0x09, 0x0A) or (data[2] == 0x0E and data[3] == 1):
            raise AssertionError("Mechanical-only model rejects cumulative/calibration commands")
        if data[2] == 0x03:
            self.assert_frame(data)
            self.writes.append(bytes(data))
            mid = data[1]
            angle = int.from_bytes(data[3:5], "big") / 10.0
            self.target[mid] = angle
            if self.enabled[mid] and not self.stuck:
                delta = m.signed_mechanical(angle-self.physical[mid])
                self.physical[mid] += -delta if self.reverse else delta
            return len(data)
        return super().write(data)


class MechanicalOnlyTests(unittest.TestCase):
    initialize = MotorTests.initialize
    arm = MotorTests.arm
    commands = MotorTests.commands

    def setUp(self):
        MotorTests.setUp(self)
        self.c["motor_reference"] = "mechanical_only"
        self.bus = MechanicalMotorModel((93.9, -1.1))
        self.bus.origin = {1: 5000, 2: -1000}  # Deliberately unrelated cumulative state.
        self.s = m.MechanicalOnlyService(self.c, lambda: self.bus)
        self.s.serial = self.bus
        self.s.stop_event = Event(self.clock)

    def test_screenshot_pose_initializes_despite_unrelated_cumulative_origin(self):
        self.initialize()
        self.assertAlmostEqual(self.s.angles[0], 93.9)
        self.assertAlmostEqual(self.s.angles[1], -1.1)
        self.assertEqual(self.s.raw_feedback, (None, None))
        self.assertTrue(self.s.reference_valid)
        self.assertFalse(self.s.snapshot()["cumulative_angle_used"])
        self.assertTrue(all(d[3] == 2 for d in self.commands(0x0E)))

    def test_arm_uses_single_turn_mode_and_current_mechanical_target(self):
        self.arm()
        self.assertEqual([int.from_bytes(d[3:5], "big") for d in self.commands(0x00)], [4, 4])
        target = self.commands(0x03)
        self.assertEqual(int.from_bytes(target[0][3:5], "big"), 939)
        self.assertEqual(int.from_bytes(target[1][3:5], "big"), 3589)
        self.assertFalse(self.commands(0x02))
        self.assertEqual(tuple(self.bus.physical.values()), (93.9, -1.1))

    def test_full_worker_home_uses_only_mechanical_feedback_and_never_clears(self):
        self.clock.pump = lambda: self.s.input(0, 0)
        count = [0]

        def schedule():
            count[0] += 1
            if self.s.phase == "DISABLED" and self.s.ready:
                self.s.confirm_reference()
                self.s.request_arm(True)
            elif self.s.phase == "ARMED":
                self.s.request_home()
            elif self.s.phase in ("HOME_HOLD", "FAULT") or count[0] > 4000:
                self.s.stop_event.set()

        self.s.stop_event.callback = schedule
        self.s._run()
        self.assertIsNone(self.s.fault)
        self.assertEqual(self.s.phase, "HOME_HOLD")
        self.assertTrue(self.bus.closed)
        self.assertFalse(any(self.bus.enabled.values()))
        self.assertFalse(any(d[2] in (0x02, 0x08, 0x09, 0x0A) for d in self.bus.writes))
        self.assertTrue(all(d[3] in (0, 2) for d in self.commands(0x0E)))
        self.assertTrue(all(abs(v) <= 0.5 for v in self.bus.physical.values()))

    def test_cumulative_changes_do_not_affect_control(self):
        self.arm()
        self.bus.origin = {1: -12345, 2: 67890}
        self.s._measure()
        self.assertTrue(self.s.reference_valid)
        self.assertIsNone(self.s.fault)

    def test_actual_mechanical_jump_still_latches_and_disables(self):
        self.arm()
        self.bus.physical[1] += 10
        with self.assertRaisesRegex(RuntimeError, "mechanical angle jump"):
            self.s._measure()
        self.assertFalse(self.s.reference_valid)
        self.assertFalse(any(self.bus.enabled.values()))

    def test_disabled_manual_reposition_inside_limits_does_not_use_motor_rpm(self):
        self.initialize()
        self.bus.physical[2] = 20
        self.clock.sleep(0.02)
        self.s._measure()
        self.assertEqual(self.s.angles[1], 20)
        self.assertTrue(self.s.reference_valid)
        self.assertIsNone(self.s.last_angle_jump)
        self.assertFalse(self.commands(0x06))
        # Manual ARM must hold this new measured pose, rather than the old one.
        self.s.confirm_reference()
        self.s.request_arm(True)
        self.s._arm_current()
        self.assertEqual(int.from_bytes(self.commands(0x03)[-1][3:5], "big"), 200)

    def test_disabled_reposition_outside_limits_still_faults(self):
        self.initialize()
        self.bus.physical[2] = 116.2
        with self.assertRaisesRegex(RuntimeError, "outside calibrated limits"):
            self.s._measure()
        self.assertFalse(self.s.reference_valid)
        self.assertFalse(self.commands(0x06))

    def test_jump_during_arm_preparation_blocks_enable_and_has_evidence(self):
        self.initialize()
        self.s.confirm_reference()
        self.s.request_arm(True)
        self.bus.physical[2] = 20
        with self.assertRaisesRegex(RuntimeError, "PITCH ID=2"):
            self.s._arm_current()
        event = self.s.snapshot()["last_angle_jump"]
        self.assertEqual(event["motor_id"], 2)
        self.assertAlmostEqual(event["previous_deg"], -1.1)
        self.assertEqual(event["current_deg"], 20)
        self.assertFalse(event["armed"])
        self.assertTrue(event["arm_requested"])
        self.assertGreater(event["delta_deg"], event["allowed_deg"])
        self.assertFalse(self.commands(0x06))

    def test_armed_jump_records_evidence_and_requests_both_disables(self):
        self.arm()
        self.bus.physical[2] = 20
        before = len(self.bus.writes)
        with self.assertRaisesRegex(RuntimeError, "delta=.*armed=True"):
            self.s._measure()
        self.assertEqual({d[1] for d in self.bus.writes[before:] if d[2] == 5}, {1, 2})
        self.assertTrue(self.s.snapshot()["last_angle_jump"]["armed"])
        self.assertFalse(self.s.reference_valid)

    def test_single_turn_large_shortest_path_jump_is_refused(self):
        self.arm()
        count = len(self.commands(0x03))
        with self.assertRaisesRegex(RuntimeError, "Unsafe single-turn step"):
            self.s._write(m.set_single_turn_position(1, -100))
        self.assertEqual(len(self.commands(0x03)), count)

    def test_steps_from_positive_to_negative_target_stay_local(self):
        self.arm()
        previous = self.bus.physical[1]
        for _ in range(30):
            self.clock.sleep(0.02)
            self.s._measure()
            self.s._advance((-100, 0), self.c["home_rate"])
            current = self.bus.physical[1]
            self.assertLessEqual(abs(current-previous), self.c["manual_lead_deg"] + 0.051)
            self.assertLessEqual(current, previous + 0.001)
            previous = current
        self.assertGreater(self.bus.physical[1], 0)  # No direct 194-degree jump.

    def test_cumulative_and_calibration_frames_are_rejected_at_final_gate(self):
        self.arm()
        frames = (m.set_multi_turn_position(1, 0), m.request_feedback(1, 1),
                  m._frame(1, 0x09), m.set_mechanical_zero_frame(1), m.save_parameters_frame(1))
        for frame in frames:
            with self.assertRaisesRegex(RuntimeError, "forbidden"):
                self.s._write(frame)

    def test_missing_mechanical_feedback_prevents_enable(self):
        self.bus.drop_feedback = {2}
        with self.assertRaises(RuntimeError):
            self.initialize()
        self.assertFalse(self.commands(0x06))

    def test_initial_disable_failure_is_not_ignored(self):
        original = self.bus.write
        self.bus.write = lambda d: 0 if d[1] == 1 and d[2] == 5 else original(d)
        with self.assertRaisesRegex(RuntimeError, "Initial disable request failed"):
            self.initialize()
        self.assertFalse(self.commands(0x06))

    def test_mechanical_limit_still_prevents_arming(self):
        self.bus.physical[2] = 101
        with self.assertRaisesRegex(RuntimeError, "Mechanical position outside"):
            self.initialize()
        self.assertFalse(self.commands(0x06))

    def test_reported_116_degree_fault_keeps_diagnostics_without_fake_control_age(self):
        self.bus.physical = {1: 7.7, 2: 116.2}
        with self.assertRaisesRegex(RuntimeError, r"PITCH ID=2 116.2 deg, allowed -100.0..30.0"):
            self.initialize()
        snap = self.s.snapshot()
        self.assertEqual(snap["mechanical_angles"], (7.7, 116.2))
        self.assertFalse(snap["reference_valid"])
        self.assertFalse(snap["control_feedback_seen"])
        self.assertIsNone(snap["age"])
        self.assertFalse(self.commands(0x06))

    def test_diagnostics_do_not_refresh_accepted_control_feedback(self):
        self.initialize()
        self.clock.sleep(1)
        age_before = self.s.snapshot()["age"]
        self.s._read_diagnostics()
        snap = self.s.snapshot()
        self.assertGreaterEqual(snap["age"], age_before)
        self.assertLess(snap["mechanical_feedback_age"][1], 0.01)

    def test_failed_diagnostic_read_does_not_refresh_its_age(self):
        self.initialize()
        self.bus.drop_feedback = {2}
        self.clock.sleep(1)
        self.s._read_diagnostics()
        snap = self.s.snapshot()
        self.assertGreaterEqual(snap["mechanical_feedback_age"][1], 1)
        self.assertLess(snap["mechanical_feedback_age"][0], snap["mechanical_feedback_age"][1])

    def test_after_disable_speed_fault_includes_id_and_rpm_and_does_not_enable(self):
        self.bus.physical = {1: 0, 2: 0}
        self.initialize()
        original = self.bus.write

        def speed_on_pitch(data):
            result = original(data)
            if data[1] == 2 and data[2] == 0x0E and data[3] == 0:
                self.bus.pending = m._frame(2, 0, (1).to_bytes(4, "big", signed=True))
            return result

        self.bus.write = speed_on_pitch
        with self.assertRaisesRegex(RuntimeError, "Axis moving after disable: ID=2 speed=1 RPM"):
            self.s._stationary_home()
        self.assertFalse(self.commands(6))

    def test_speed_feedback_has_integer_rpm_units(self):
        self.bus.nonzero_speed = True
        self.assertEqual(self.s._feedback(1, m.FEEDBACK_SPEED), 10)


class JoystickTests(unittest.TestCase):
    def test_capture_deadzone_release_and_circular_limit(self):
        ui = m.JoystickUI(m.CONFIG)
        self.assertEqual(ui.touch(175, 235, True, 0)[0], (0, 0))
        vector, _ = ui.touch(400, 10, True, 0.1)
        self.assertLessEqual(m.math.hypot(*vector), 1.000001)
        self.assertEqual(ui.touch(400, 10, False, 0.2)[0], (0, 0))

    def test_hold_deadman_requires_release_and_press(self):
        ui = m.JoystickUI(m.CONFIG)
        self.assertGreater(ui.touch(270, 235, True, 0)[0][0], 0)
        self.assertEqual(ui.touch(270, 235, True, 9)[0], (0, 0))
        self.assertTrue(ui.blocked)
        ui.touch(270, 235, False, 9.1)
        self.assertGreater(ui.touch(270, 235, True, 9.2)[0][0], 0)

    def test_buttons_do_not_repeat_home_or_capture_joystick_when_dragging(self):
        ui = m.JoystickUI(m.CONFIG)
        self.assertEqual(ui.touch(450, 410, True, 0)[1], "home")
        self.assertIsNone(ui.touch(450, 410, True, 0.1)[1])
        self.assertEqual(ui.touch(250, 235, True, 0.2)[0], (0, 0))
        self.assertEqual(ui.touch(160, 410, True, 0.3)[1], "stop")

    def test_pitch_limit_and_nonfinite_targets_rejected(self):
        c = copy.deepcopy(m.CONFIG)
        c["limits"] = ((-150, 150), (-101, 100))
        with self.assertRaises(ValueError):
            m.validate_config(c)
        service = m.JoystickMotorService(m.CONFIG)
        with self.assertRaises(ValueError):
            service.input(float("nan"), 0)

    def test_render_uses_maix_image_calls_without_accessing_motor_or_camera(self):
        labels, shown = [], []

        class Image:
            def __init__(self, *args, **kwargs):
                pass

            def draw_string(self, x, y, text, *args, **kwargs):
                self.assert_xy(x, y)
                labels.append(text)

            @staticmethod
            def assert_xy(x, y):
                if not 0 <= x < 640 or not 0 <= y < 480:
                    raise AssertionError("Label outside screen")

            def draw_rect(self, *args):
                pass

            draw_circle = draw_line = draw_rect

        maix = types.ModuleType("maix")
        maix.image = types.SimpleNamespace(Image=Image, Format=types.SimpleNamespace(FMT_RGB888=0),
                                          Color=types.SimpleNamespace(from_rgb=lambda *args: args))
        service = m.JoystickMotorService(m.CONFIG)
        with patch.dict(sys.modules, {"maix": maix}):
            m.draw_screen(types.SimpleNamespace(show=lambda img: shown.append(img)), service.snapshot(), m.JoystickUI(m.CONFIG))
        self.assertTrue(shown)
        self.assertTrue(all(label in labels for label in ("ARM", "DISARM", "ZERO OK", "HOME")))


if __name__ == "__main__":
    unittest.main()
