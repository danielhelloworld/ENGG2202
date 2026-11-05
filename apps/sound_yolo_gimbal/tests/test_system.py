"""PC-only state, protocol, fault and native-adapter contract tests. No hardware."""
import copy
import importlib.util
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("sound_yolo_bundle", ROOT / "main.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def config():
    c = copy.deepcopy(m.CONFIG)
    c.update(sound_calibrated=True, sound_samples=[(-.7, .2, -35, 0), (0, .2, 0, 0), (.7, .2, 35, 0)])
    return c


def sound(frame, vector=(.7, .2), **kw):
    data = dict(valid=True, quality=.9, age_seconds=0, vector_xy=vector, frames=frame)
    data.update(kw)
    return data


def track(key=(0, 0), cx=.5, cy=.5):
    return dict(key=key, cx=cx, cy=cy, score=.9)


class StateTests(unittest.TestCase):
    def setUp(self):
        self.c = config()
        self.controller = m.SoundVisualController(self.c)

    def to_search(self):
        ctl = self.controller
        ctl.step(0, sound(1), [], (10, 0))
        ctl.step(.4, sound(2), [], (10, 0))
        self.assertEqual(ctl.state, "SLEW")
        self.assertEqual(ctl.target, (35, 0))  # NOT current yaw 10 + 35.
        ctl.step(.5, sound(3), [], (35, 0))
        ctl.step(.9, sound(4), [], (35, 0))
        self.assertEqual(ctl.state, "SEARCH")

    def test_full_sound_visual_loss_cycle(self):
        self.to_search()
        for now in (1, 1.1, 1.2):
            self.controller.step(now, sound(4), [track()], (35, 0))
        self.assertEqual(self.controller.selected, (0, 0))  # ID zero is valid.
        self.controller.step(1.3, sound(5, (-.7, .2)), [track()], (35, 0))
        self.assertEqual(self.controller.target, (35, 0))  # Sound cannot steal a lock.
        self.controller.step(1.4, sound(6), [track((0, 99))], (35, 0))
        self.assertEqual(self.controller.state, "LOST")
        self.assertEqual(self.controller.selected, (0, 0))
        self.controller.step(3.5, sound(6), [], (35, 0))
        self.assertEqual(self.controller.state, "WAIT_SOUND")
        self.controller.step(4, sound(6), [], (35, 0))
        self.assertEqual(self.controller.state, "WAIT_SOUND")

    def test_no_calibration_no_motion(self):
        self.c["sound_calibrated"] = False
        for i in range(10):
            self.assertEqual(self.controller.step(i, sound(i), [], (12, 2)), (12, 2))

    def test_repeated_frame_not_stable_evidence(self):
        for i in range(10):
            self.controller.step(i, sound(1), [], (0, 0))
        self.assertEqual(self.controller.state, "WAIT_SOUND")

    def test_stale_weak_ambiguous_outside_sound(self):
        for data in (sound(1, age_seconds=1), sound(1, quality=.1), sound(1, valid=False), sound(1, (0, -.8))):
            self.assertIsNone(m.sound_to_angles(data, self.c))

    def test_stability_resets_on_invalid(self):
        self.controller.step(0, sound(1), [], (0, 0))
        self.controller.step(.3, sound(2, valid=False), [], (0, 0))
        self.controller.step(.4, sound(3), [], (0, 0))
        self.assertEqual(self.controller.state, "WAIT_SOUND")

    def test_settle_requires_continuous_measured_arrival(self):
        self.controller.step(0, sound(1), [], (0, 0))
        self.controller.step(.4, sound(2), [], (0, 0))
        self.controller.step(.5, sound(3), [], (35, 0))
        self.controller.step(.7, sound(4), [], (20, 0))
        self.controller.step(.9, sound(5), [], (35, 0))
        self.assertEqual(self.controller.state, "SLEW")

    def test_turn_timeout_fault(self):
        self.controller.step(0, sound(1), [], (0, 0))
        self.controller.step(.4, sound(2), [], (0, 0))
        self.assertIsNone(self.controller.step(20, sound(3), [], (0, 0)))
        self.assertEqual(self.controller.state, "FAULT")

    def test_ambiguous_people_need_manual_select(self):
        self.to_search()
        boxes = [track((0, 1), .48), track((0, 2), .52)]
        for now in (1, 1.1, 1.2, 1.3):
            self.controller.step(now, sound(4), boxes, (35, 0))
        self.assertIsNone(self.controller.selected)
        self.assertTrue(self.controller.select((0, 2), boxes, 1.4, (35, 0)))

    def test_cached_vision_not_confirmation_frames(self):
        self.to_search()
        for now in (1, 1.1, 1.2):
            self.controller.step(now, sound(4), [track()], (35, 0), new_vision=False)
        self.assertIsNone(self.controller.selected)

    def test_visual_deadzone_and_bounded_motion(self):
        self.controller.select((0, 0), [track()], 0, (35, 0))
        self.assertEqual(self.controller.step(.1, sound(1), [track()], (35, 0)), (35, 0))
        target = self.controller.step(.2, sound(2), [track(cx=.99, cy=.99)], (35, 0))
        self.assertTrue(35 < target[0] <= 35.8)
        self.assertTrue(-1 <= target[1] < 0)

    def test_same_id_reappears_before_timeout(self):
        self.controller.select((0, 0), [track()], 0, (0, 0))
        self.controller.step(.1, sound(1), [], (0, 0))
        self.controller.step(1.5, sound(2), [track()], (0, 0))
        self.assertEqual(self.controller.state, "TRACK")

    def test_search_sweeps_both_sides_then_waits(self):
        self.to_search()
        right = self.controller.step(2.4, sound(5), [], (35, 0))[0]
        left = self.controller.step(5.4, sound(6), [], (35, 0))[0]
        self.assertGreater(right, 35)
        self.assertLess(left, 35)
        self.controller.step(7, sound(7), [], (35, 0))
        self.assertEqual(self.controller.state, "WAIT_SOUND")

    def test_pitch_hard_limit_validation(self):
        self.c["limits"] = ((-150, 150), (-101, 60))
        with self.assertRaises(ValueError):
            m.validate_config(self.c)

    def test_real_motion_requires_calibration_and_ports(self):
        for key, value in (("motor_calibrated", False), ("sound_calibrated", False), ("motor_port", "/dev/ttyS1"), ("motor_port", "/dev/ttyS4")):
            c = config()
            c.update(motor_enabled=True, motor_calibrated=True)
            c[key] = value
            with self.assertRaises(ValueError):
                m.validate_config(c)

    def test_touch_letterbox_mapping(self):
        self.assertEqual(m.touch_to_image(400, 240, 640, 480, 800, 480), (320, 240))
        self.assertLess(m.touch_to_image(10, 240, 640, 480, 800, 480)[0], 0)


class FakeUART:
    def __init__(self, missing=(), fail_enable=None):
        self.pending = bytearray()
        self.writes = []
        self.missing = missing
        self.fail_enable = fail_enable
        self.closed = False
        self.totals = {1: 10.0, 2: -20.0}
        self.mechanical = {1: 0.0, 2: 0.0}
        self.ignore_zero = False

    def write(self, data):
        self.writes.append(data)
        mid, command = data[1:3]
        if command == m.FUNCTION_ENABLE and mid == self.fail_enable:
            raise OSError("fake second enable failure")
        if command == m.FUNCTION_FEEDBACK and mid not in self.missing:
            kind = data[3]
            value = self.totals[mid] if kind == 1 else (self.mechanical[mid] if kind == 2 else 0)
            body = bytes((0x7A, mid, kind)) + round(value*10).to_bytes(4, "big", signed=True)
            self.pending.extend(body + bytes((m.bcc_xor(body), 0x7B)))
        if command == m.FUNCTION_MECHANICAL_ZERO and not self.ignore_zero:
            self.mechanical[mid] = 0
        return len(data)

    def read(self, size, timeout=2):
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data

    def close(self):
        self.closed = True


class MotorTests(unittest.TestCase):
    def service(self, serial):
        c = config()
        c.update(motor_enabled=True, motor_calibrated=True, motor_raw_zero=(10, -20), feedback_timeout=.025, watchdog_s=.12)
        return m.MotorService(c, lambda: serial)

    def await_stop(self, service):
        deadline = time.monotonic()+2
        while not service.snapshot()["fault"] and service.thread.is_alive() and time.monotonic()<deadline:
            time.sleep(.005)
        self.assertIsNotNone(service.snapshot()["fault"])
        service.close()
        self.assertFalse(service.thread.is_alive())

    def arm(self, service):
        deadline = time.monotonic()+1
        while not service.snapshot()["ready"] and time.monotonic()<deadline:
            time.sleep(.005)
        self.assertTrue(service.snapshot()["ready"])
        service.request_arm(True)

    def test_missing_feedback_never_enables_and_closes(self):
        serial = FakeUART(missing=(2,))
        service = self.service(serial)
        service.start()
        self.await_stop(service)
        self.assertFalse(any(d[2] == m.FUNCTION_ENABLE for d in serial.writes))
        self.assertTrue(serial.closed)
        self.assertIn("motor 2", service.snapshot()["fault"])

    def test_preload_before_enable_then_watchdog_disables_both(self):
        serial = FakeUART()
        service = self.service(serial)
        service.start()
        self.arm(service)
        self.await_stop(service)
        commands = [d[2] for d in serial.writes]
        first_enable = commands.index(m.FUNCTION_ENABLE)
        self.assertEqual(commands[:first_enable].count(m.FUNCTION_MULTI_TURN_POSITION), 2)
        preloads = [d for d in serial.writes[:first_enable] if d[2] == m.FUNCTION_MULTI_TURN_POSITION]
        self.assertEqual([int.from_bytes(d[3:7], "big", signed=True) for d in preloads], [100, -200])
        self.assertEqual([(d[1], d[2]) for d in serial.writes[-2:]], [(1, 5), (2, 5)])
        self.assertIn("heartbeat", service.snapshot()["fault"])
        with self.assertRaises(RuntimeError):
            service.submit((0, 0))

    def test_partial_enable_failure_disables_each_axis(self):
        serial = FakeUART(fail_enable=2)
        service = self.service(serial)
        service.start()
        self.arm(service)
        self.await_stop(service)
        self.assertTrue(all(any(d[1] == mid and d[2] == 5 for d in serial.writes) for mid in (1, 2)))
        self.assertTrue(serial.closed)

    def test_feedback_loss_after_start_disables(self):
        serial = FakeUART()
        service = self.service(serial)
        service.start()
        deadline = time.monotonic() + 1
        while not service.snapshot()["ready"] and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(service.snapshot()["ready"])
        serial.missing = (1,)
        self.await_stop(service)
        self.assertIn("feedback", service.snapshot()["fault"])

    def test_simulation_never_opens_uart(self):
        service = m.MotorService(config(), lambda: self.fail("must not open serial"))
        service.start()
        time.sleep(.04)
        service.submit((30, 0))
        time.sleep(.06)
        state = service.snapshot()
        service.close()
        self.assertTrue(state["simulated"])
        self.assertTrue(0 < state["angles"][0] < 5)

    def test_reject_nan_and_overlimit_before_send(self):
        service = self.service(FakeUART())
        for target in ((0, 101), (float("nan"), 0)):
            with self.assertRaises(ValueError):
                service.submit(target)

    def test_stop_prevents_enable_but_allows_disable(self):
        serial = FakeUART()
        service = self.service(serial)
        service.serial = serial
        service.stop_event.set()
        with self.assertRaises(RuntimeError):
            service._write(m.enable(1))
        service._disable_all()
        self.assertEqual([(d[1], d[2]) for d in serial.writes], [(1, 5), (2, 5)])

    def test_protocol_resync_corrupt_fragmented_frames(self):
        fake = FakeUART()
        fake.write(m.request_feedback(1, 1))
        good = fake.read(100)
        corrupt = good[:-2] + b"\x00\x7b"
        parser = m.FeedbackParser()
        self.assertEqual(parser.feed(b"noise" + corrupt + good[:4]), [])
        self.assertEqual(parser.feed(good[4:]), [(1, 1, 100)])


class VisionAdapterTests(unittest.TestCase):
    def test_class_isolation_and_lost_tracks_not_current(self):
        obj = lambda cid: types.SimpleNamespace(class_id=cid, x=10, y=20, w=30, h=40, score=.9)
        model = types.SimpleNamespace(detect=lambda *a, **k: [obj(0), obj(2)], labels=["person", "bicycle", "car"])
        class FakeTracker:
            def __init__(self, *args):
                pass
            def update(self, objects):
                return [types.SimpleNamespace(id=0, lost=False, history=objects, score=.9),
                        types.SimpleNamespace(id=9, lost=True, history=objects, score=.9)] if objects else []
        api = types.SimpleNamespace(ByteTracker=FakeTracker, Object=lambda x,y,w,h,c,s: obj(c))
        maix = types.SimpleNamespace(nn=types.SimpleNamespace(YOLO11=lambda **kw: model), tracker=api)
        with patch.dict(sys.modules, {"maix": maix}):
            vision = m.NativeVision(config())
            result = vision.detect(types.SimpleNamespace(width=lambda: 640, height=lambda: 480))
        self.assertEqual([t["key"] for t in result], [(0, 0), (2, 0)])
        self.assertEqual(len(vision.trackers), 2)


class FixedReferenceTests(unittest.TestCase):
    def service(self, serial):
        c = config()
        c.update(motor_enabled=True, motor_calibrated=True, feedback_timeout=.01)
        service = m.MotorService(c, lambda: serial)
        service.serial = serial
        return service

    def test_power_cycle_preserves_physical_coordinates(self):
        for total in (125.0, 0.0):
            serial = FakeUART()
            serial.totals[2], serial.mechanical[2] = total, 25
            service = self.service(serial)
            self.assertEqual(service._initialize(), (0, 25))
            self.assertEqual(service.reference_zero[1], total-25)
            target_zero_raw = service.reference_zero[1]  # q=0 maps to T-25, not T.
            self.assertEqual(target_zero_raw, total-25)
            self.assertFalse(any(d[2] in (2, 6, 8, 9, 10) for d in serial.writes))

    def test_negative_mechanical_wrap_and_sign(self):
        serial = FakeUART()
        serial.totals[2], serial.mechanical[2] = 0, 350
        service = self.service(serial)
        service.c["motor_signs"] = (1, -1)
        self.assertEqual(service._initialize(), (0, 10))
        self.assertEqual(service.reference_zero[1], 10)

    def test_initial_physical_limit_refuses_enable(self):
        serial = FakeUART()
        serial.totals[2], serial.mechanical[2] = 0, 101
        service = self.service(serial)
        with self.assertRaisesRegex(RuntimeError, "outside calibrated limits"):
            service._initialize()
        self.assertFalse(any(d[2] == 6 for d in serial.writes))

    def test_motor_restart_invalidates_reference(self):
        serial = FakeUART()
        serial.totals[2], serial.mechanical[2] = 30, 25
        service = self.service(serial)
        service._initialize()
        serial.totals[2] = 0
        with self.assertRaisesRegex(RuntimeError, "reference mismatch"):
            service._measure()

    def test_connect_disabled_reads_feedback_until_explicit_arm(self):
        serial = FakeUART()
        service = self.service(serial)
        service.serial = None
        service.start()
        deadline = time.monotonic()+1
        while not service.snapshot()["ready"] and time.monotonic()<deadline:
            time.sleep(.005)
        self.assertTrue(service.snapshot()["ready"])
        self.assertFalse(service.snapshot()["armed"])
        count = len(serial.writes)
        time.sleep(.08)
        self.assertGreater(len(serial.writes), count)
        self.assertFalse(any(d[2] == 6 for d in serial.writes))
        service.close()
        self.assertIsNone(service.snapshot()["fault"])

    def test_calibration_requires_confirmation_before_uart(self):
        service = m.MotorService(config(), lambda: self.fail("must not open"))
        with self.assertRaises(ValueError):
            service.calibrate_mechanical_zero(2)

    def test_calibration_zero_readback_then_save_no_movement_or_enable(self):
        serial = FakeUART()
        serial.mechanical[2] = 25
        service = self.service(serial)
        service.calibrate_mechanical_zero(2, confirm=True)
        commands = [d[2] for d in serial.writes]
        self.assertIn(10, commands)
        self.assertIn(8, commands)
        self.assertLess(commands.index(10), commands.index(8))
        self.assertFalse(any(cmd in (2, 6, 9) for cmd in commands))
        self.assertTrue(serial.closed)

    def test_ignored_zero_never_saves(self):
        serial = FakeUART()
        serial.mechanical[2], serial.ignore_zero = 25, True
        service = self.service(serial)
        with self.assertRaisesRegex(RuntimeError, "readback failed"):
            service.calibrate_mechanical_zero(2, confirm=True)
        self.assertFalse(any(d[2] in (8, 6) for d in serial.writes))
        self.assertTrue(serial.closed)

    def test_calibration_missing_readback_never_saves(self):
        class DropReadback(FakeUART):
            def write(self, data):
                result = super().write(data)
                if data[2] == m.FUNCTION_MECHANICAL_ZERO:
                    self.missing = (2,)
                return result
        serial = DropReadback()
        service = self.service(serial)
        with self.assertRaisesRegex(RuntimeError, "No valid feedback"):
            service.calibrate_mechanical_zero(2, confirm=True)
        self.assertFalse(any(d[2] == 8 for d in serial.writes))
        self.assertTrue(serial.closed)

    def test_motion_during_reference_refuses_enable(self):
        class MovingUART(FakeUART):
            def write(self, data):
                if data[2] == 14 and data[3] == 1:
                    self.totals[data[1]] += 1
                return super().write(data)
        serial = MovingUART()
        service = self.service(serial)
        with self.assertRaisesRegex(RuntimeError, "moved while reading"):
            service._initialize()
        self.assertFalse(any(d[2] == 6 for d in serial.writes))


class MotionProtectionTests(unittest.TestCase):
    def test_stalled_position_latches_fault_after_grace(self):
        guard = m.MotionGuard(config())
        guard.update(0, (0, 0), (10, 0))
        guard.update(.5, (0, 0), (10, 0))
        with self.assertRaisesRegex(RuntimeError, "stalled"):
            guard.update(2.4, (0, 0), (10, 0))

    def test_wrong_direction_rejected(self):
        guard = m.MotionGuard(config())
        guard.update(0, (0, 0), (10, 0))
        with self.assertRaisesRegex(RuntimeError, "opposite"):
            guard.update(1, (-2, 0), (10, 0))

    def test_normal_progress_and_goal_reversal_reset_window(self):
        guard = m.MotionGuard(config())
        guard.update(0, (0, 0), (10, 0))
        guard.update(2.4, (2, 0), (10, 0))
        guard.update(2.5, (2, 0), (-10, 0))
        guard.update(4.9, (-2, 0), (-10, 0))

    def test_deadzone_not_classified_as_stall(self):
        guard = m.MotionGuard(config())
        guard.update(0, (0, 0), (1, 1))
        guard.update(20, (0, 0), (1, 1))

    def test_runtime_guard_fault_disables_and_keeps_feedback(self):
        serial = FakeUART()
        c = config()
        c.update(motor_enabled=True, motor_calibrated=True)
        service = m.MotorService(c, lambda: serial)
        service.guard.update = lambda *a: (_ for _ in ()).throw(RuntimeError("injected stall"))
        service.start()
        deadline = time.monotonic()+1
        while not service.snapshot()["ready"] and time.monotonic()<deadline:
            time.sleep(.005)
        service.request_arm(True)
        deadline = time.monotonic()+1
        while not service.snapshot()["fault"] and time.monotonic()<deadline:
            time.sleep(.005)
        self.assertIn("stall", service.snapshot()["fault"])
        self.assertFalse(service.snapshot()["armed"])
        count = len(serial.writes)
        time.sleep(.15)
        self.assertGreater(len(serial.writes), count)
        self.assertFalse(serial.closed)
        with self.assertRaises(RuntimeError):
            service.request_arm(True)
        service.close()

    def test_failed_disarm_latches_fault_and_attempts_other_axis(self):
        class FailingDisable(FakeUART):
            fail_disable = False
            def write(self, data):
                if self.fail_disable and data[1:3] == bytes((1, 5)):
                    self.writes.append(data)
                    raise OSError("injected disable failure")
                return super().write(data)
        serial = FailingDisable()
        c = config()
        c.update(motor_enabled=True, motor_calibrated=True)
        service = m.MotorService(c, lambda: serial)
        service.start()
        deadline = time.monotonic()+1
        while not service.snapshot()["ready"] and time.monotonic()<deadline:
            time.sleep(.005)
        service.request_arm(True)
        deadline = time.monotonic()+1
        while not service.snapshot()["armed"] and time.monotonic()<deadline:
            time.sleep(.005)
        self.assertTrue(service.snapshot()["armed"])
        serial.fail_disable = True
        start = len(serial.writes)
        service.request_arm(False)
        deadline = time.monotonic()+1
        while not service.snapshot()["fault"] and time.monotonic()<deadline:
            time.sleep(.005)
        self.assertIn("Disable request failed", service.snapshot()["fault"])
        self.assertTrue(any(d[1:3] == bytes((2, 5)) for d in serial.writes[start:]))
        service.close()


class ZeroLossProtectionTests(unittest.TestCase):
    def initialized(self):
        serial = FakeUART()
        c = config()
        c.update(motor_enabled=True, motor_calibrated=True, feedback_timeout=.01)
        service = m.MotorService(c, lambda: serial)
        service.serial = serial
        service.sent_target = service._initialize()
        return serial, service

    def assert_stopped(self, serial, service, start):
        state = service.snapshot()
        self.assertFalse(state["reference_valid"])
        self.assertFalse(state["armed"])
        self.assertIsNone(service.command_target)
        self.assertIsNone(service.sent_target)
        self.assertIn("ZERO_REFERENCE_LOST", state["reference_fault"])
        after = serial.writes[start:]
        self.assertTrue(all(any(d[1] == mid and d[2] == 5 for d in after) for mid in (1, 2)))
        self.assertFalse(any(d[2] in (2, 6) for d in after))
        with self.assertRaises(RuntimeError):
            service.request_arm(True)
        with self.assertRaises(RuntimeError):
            service.submit((0, 0))

    def test_missing_zero_state_blocks_enable_and_disables_both(self):
        serial, service = self.initialized()
        service.reference_valid = False
        start = len(serial.writes)
        with self.assertRaisesRegex(RuntimeError, "ZERO_REFERENCE_LOST"):
            service._write(m.enable(1))
        self.assert_stopped(serial, service, start)

    def test_nonfinite_zero_blocks_position_before_transmission(self):
        serial, service = self.initialized()
        service.reference_zero = (float("nan"), 0)
        start = len(serial.writes)
        with self.assertRaisesRegex(RuntimeError, "nonfinite"):
            service._write(m.set_multi_turn_position(1, 10))
        self.assert_stopped(serial, service, start)

    def test_finite_zero_corruption_blocks_position(self):
        serial, service = self.initialized()
        service.reference_zero = (0, 0)
        start = len(serial.writes)
        with self.assertRaisesRegex(RuntimeError, "changed unexpectedly"):
            service._write(m.set_multi_turn_position(1, 10))
        self.assert_stopped(serial, service, start)

    def test_expired_zero_blocks_position(self):
        serial, service = self.initialized()
        service.feedback_time -= 1
        start = len(serial.writes)
        with self.assertRaisesRegex(RuntimeError, "expired"):
            service._write(m.set_multi_turn_position(1, 10))
        self.assert_stopped(serial, service, start)

    def test_full_turn_jump_not_hidden_by_modulo(self):
        serial, service = self.initialized()
        serial.totals[1] += 360
        start = len(serial.writes)
        with self.assertRaisesRegex(RuntimeError, "discontinuity"):
            service._measure()
        self.assert_stopped(serial, service, start)

    def test_missing_mechanical_feedback_loses_zero(self):
        class MechanicalMissing(FakeUART):
            drop = False
            def write(self, data):
                if self.drop and data[2] == 14 and data[3] == 2:
                    self.writes.append(data)
                    return len(data)
                return super().write(data)
        serial = MechanicalMissing()
        c = config()
        c.update(motor_enabled=True, motor_calibrated=True, feedback_timeout=.01)
        service = m.MotorService(c, lambda: serial)
        service.serial = serial
        service._initialize()
        service.armed, serial.drop = True, True
        start = len(serial.writes)
        with self.assertRaisesRegex(RuntimeError, "ZERO_REFERENCE_LOST"):
            service._measure()
        self.assert_stopped(serial, service, start)

    def test_normal_mechanical_wrap_is_not_a_total_jump(self):
        serial, service = self.initialized()
        # Continuous -1 degree movement is valid despite M jumping from 0 to 359.
        serial.totals[1] -= 1
        serial.mechanical[1] = 359
        self.assertEqual(service._measure()[1], (-1, 0))
        self.assertTrue(service.snapshot()["reference_valid"])

    def test_feedback_recovery_cannot_restore_zero_or_motion(self):
        serial, service = self.initialized()
        serial.totals[1] += 360
        with self.assertRaises(RuntimeError):
            service._measure()
        serial.totals[1] -= 360
        start = len(serial.writes)
        service._read_diagnostics()
        self.assertEqual(service.snapshot()["raw_feedback"], (10, -20))
        self.assertFalse(service.snapshot()["reference_valid"])
        self.assertFalse(any(d[2] in (2, 6, 8, 9, 10) for d in serial.writes[start:]))
        with self.assertRaises(RuntimeError):
            service.request_arm(True)

    def test_thread_automatically_disables_on_zero_loss(self):
        serial = FakeUART()
        c = config()
        c.update(motor_enabled=True, motor_calibrated=True, feedback_timeout=.01)
        service = m.MotorService(c, lambda: serial)
        service.start()
        try:
            deadline = time.monotonic()+1
            while not service.snapshot()["ready"] and time.monotonic()<deadline:
                time.sleep(.005)
            service.request_arm(True)
            deadline = time.monotonic()+1
            while not service.snapshot()["armed"] and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertTrue(service.snapshot()["armed"])
            start = len(serial.writes)
            serial.totals[1] += 360
            deadline = time.monotonic()+1
            while not service.snapshot()["reference_fault"] and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertIn("ZERO_REFERENCE_LOST", service.snapshot()["reference_fault"])
            time.sleep(.03)  # Allow both disable frames to finish.
            disabled_at = next(i for i in range(start, len(serial.writes)) if serial.writes[i][2] == 5)
            self.assert_stopped(serial, service, disabled_at)
            time.sleep(.15)
            self.assertFalse(any(d[2] in (2, 6) for d in serial.writes[disabled_at:]))
        finally:
            service.close()


class LifecycleTests(unittest.TestCase):
    def test_camera_failure_closes_open_resources_without_starting_motor(self):
        closed = []
        cam = types.SimpleNamespace(read=lambda **kw: None, close=lambda: closed.append("camera"))
        disp = types.SimpleNamespace(close=lambda: closed.append("display"))
        touch = types.SimpleNamespace(close=lambda: closed.append("touch"))
        model = types.SimpleNamespace(input_width=lambda: 640, input_height=lambda: 480, input_format=lambda: 0)
        maix = types.SimpleNamespace(app=object(), camera=types.SimpleNamespace(Camera=lambda *a: cam),
            display=types.SimpleNamespace(Display=lambda: disp), touchscreen=types.SimpleNamespace(TouchScreen=lambda: touch),
            image=object(), sys=types.SimpleNamespace(device_name=lambda: "MaixCAM2"))
        with patch.dict(sys.modules, {"maix": maix}), patch.object(m, "NativeVision", return_value=types.SimpleNamespace(model=model)), \
             patch.object(m.SoundDirectionSensor, "open_uart"), patch.object(m.SoundDirectionSensor, "close", side_effect=lambda: closed.append("mic")), \
             patch.object(m.MotorService, "start") as start:
            with self.assertRaisesRegex(RuntimeError, "first camera frame"):
                m.run_board(config())
        start.assert_not_called()
        self.assertEqual(set(closed), {"camera", "display", "touch", "mic"})

    def test_calibration_tool_rejects_ambiguous_or_unstable_samples(self):
        spec = importlib.util.spec_from_file_location("calibration_table", ROOT / "calibration_table.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rows = [dict(vx=vx, vy=.2, yaw_deg=yaw, pitch_deg=0) for vx, yaw in ((-.7,-35),(0,0),(.7,35)) for _ in range(5)]
        self.assertEqual(len(module.make_table(rows)["sound_samples"]), 3)
        self.assertFalse(module.make_table(rows)["sound_calibrated"])
        rows[0]["vx"] = .9
        with self.assertRaises(ValueError):
            module.make_table(rows)


if __name__ == "__main__":
    unittest.main()
