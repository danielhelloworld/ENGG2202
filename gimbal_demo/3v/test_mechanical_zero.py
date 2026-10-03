"""Independent motor model: physical position, power-session total, flash zero."""

import queue
import time
import unittest

from f32c_protocol import (
    F32CGimbal, F32CError, F32CLimitError, bcc, mechanical_zero_frame,
)
from gimbal_gui import GimbalWorker


class MotorModel:
    def __init__(self):
        self.physical = {1: 700, 2: 1300}
        self.power_origin = dict(self.physical)
        self.zero = {1: 700, 2: 1300}
        self.flash = dict(self.zero)
        self.speed = {1: 0, 2: 0}
        self.frames = []
        self.rx = bytearray()
        self.ignore_zero = False
        self.fail_after_zero = False
        self.zero_sent = False

    def power_cycle(self):
        self.power_origin = dict(self.physical)
        self.zero = dict(self.flash)

    def write(self, frame):
        self.frames.append(frame)
        if len(frame) < 5:
            return len(frame)
        motor, command = frame[1:3]
        if command == 0x02:
            target = int.from_bytes(frame[3:7], "big", signed=True)
            self.physical[motor] = self.power_origin[motor] + target
        elif command == 0x0A:
            self.zero_sent = True
            if not self.ignore_zero:
                self.zero[motor] = self.physical[motor]
        elif command == 0x08:
            self.flash[motor] = self.zero[motor]
        elif command == 0x0E:
            if self.fail_after_zero and self.zero_sent:
                return len(frame)
            value = {
                0: self.speed[motor],
                1: self.physical[motor] - self.power_origin[motor],
                2: (self.physical[motor] - self.zero[motor]) % 3600,
            }.get(frame[3], 0)
            body = bytes((0x7A, motor, frame[3])) + value.to_bytes(4, "big", signed=True)
            self.rx.extend(body + bytes((bcc(body), 0x7B)))
        return len(frame)

    def read(self, size=1):
        result = bytes(self.rx[:size])
        del self.rx[:size]
        return result

    def close(self):
        pass


class MechanicalZeroTests(unittest.TestCase):
    def test_vendor_zero_frame_and_custom_axis_id(self):
        self.assertEqual(mechanical_zero_frame(2).hex(), "7a020a727b")
        self.assertEqual(mechanical_zero_frame(9).hex(), "7a090a797b")

    def test_calibration_targets_one_axis_and_does_not_move_or_enable(self):
        model = MotorModel()
        model.physical[2] += 420
        bus = F32CGimbal(model)
        bus.set_mechanical_zero(2)
        self.assertEqual(model.flash[2], 1720)
        self.assertEqual(model.flash[1], 700)
        writes = [(f[1], f[2]) for f in model.frames if len(f) >= 5]
        self.assertIn((2, 0x0A), writes)
        self.assertIn((2, 0x08), writes)
        self.assertNotIn((1, 0x0A), writes)
        self.assertFalse(any(cmd in (0x02, 0x03, 0x06, 0x09) for _, cmd in writes))
        self.assertFalse(bus.enabled)
        self.assertEqual(bus.reference_mode, "mechanical")
        self.assertEqual(bus.read_relative_angles(), (0.0, 0.0))

        model.physical[2] -= 250
        model.power_cycle()
        reboot = F32CGimbal(model, reference_mode="mechanical")
        reboot.start(power_on_delay_s=0, enable_motors=False)
        self.assertEqual(reboot.read_relative_angles(), (0.0, -25.0))
        self.assertEqual(reboot.get_software_zero(), (0.0, 25.0))
        reboot.enable_at_current_position()
        reboot.set_relative_angles(0, 30)
        self.assertEqual(model.physical[2], 2020)
        reboot.stop()

    def test_enabled_or_moving_axis_cannot_be_calibrated(self):
        model = MotorModel()
        bus = F32CGimbal(model)
        bus.enabled = True
        with self.assertRaises(F32CError):
            bus.set_mechanical_zero(2)
        self.assertEqual(model.frames, [])
        bus.enabled = False
        model.speed[2] = 1
        with self.assertRaises(F32CError):
            bus.set_mechanical_zero(2)
        self.assertFalse(any(f[2] in (0x0A, 0x08) for f in model.frames))

    def test_ignored_zero_or_lost_feedback_blocks_save_and_enable(self):
        for lose_feedback in (False, True):
            with self.subTest(lose_feedback=lose_feedback):
                model = MotorModel()
                model.physical[2] += 300
                model.ignore_zero = not lose_feedback
                model.fail_after_zero = lose_feedback
                bus = F32CGimbal(model, feedback_timeout_s=.01, feedback_retries=1)
                with self.assertRaises(F32CError):
                    bus.set_mechanical_zero(2)
                self.assertFalse(bus.referenced)
                self.assertTrue(bus.calibration_fault)
                self.assertFalse(any(f[2] == 0x08 for f in model.frames))
                with self.assertRaises(F32CError):
                    bus.enable_at_current_position()
                with self.assertRaises(F32CError):
                    bus.capture_reference()

    def test_fixed_reference_refuses_outside_y_without_motion(self):
        model = MotorModel()
        model.physical[2] += 1100
        bus = F32CGimbal(model, reference_mode="mechanical")
        bus.start(power_on_delay_s=0, enable_motors=False)
        self.assertEqual(bus.read_relative_angles()[1], 110)
        with self.assertRaises(F32CLimitError):
            bus.enable_at_current_position()
        self.assertFalse(any(f[2] in (0x02, 0x06) for f in model.frames if len(f) >= 5))

    def test_negative_wrap_and_session_reset_are_not_silently_new_zero(self):
        model = MotorModel()
        model.physical[2] -= 990
        model.power_cycle()
        bus = F32CGimbal(model, reference_mode="mechanical")
        bus.start(power_on_delay_s=0, enable_motors=False)
        self.assertEqual(bus.read_relative_angles()[1], -99)
        model.physical[2] += 500
        self.assertEqual(bus.read_relative_angles()[1], -49)
        model.power_cycle()
        with self.assertRaises(F32CError):
            bus.read_relative_angles()

    def test_movement_without_reported_speed_blocks_zero_write(self):
        class MovingModel(MotorModel):
            def write(self, frame):
                if len(frame) == 6 and frame[1:4] == bytes((2, 0x0E, 1)):
                    self.physical[2] += 2
                return super().write(frame)

        model = MovingModel()
        bus = F32CGimbal(model)
        with self.assertRaises(F32CError):
            bus.set_mechanical_zero(2)
        self.assertFalse(any(f[2] in (0x0A, 0x08) for f in model.frames))


class MechanicalWorkerTests(unittest.TestCase):
    def wait_for(self, events, kind):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                event, value = events.get(timeout=.1)
            except queue.Empty:
                continue
            if event == "error":
                self.fail(value)
            if event == kind:
                return value
        self.fail("missing event " + kind)

    def test_disabled_connection_calibration_and_feedback(self):
        model = MotorModel()
        model.physical[2] += 250
        events = queue.Queue()
        worker = GimbalWorker(None, 20, 100, events, demo_mode=True,
                              reference_mode="mechanical", start_enabled=False)
        worker._open_transport = lambda: model
        worker.start()
        try:
            self.assertFalse(self.wait_for(events, "connected")[2])
            self.assertEqual(self.wait_for(events, "position"), (0, 25))
            worker.calibrate_zero("Y")
            self.assertEqual(self.wait_for(events, "zero_saved"), "Y")
            self.assertEqual(self.wait_for(events, "position"), (0, 0))
            self.assertFalse(worker.motor_enabled)
            self.assertTrue(worker.is_alive())
            self.assertFalse(any(f[2] in (0x02, 0x06) for f in model.frames if len(f) >= 5))
        finally:
            worker.request_stop()
            worker.join(3)

    def test_runtime_overtravel_stops_and_disabled_telemetry_continues(self):
        model = MotorModel()
        events = queue.Queue()
        worker = GimbalWorker(None, 20, 100, events, demo_mode=True,
                              reference_mode="mechanical", start_enabled=False)
        worker._open_transport = lambda: model
        worker.start()
        try:
            self.wait_for(events, "position")
            worker.set_enabled(True)
            self.assertTrue(self.wait_for(events, "motor_enabled"))
            model.physical[2] += 1010
            self.assertFalse(self.wait_for(events, "motor_enabled"))
            self.assertTrue(self.wait_for(events, "feedback_issue")[1])
            self.assertEqual(self.wait_for(events, "position"), (0, 101))
            self.assertFalse(worker.motor_enabled)
            self.assertTrue(worker.is_alive())
        finally:
            worker.request_stop()
            worker.join(3)


if __name__ == "__main__":
    unittest.main()
