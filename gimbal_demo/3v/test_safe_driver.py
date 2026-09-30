import unittest

from demo import DryRunTransport
from safe_f32c_driver import F32CLimitError, SafeF32CGimbal


class RecordingTransport(DryRunTransport):
    def __init__(self):
        super().__init__()
        self.frames = []

    def write(self, data):
        self.frames.append(bytes(data))
        # Avoid DryRunTransport's diagnostic print while keeping its simulator.
        if len(data) == 9 and data[2] == 0x02:
            self.position[data[1]] = int.from_bytes(data[3:7], "big", signed=True)
        if len(data) == 6 and data[2] == 0x0E:
            motor_id, parameter_address = data[1], data[3]
            if parameter_address == 0x01:
                value = self.position.get(motor_id, 0)
            elif parameter_address == 0x04:
                value = 1200
            else:
                value = 0
            from f32c_protocol import bcc

            body = bytes((0x7A, motor_id, parameter_address)) + value.to_bytes(
                4, "big", signed=True
            )
            self.rx.extend(body + bytes((bcc(body), 0x7B)))
        return len(data)


class DriverHardLimitTests(unittest.TestCase):
    def setUp(self):
        self.transport = RecordingTransport()
        self.gimbal = SafeF32CGimbal(self.transport)
        self.gimbal.start(power_on_delay_s=0.0)

    def tearDown(self):
        self.gimbal.stop()

    def test_boundaries_are_allowed(self):
        self.gimbal.set_relative_angles(0.0, -100.0)
        self.gimbal.set_relative_angles(0.0, 100.0)

    def test_positive_overtravel_is_rejected_before_write(self):
        before = len(self.transport.frames)
        with self.assertRaises(F32CLimitError):
            self.gimbal.set_relative_angles(0.0, 100.1)
        self.assertEqual(len(self.transport.frames), before)

    def test_negative_overtravel_is_rejected_before_write(self):
        before = len(self.transport.frames)
        with self.assertRaises(F32CLimitError):
            self.gimbal.set_relative_angles(0.0, -100.1)
        self.assertEqual(len(self.transport.frames), before)

    def test_non_finite_target_is_rejected(self):
        before = len(self.transport.frames)
        with self.assertRaises(F32CLimitError):
            self.gimbal.set_relative_angles(float("nan"), 0.0)
        self.assertEqual(len(self.transport.frames), before)


if __name__ == "__main__":
    unittest.main()
