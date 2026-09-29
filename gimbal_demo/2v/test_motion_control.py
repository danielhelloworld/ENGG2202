import unittest

from f32c_protocol import (
    F32CGimbal,
    MODE_MULTI_T,
    PID_SPEED_KI,
    PID_SPEED_KP,
    acceleration_frame,
    pid_frame,
    save_parameters_frame,
)
from test_safe_driver import RecordingTransport


class MotionProtocolTests(unittest.TestCase):
    def test_manual_acceleration_and_pid_examples(self):
        self.assertEqual(
            acceleration_frame(2, 100), bytes.fromhex("7A 02 07 00 64 1B 7B")
        )
        self.assertEqual(
            pid_frame(2, PID_SPEED_KP, 10),
            bytes.fromhex("7A 02 0F 00 0A 7D 7B"),
        )
        self.assertEqual(
            pid_frame(2, PID_SPEED_KI, 10),
            bytes.fromhex("7A 02 10 00 0A 62 7B"),
        )
        self.assertEqual(
            save_parameters_frame(2), bytes.fromhex("7A 02 08 70 7B")
        )

    def test_pid_range_is_checked(self):
        with self.assertRaises(ValueError):
            pid_frame(1, PID_SPEED_KP, 257)


class DisabledFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.transport = RecordingTransport()
        self.gimbal = F32CGimbal(self.transport)
        self.gimbal.start(
            mode=MODE_MULTI_T,
            acceleration=100,
            power_on_delay_s=0.0,
        )

    def tearDown(self):
        self.gimbal.stop()

    def test_feedback_remains_available_while_disabled(self):
        self.gimbal.disable()
        self.assertFalse(self.gimbal.enabled)

        # Simulate manually rotating the powerless motors after disabling torque.
        self.transport.position[1] = 123
        self.transport.position[2] = -456
        self.assertEqual(self.gimbal.read_relative_angles(), (12.3, -45.6))

    def test_reenable_keeps_the_original_software_zero(self):
        zero_before = self.gimbal.get_software_zero()
        self.gimbal.disable()
        self.gimbal.enable()
        self.assertTrue(self.gimbal.enabled)
        self.assertEqual(self.gimbal.get_software_zero(), zero_before)


if __name__ == "__main__":
    unittest.main()
