import unittest

from f32c_protocol import F32CGimbal, F32CLimitError
from test_safe_driver import RecordingTransport


class ZeroReferenceTests(unittest.TestCase):
    def setUp(self):
        self.transport = RecordingTransport()
        # Simulate non-zero total-angle feedback at connection time.
        self.transport.position = {1: 1234, 2: -567}
        self.gimbal = F32CGimbal(self.transport)
        self.gimbal.start(power_on_delay_s=0.0)

    def tearDown(self):
        self.gimbal.stop()

    def test_start_automatically_reads_zero_reference(self):
        self.assertEqual(self.gimbal.get_software_zero(), (123.4, -56.7))
        self.assertEqual(self.gimbal.read_relative_angles(), (0.0, 0.0))

    def test_return_to_zero_uses_captured_total_angles(self):
        self.gimbal.set_relative_angles(25.0, 50.0)
        self.assertEqual(self.transport.position, {1: 1484, 2: -67})

        self.gimbal.return_to_zero()

        self.assertEqual(self.transport.position, {1: 1234, 2: -567})
        self.assertEqual(self.gimbal.last_relative_deg, {1: 0.0, 2: 0.0})

    def test_base_driver_rejects_y_overtravel_before_serial_write(self):
        before = len(self.transport.frames)
        with self.assertRaises(F32CLimitError):
            self.gimbal.set_relative_angles(0.0, 100.1)
        self.assertEqual(len(self.transport.frames), before)


if __name__ == "__main__":
    unittest.main()
