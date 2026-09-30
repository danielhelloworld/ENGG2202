import unittest

from f32c_protocol import FEEDBACK_TOTAL_ANGLE, F32CGimbal, F32CTimeout
from test_safe_driver import RecordingTransport


class SilentTransport:
    def __init__(self):
        self.frames = []

    def write(self, data):
        self.frames.append(bytes(data))
        return len(data)

    def read(self, _size=1):
        return b""


class SecondAttemptTransport(RecordingTransport):
    def __init__(self):
        super().__init__()
        self.feedback_requests = 0

    def write(self, data):
        if len(data) == 6 and data[2] == 0x0E:
            self.feedback_requests += 1
            if self.feedback_requests == 1:
                self.frames.append(bytes(data))
                return len(data)
        return super().write(data)


class FeedbackDiagnosticTests(unittest.TestCase):
    def test_timeout_reports_motor_type_attempts_and_empty_rx(self):
        transport = SilentTransport()
        gimbal = F32CGimbal(
            transport, feedback_timeout_s=0.005, feedback_retries=2
        )
        with self.assertRaises(F32CTimeout) as caught:
            gimbal.request_feedback(1, FEEDBACK_TOTAL_ANGLE)
        message = str(caught.exception)
        self.assertIn("motor ID 1", message)
        self.assertIn("type 0x01", message)
        self.assertIn("after 2 attempts", message)
        self.assertIn("<no bytes received>", message)
        self.assertEqual(len(transport.frames), 2)

    def test_second_attempt_can_recover_a_slow_startup_reply(self):
        transport = SecondAttemptTransport()
        transport.position[1] = 321
        gimbal = F32CGimbal(
            transport, feedback_timeout_s=0.005, feedback_retries=2
        )
        self.assertEqual(gimbal.request_feedback(1, FEEDBACK_TOTAL_ANGLE), 321)
        self.assertEqual(transport.feedback_requests, 2)


if __name__ == "__main__":
    unittest.main()
