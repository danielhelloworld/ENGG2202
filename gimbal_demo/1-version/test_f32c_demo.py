import unittest

from f32c_protocol import (
    FEEDBACK_TOTAL_ANGLE,
    bcc,
    enable_frame,
    feedback_request_frame,
    mode_frame,
    multi_turn_position_frame,
    parse_feedback_frame,
    speed_frame,
)
from tracking_controller import ImageTrackingController, TrackingConfig, YAxisDeadZone


class ProtocolTests(unittest.TestCase):
    def test_manual_examples(self):
        self.assertEqual(enable_frame(1), bytes.fromhex("7A 01 06 7D 7B"))
        self.assertEqual(enable_frame(2), bytes.fromhex("7A 02 06 7E 7B"))
        self.assertEqual(mode_frame(2, 1), bytes.fromhex("7A 02 00 00 01 79 7B"))
        self.assertEqual(speed_frame(2, 100), bytes.fromhex("7A 02 01 00 64 1D 7B"))
        self.assertEqual(
            multi_turn_position_frame(2, 360.0),
            bytes.fromhex("7A 02 02 00 00 0E 10 64 7B"),
        )
        self.assertEqual(
            multi_turn_position_frame(2, -360.0),
            bytes.fromhex("7A 02 02 FF FF F1 F0 7B 7B"),
        )
        self.assertEqual(
            feedback_request_frame(2, FEEDBACK_TOTAL_ANGLE),
            bytes.fromhex("7A 02 0E 01 77 7B"),
        )

    def test_feedback_parse(self):
        body = bytes.fromhex("7A 02 01 FF FF F1 F0")
        frame = body + bytes((bcc(body), 0x7B))
        self.assertEqual(parse_feedback_frame(frame), (2, 1, -3600))


class TrackingTests(unittest.TestCase):
    def test_default_y_dead_zone(self):
        zone = YAxisDeadZone(120.0, 180.0, 2.0)
        self.assertEqual((zone.low, zone.high), (-118.0, 118.0))
        self.assertEqual(zone.clamp(200.0), 118.0)
        self.assertEqual(zone.clamp(-200.0), -118.0)

    def test_right_target_moves_x_positive(self):
        controller = ImageTrackingController(
            TrackingConfig(max_rate_deg_s=1000.0, pixel_deadband=0.0)
        )
        x, y, active = controller.update(
            target_cx=500,
            target_cy=240,
            frame_width=640,
            frame_height=480,
            now_s=1.0,
        )
        self.assertTrue(active)
        self.assertGreater(x, 0.0)
        self.assertAlmostEqual(y, 0.0)


if __name__ == "__main__":
    unittest.main()

