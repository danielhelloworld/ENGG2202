"""Compatibility scheduler tests with an explicit fake model, not a board test."""
from pathlib import Path
import runpy
import types
import unittest
from unittest.mock import patch

import numpy as np

APP = Path(__file__).resolve().parents[1]
m = types.SimpleNamespace(**runpy.run_path(str(APP / "main.py")))


class FakeModel:
    labels = ["target"]

    def input_format(self):
        return "RGB"

    def detect(self, img, **kwargs):
        return [types.SimpleNamespace(x=1, y=2, w=3, h=4, score=.9, class_id=0)]


class InlineTests(unittest.TestCase):
    def setUp(self):
        c = dict(m.CONFIG, inference_fps=10)
        image = types.SimpleNamespace(cv2image=lambda *a, **kw: types.SimpleNamespace(format=lambda: "RGB"))
        self.detector = m.InlineDetector(c, FakeModel(), image)
        self.rgb = np.zeros((8, 8, 3), np.uint8)

    def submit_at(self, now):
        with patch.object(m.time, "monotonic", return_value=now):
            self.detector.submit(self.rgb, now)

    def test_intervening_frames_skip_inference_without_refreshing_result_timestamp(self):
        self.submit_at(10.)
        self.submit_at(10.04)
        boxes, stats = self.detector.snapshot(10.04)
        self.assertEqual(stats["completed"], 1)
        self.assertEqual(boxes[0]["captured_at"], 10.)
        self.submit_at(10.11)
        boxes, stats = self.detector.snapshot(10.11)
        self.assertEqual(stats["completed"], 2)
        self.assertEqual(boxes[0]["captured_at"], 10.11)

    def test_stale_or_future_result_hidden(self):
        self.submit_at(10.)
        self.assertTrue(self.detector.snapshot(10.1)[0])
        self.assertEqual(self.detector.snapshot(11.)[0], [])
        self.assertEqual(self.detector.snapshot(9.)[0], [])

    def test_calibration_clears_boxes_and_resume_infers_immediately(self):
        self.submit_at(10.)
        self.detector.set_enabled(False)
        self.submit_at(10.01)
        self.assertEqual(self.detector.completed, 1)
        self.assertEqual(self.detector.snapshot(10.01)[0], [])
        self.detector.set_enabled(True)
        self.assertEqual(self.detector.snapshot(10.02)[0], [])
        self.submit_at(10.02)
        self.assertEqual(self.detector.completed, 2)

    def test_failure_clears_old_boxes_and_does_not_retry_each_video_frame(self):
        self.submit_at(10.)
        with patch.object(self.detector.model, "detect", side_effect=RuntimeError("fake NPU failure")) as detect:
            with self.assertLogs("thermal_yolo", level="ERROR"):
                self.submit_at(11.)
            self.submit_at(12.)
            self.assertEqual(detect.call_count, 1)
        boxes, stats = self.detector.snapshot(12.)
        self.assertEqual(boxes, [])
        self.assertIn("fake NPU failure", stats["error"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
