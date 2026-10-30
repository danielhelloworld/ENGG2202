"""NPU pipeline and camera API models; these tests do not run a real NPU."""
import contextlib
import io
import types
import unittest
from unittest.mock import patch
import test_tracker as app

m = app.m


class DelayedDetector:
    def __init__(self, now, dual):
        self.previous = None
        self.now, self.dual = now, dual

    def detect(self, img):
        self.now[0] += .02
        result = self.previous if self.dual else img
        self.previous = img
        return [] if result is None else [app.track(box=(result*10, 120, 80, 80))]


class PipelineTests(unittest.TestCase):
    def make(self, dual=True):
        now = [1.]
        detector = DelayedDetector(now, dual)
        return m.DetectionPipeline(detector, dict(m.CONFIG, dual_buff=dual), lambda: now[0]), now

    def test_double_buffer_warmup_does_not_offer_a_detection(self):
        pipeline, now = self.make()
        self.assertIsNone(pipeline.push(1, 1., (10., -5.)))
        self.assertEqual(len(pipeline.completed), 0)

    def test_previous_image_pose_and_timestamp_match_the_delayed_boxes(self):
        pipeline, now = self.make()
        pipeline.push(1, 1., (10., -5.))
        now[0] = 1.04
        img, tracks, stamp, pose = pipeline.push(2, 1.04, (12., -3.))
        self.assertEqual((img, stamp, pose), (1, 1., (10., -5.)))
        self.assertEqual(tracks[0]["box"][0], 10)
        self.assertAlmostEqual(pipeline.stats()["capture_to_result_ms"], 60.)
        now[0] = 1.08
        result = pipeline.push(3, 1.08, (14., -1.))
        self.assertEqual((result[0], result[2], result[3]), (2, 1.04, (12., -3.)))
        self.assertAlmostEqual(pipeline.stats()["inference_fps_actual"], 25.)
        self.assertAlmostEqual(pipeline.stats()["detect_api_ms"], 20.)

    def test_single_buffer_uses_current_image_without_one_frame_delay(self):
        pipeline, now = self.make(False)
        result = pipeline.push(2, 1., (12., -3.))
        self.assertEqual((result[0], result[2], result[3]), (2, 1., (12., -3.)))
        self.assertEqual(result[1][0]["box"][0], 20)
        self.assertIsNone(pipeline.pending)

    def test_pipeline_close_releases_pending_camera_image(self):
        pipeline, now = self.make()
        pipeline.push(1, 1., None)
        pipeline.close()
        self.assertIsNone(pipeline.pending)


class FakeCamera:
    def __init__(self, flips=(0, 0), fail_vertical=False, fail_restore=False):
        self.h, self.v = flips
        self.fail_vertical, self.fail_restore = fail_vertical, fail_restore

    def hmirror(self, value=-1):
        if value != -1:
            if self.fail_restore and value == 0 and self.h == 1:
                raise RuntimeError("restore failed")
            self.h = value
        return self.h

    def vflip(self, value=-1):
        if value == 1 and self.fail_vertical:
            raise RuntimeError("not supported")
        if value != -1:
            self.v = value
        return self.v


class CameraRotationTests(unittest.TestCase):
    def setUp(self):
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def test_camera_flip_is_relative_to_its_existing_orientation(self):
        for flips in ((0, 0), (1, 0), (0, 1), (1, 1)):
            cam = FakeCamera(flips)
            self.assertTrue(m.configure_camera_rotation(cam, dict(m.CONFIG)))
            self.assertEqual((cam.h, cam.v), tuple(1-v for v in flips))

    def test_disabled_camera_rotation_keeps_software_fallback(self):
        cam = FakeCamera()
        self.assertFalse(m.configure_camera_rotation(cam, dict(m.CONFIG, camera_rotate=False)))
        self.assertEqual((cam.h, cam.v), (0, 0))
        self.assertFalse(m.configure_camera_rotation(cam, dict(m.CONFIG, rotate_180=False)))

    def test_missing_api_falls_back_without_partial_camera_changes(self):
        self.assertFalse(m.configure_camera_rotation(object(), dict(m.CONFIG)))

    def test_partial_setting_is_restored_before_software_fallback(self):
        cam = FakeCamera(fail_vertical=True)
        self.assertFalse(m.configure_camera_rotation(cam, dict(m.CONFIG)))
        self.assertEqual((cam.h, cam.v), (0, 0))

    def test_uncertain_orientation_blocks_motor_startup(self):
        cam = FakeCamera(fail_vertical=True, fail_restore=True)
        with self.assertRaisesRegex(RuntimeError, "orientation is uncertain"):
            m.configure_camera_rotation(cam, dict(m.CONFIG))

    def test_camera_rotated_frame_skips_opencv_rotation_and_preserves_pixels(self):
        # No cv2 binding is needed on this path.
        maix = types.SimpleNamespace(image=object())
        img = types.SimpleNamespace(format=lambda: "rgb")
        model = types.SimpleNamespace(input_format=lambda: "rgb")
        with patch.dict("sys.modules", {"maix": maix, "cv2": None}):
            self.assertIs(m.prepare_image(img, dict(m.CONFIG), model, True), img)


if __name__ == "__main__":
    unittest.main()
