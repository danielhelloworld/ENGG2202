"""Known-transform geometry, held-out validation, persistence and touch workflows."""
import json
import contextlib
import io
from pathlib import Path
import runpy
import tempfile
import time
import types
import unittest

import cv2
import numpy as np

APP = Path(__file__).resolve().parents[1]
m = types.SimpleNamespace(**runpy.run_path(str(APP / "main.py")))


class RegistrationTests(unittest.TestCase):
    def setUp(self):
        self.c = dict(m.CONFIG)
        self.size = (640, 480)
        self.h = np.array([[2.4, .18, 85], [-.12, 2.5, 65], [.0004, -.0003, 1.]])
        self.src = np.array([[12, 24], [65, 14], [139, 20], [145, 69],
                             [130, 106], [78, 104], [18, 101], [15, 60]], np.float64)
        self.check = np.array([[48, 46], [104, 81], [85, 60]], np.float64)
        self.dst = self.project(self.src)

    def project(self, points, matrix=None):
        return cv2.perspectiveTransform(np.asarray(points, np.float64).reshape(-1, 1, 2),
                                        self.h if matrix is None else matrix).reshape(-1, 2)

    def solved(self):
        h, fit = m.fit_registration(self.src, self.dst, self.size)
        checks = m.check_registration(h, self.check, self.project(self.check), self.size, fit)
        return h, dict(fit, **checks)

    def test_recovers_projective_transform_on_unseen_points(self):
        h, stats = self.solved()
        np.testing.assert_allclose(self.project(self.check, h), self.project(self.check), atol=1e-3)
        self.assertLess(stats["check_max_px"], .001)

    def test_ransac_rejects_mismatched_marker(self):
        dst = self.dst.copy()
        dst[2] += [50, -30]
        h, fit = m.fit_registration(self.src, dst, self.size)
        self.assertEqual(fit["inliers"], 7)
        self.assertEqual(fit["rejected_indices"], [2])
        np.testing.assert_allclose(self.project(self.check, h), self.project(self.check), atol=.001)

    def test_degenerate_and_insufficient_point_sets_rejected(self):
        line = np.array([[i * 15, 40] for i in range(8)])
        for src, dst in ((self.src[:4], self.dst[:4]), (line, self.project(line)),
                         (self.src * .05 + 50, self.project(self.src * .05 + 50))):
            with self.assertRaises(ValueError):
                m.fit_registration(src, dst, self.size)

    def test_independent_checks_cannot_reuse_training_points_or_large_error(self):
        h, fit = m.fit_registration(self.src, self.dst, self.size)
        with self.assertRaisesRegex(ValueError, "NEW markers"):
            m.check_registration(h, self.src[:2], self.dst[:2], self.size, fit)
        with self.assertRaisesRegex(ValueError, "Check error"):
            m.check_registration(h, self.check, self.project(self.check) + 20, self.size, fit)

    def test_singular_nonfinite_and_horizon_matrices_rejected(self):
        for h in (np.zeros((3, 3)), np.full((3, 3), np.nan),
                  [[1, 0, 0], [0, 1, 0], [-.01, 0, 1]]):
            with self.assertRaises(ValueError):
                m.validate_homography(h, self.size)

    def test_flips_not_applied_twice_and_roi_uses_projected_raw_pixel(self):
        a = m.Alignment(self.c, self.size)
        a.set_registration(*self.solved())
        expected = self.project([[48, 46]])[0]
        np.testing.assert_allclose([a.mx[46, 48], a.my[46, 48]], expected, atol=.001)
        frame = {"pixels": np.zeros((120, 160), np.uint8), "lo": 200, "hi": 400, "valid_temperature": True}
        frame["pixels"][46, 48] = 254
        result = m.roi_temperature(frame, (expected[0] - .1, expected[1] - .1, .2, .2), a)
        self.assertEqual(result, {"max": 40., "mean": 40., "samples": 1})
        self.assertFalse(a.mask[0, 0])

    def test_letterbox_keeps_homography_in_camera_coordinates(self):
        c = dict(self.c, width=800, height=800)
        a = m.Alignment(c, self.size)
        a.set_registration(*self.solved())
        x, y, w, h, _, _ = a.viewport
        expected = (self.project([[48, 46]])[0] + .5) * [w / 640, h / 480] - .5 + [x, y]
        np.testing.assert_allclose([a.mx[46, 48], a.my[46, 48]], expected, atol=.001)
        self.assertFalse(a.mask[:y].any())

    def test_distance_switch_and_save_reload_preserve_both_profiles(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "new-directory" / "alignment.json")
            a = m.Alignment(self.c, self.size, path)
            a.set_registration(*self.solved())
            first = a.matrix.copy()
            a.select_distance(2.)
            self.assertIsNone(a.homography)
            self.h[0, 2] += 17
            self.dst = self.project(self.src)
            a.set_registration(*self.solved())
            second = a.matrix.copy()
            with contextlib.redirect_stdout(io.StringIO()):
                a.save()
            b = m.Alignment(self.c, self.size, path)
            self.assertEqual(b.distance, 2.)
            np.testing.assert_allclose(b.matrix, second)
            b.select_distance(1.)
            np.testing.assert_allclose(b.matrix, first)
            b.select_distance(3.)
            self.assertIsNone(b.homography)
            self.assertEqual(set(b.profiles), {"1m", "2m"})

    def test_profiles_rejected_after_camera_geometry_or_orientation_change(self):
        a = m.Alignment(self.c, self.size)
        a.set_registration(*self.solved())
        c = dict(self.c, calibration_profiles=a.profiles)
        self.assertIsNone(m.Alignment(c, (320, 240)).homography)
        self.assertIsNone(m.Alignment(dict(c, thermal_flip_x=False), self.size).homography)
        self.assertIsNone(m.Alignment(dict(c, camera_fps=30), self.size).homography)
        copied = json.loads(json.dumps(a.profiles))
        copied["1m"]["matrix"][0][2] += 100
        self.assertIsNone(m.Alignment(dict(c, calibration_profiles=copied), self.size).homography)

    def test_touch_full_frame_workflow_and_cancellation_preserves_profile(self):
        a = m.Alignment(self.c, self.size)
        session = m.CalibrationSession(a)
        now = time.monotonic()
        frame = {"pixels": np.zeros((120, 160), np.uint8), "lo": 200, "hi": 400,
                 "received_at": now, "valid_temperature": True}
        session.observe(np.zeros((480, 640, 3), np.uint8), frame, now, now)
        session.begin()
        session.action("FREEZE")
        def add_pair(src, dst):
            # Fractional coordinates exercise exact panel mapping rather than rounded display marks.
            def screen(point, thermal):
                bx, by, w, h, sw, sh = session.panel(thermal)
                px, py = point
                if thermal:
                    px, py = sw - 1 - px, sh - 1 - py
                return bx + (px + .5) * w / sw - .5, by + (py + .5) * h / sh - .5
            session.click(*screen(dst, False))
            session.click(*screen(src, True))
        for src, dst in zip(self.src, self.dst):
            add_pair(src, dst)
        self.assertEqual(len(session.thermal_points), 8)
        session.action("FIT")
        self.assertEqual(session.phase, "CHECK")
        for src, dst in zip(self.check, self.project(self.check)):
            add_pair(src, dst)
        session.action("APPLY")
        self.assertFalse(session.active)
        self.assertIsNotNone(a.homography)
        matrix = a.matrix.copy()
        session.begin()
        session.action("CANCEL")
        np.testing.assert_array_equal(a.matrix, matrix)

    def test_freeze_rejects_stale_or_unpaired_frames(self):
        s = m.CalibrationSession(m.Alignment(self.c, self.size))
        now = time.monotonic()
        rgb = np.zeros((480, 640, 3), np.uint8)
        frame = {"pixels": np.zeros((120, 160), np.uint8), "received_at": now - 3,
                 "valid_temperature": True}
        s.observe(rgb, frame, now, now)
        with self.assertRaisesRegex(ValueError, "STALE"):
            s.freeze()
        frame["received_at"] = now
        s.observe(rgb, frame, now - .4, now)
        with self.assertRaisesRegex(ValueError, "DESYNC"):
            s.freeze()

    def test_manual_uniform_scale_keeps_centre_and_aspect(self):
        a = m.Alignment(self.c, self.size)
        centre = a.overlay_centre
        original_x, original_y = a.mx.copy(), a.my.copy()
        a.adjust("NEAR")
        np.testing.assert_allclose(a.overlay_centre, centre)
        np.testing.assert_allclose(a.mx - centre[0], (original_x - centre[0]) * 1.05, atol=5e-5)
        np.testing.assert_allclose(a.my - centre[1], (original_y - centre[1]) * 1.05, atol=5e-5)
        a.adjust("FAR")
        np.testing.assert_allclose(a.mx, original_x, atol=5e-5)
        self.assertFalse(a.manually_tuned)

    def test_manual_profile_tuning_moves_roi_and_reset_recovers_verified_base(self):
        a = m.Alignment(self.c, self.size)
        a.set_registration(*self.solved())
        original = a.matrix.copy()
        for action in ("NEAR", "RIGHT", "DOWN"):
            a.adjust(action)
        self.assertTrue(a.manually_tuned)
        np.testing.assert_array_equal(a.homography, original)
        frame = {"pixels": np.zeros((120, 160), np.uint8), "lo": 200, "hi": 400, "valid_temperature": True}
        frame["pixels"][46, 48] = 254
        x, y = a.mx[46, 48], a.my[46, 48]
        self.assertEqual(m.roi_temperature(frame, (x - .1, y - .1, .2, .2), a)["max"], 40)
        a.adjust("RESET")
        self.assertFalse(a.manually_tuned)
        np.testing.assert_array_equal(a.matrix, original)

    def test_manual_settings_are_independent_and_survive_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "alignment.json")
            a = m.Alignment(self.c, self.size, path)
            a.set_registration(*self.solved())
            a.adjust("FAR")
            a.adjust("LEFT")
            one = a.matrix.copy()
            a.select_distance(2)
            self.assertFalse(a.manually_tuned)
            a.adjust("NEAR")
            two = a.matrix.copy()
            with contextlib.redirect_stdout(io.StringIO()):
                a.save()
            b = m.Alignment(self.c, self.size, path)
            np.testing.assert_array_equal(b.matrix, two)
            b.select_distance(1)
            np.testing.assert_array_equal(b.matrix, one)
            self.assertTrue(b.manually_tuned)
            b.select_distance(3)
            self.assertFalse(b.manually_tuned)

    def test_default_narrow_footprint_preserves_rgb_outside_and_no_roi_temperature(self):
        a = m.Alignment(self.c, self.size)
        self.assertLess(np.count_nonzero(a.mask), 640 * 480 * .3)
        self.assertFalse(a.mask[100, 100])
        frame = {"pixels": np.full((120, 160), 254, np.uint8), "lo": 200, "hi": 400,
                 "valid_temperature": True, "received_at": 1.}
        rgb = np.full((480, 640, 3), (40, 50, 60), np.uint8)
        canvas, _, _ = m.compose(rgb, [], frame, a, 1., 1.)
        np.testing.assert_array_equal(canvas[100, 100], rgb[100, 100])
        self.assertIsNone(m.roi_temperature(frame, (90, 90, 20, 20), a))

    def test_fine_controls_and_joint_scale_bounds(self):
        a = m.Alignment(self.c, self.size)
        a.adjust("RIGHT", fine=True)
        self.assertAlmostEqual(a.tuning[2], .002)
        a.adjust("NEAR", fine=True)
        self.assertAlmostEqual(a.tuning[0], 1.01)
        a.tuning[:2] = [3.99, 2.]
        a.adjust("NEAR")
        self.assertLessEqual(a.tuning[0], 4.)
        self.assertAlmostEqual(a.tuning[0] / a.tuning[1], 3.99 / 2.)

    def test_new_point_calibration_replaces_current_manual_tuning_only(self):
        a = m.Alignment(self.c, self.size)
        a.select_distance(2)
        a.adjust("FAR")
        a.select_distance(1)
        a.adjust("NEAR")
        a.set_registration(*self.solved())
        self.assertFalse(a.manually_tuned)
        a.select_distance(2)
        self.assertTrue(a.manually_tuned)

    def test_invalid_or_wrong_geometry_manual_settings_rejected(self):
        record = {"values": [1., 1., 0., 0.], "source_size": [640, 480], "flips": [True, True],
                  "camera_fps": self.c["camera_fps"]}
        for bad in (dict(record, values=[10, 1, 0, 0]), dict(record, values=[1, 1, float('nan'), 0]),
                    dict(record, source_size=[320, 240])):
            a = m.Alignment(dict(self.c, manual_adjustments={"1m": bad}), self.size)
            self.assertFalse(a.manually_tuned)
            self.assertEqual(a.adjustments, {})


if __name__ == "__main__":
    unittest.main(verbosity=2)
