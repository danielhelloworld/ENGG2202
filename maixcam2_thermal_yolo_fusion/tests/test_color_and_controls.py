"""Check RGB ownership/channel order and independent on-screen layer controls."""
from pathlib import Path
import runpy
import types
import unittest

import numpy as np

APP = Path(__file__).resolve().parents[1]
m = types.SimpleNamespace(**runpy.run_path(str(APP / "main.py")))


class FakeImage:
    def __init__(self, pixels, fmt="RGB"):
        self.pixels, self.fmt = pixels, fmt

    def format(self):
        return self.fmt

    def to_format(self, fmt):
        assert fmt == "RGB"
        return FakeImage(self.pixels[..., ::-1].copy(), fmt)


class FakeMaixImage:
    Format = types.SimpleNamespace(FMT_RGB888="RGB")

    def __init__(self):
        self.calls = []

    def image2cv(self, frame, ensure_bgr, copy):
        self.calls.append((ensure_bgr, copy))
        # MaixPy's documented behavior: copy=False ignores ensure_bgr.
        return frame.pixels.copy() if copy else frame.pixels


class ColorAndControlTests(unittest.TestCase):
    def setUp(self):
        self.c = dict(m.CONFIG)
        self.a = m.Alignment(self.c, (640, 480))

    def test_camera_rgb_remains_rgb_and_owns_storage(self):
        pixels = np.array([[[255, 0, 0], [0, 0, 255]]], np.uint8)
        fake = FakeMaixImage()
        rgb = m.camera_image_to_rgb(FakeImage(pixels), fake)
        self.assertEqual(fake.calls, [(False, True)])
        self.assertEqual(rgb[0, 0].tolist(), [255, 0, 0])
        self.assertEqual(rgb[0, 1].tolist(), [0, 0, 255])
        pixels[:] = 0
        self.assertEqual(rgb[0, 0].tolist(), [255, 0, 0])

    def test_bgr_camera_frame_is_converted_once_to_rgb(self):
        fake = FakeMaixImage()
        rgb = m.camera_image_to_rgb(FakeImage(np.array([[[0, 0, 255]]], np.uint8), "BGR"), fake)
        self.assertEqual(rgb[0, 0].tolist(), [255, 0, 0])

    def test_heat_switch_does_not_disable_temperature_or_tracking(self):
        thermal = m.demo_frame()[1]
        rgb = np.zeros((480, 640, 3), np.uint8)
        rgb[200:240, 220:260] = (220, 10, 35)
        now = thermal["received_at"]
        base, _, _ = m.compose(rgb, [], thermal, self.a, now, now, show_overlay=False)
        fused, _, _ = m.compose(rgb, [], thermal, self.a, now, now, show_overlay=True)
        self.assertEqual(base[210, 230].tolist(), [220, 10, 35])
        self.assertFalse(np.array_equal(base[210, 230], fused[210, 230]))
        self.assertTrue((base[76:95] != 0).any())  # temperature/status remains on RGB

    def test_temp_switch_hides_all_numeric_temperatures_and_keeps_yolo(self):
        rgb, thermal, detections = m.demo_frame()
        now = thermal["received_at"]
        canvas, values, _ = m.compose(rgb, detections, thermal, self.a, now, now,
                                       show_temperature=False, show_overlay=False)
        self.assertEqual(values, [None])
        self.assertEqual(canvas.shape, (480, 640, 3))
        self.assertFalse(np.array_equal(canvas[120, 213], rgb[120, 213]))  # YOLO box still drawn

    def test_touch_buttons_independently_toggle_temp_heat_and_hot(self):
        tracker = m.HotTracker(self.c)
        ui = m.TouchUI(self.a, tracker)
        boxes = dict(ui.buttons())
        self.assertEqual(len([k for k in boxes if k in ("TEMP", "HEAT", "HOT")]), 3)
        for name in ("TEMP", "HEAT", "HOT"):
            x, y, _, _ = boxes[name]
            ui.touch(x + 2, y + 2, True)
            ui.touch(x + 2, y + 2, False)
        self.assertFalse(ui.show_temperatures)
        self.assertFalse(ui.show_heat)
        self.assertEqual(tracker.mode, "PICK")

    def test_selected_hotspot_remains_visible_when_heat_is_off_and_temp_is_off(self):
        ui = m.TouchUI(self.a, m.HotTracker(self.c))
        ui.hot_tracker.toggle()
        rgb, thermal, _ = m.demo_frame()
        now = thermal["received_at"]
        ui.hot_tracker.update(thermal, now)
        candidate = ui.hot_tracker.candidates[0]
        tx, ty = (int(round(v)) for v in candidate["center"])
        x, y = float(self.a.mx[ty, tx]), float(self.a.my[ty, tx])
        self.assertTrue(ui.hot_tracker.select_canvas(x, y, self.a))
        ui.show_heat = ui.show_temperatures = False
        canvas, _, _ = m.compose(rgb, [], thermal, self.a, now, now,
                                  show_temperature=ui.show_temperatures, show_overlay=ui.show_heat)
        before = canvas.copy()
        ui.hot_tracker.draw(canvas, self.a, True, ui.show_temperatures)
        self.assertFalse(np.array_equal(canvas, before))
        self.assertNotIn("C", ui.hot_tracker.status(ui.show_temperatures))


if __name__ == "__main__":
    unittest.main(verbosity=2)
