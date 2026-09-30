"""Synthetic raw Thermal160 blobs; tests coordinate selection and target loss."""
from pathlib import Path
import runpy
import types
import unittest

import numpy as np

APP = Path(__file__).resolve().parents[1]
m = types.SimpleNamespace(**runpy.run_path(str(APP / "main.py")))


def frame(blobs, at=10., valid=True):
    pixels = np.zeros((m.THERMAL_H, m.THERMAL_W), np.uint8)
    for x, y, value in blobs:
        pixels[y:y + 8, x:x + 8] = value
    return {"pixels": pixels, "lo": 200, "hi": 500,
            "received_at": at, "valid_temperature": valid}


class HotTrackerTests(unittest.TestCase):
    def setUp(self):
        self.c = dict(m.CONFIG)
        self.a = m.Alignment(self.c, (640, 480))
        self.tracker = m.HotTracker(self.c)
        self.tracker.toggle()

    def canvas_point(self, tx, ty):
        return float(self.a.mx[ty, tx]), float(self.a.my[ty, tx])

    def test_candidates_are_raw_thermal_components_with_temperature(self):
        found = m.hot_candidates(frame([(20, 40, 200), (100, 70, 250)]), self.c)
        self.assertEqual(len(found), 2)
        self.assertEqual(found[0]["box"], (100, 70, 8, 8))
        self.assertEqual(found[0]["area"], 64)
        self.assertAlmostEqual(found[0]["max_c"], 20 + 30 * 250 / 254, places=3)

    def test_tap_selects_one_region_through_alignment_and_tracks_it(self):
        first = frame([(20, 40, 200), (100, 70, 250)])
        self.tracker.update(first, 10.)
        x, y = self.canvas_point(23, 43)
        self.assertTrue(self.tracker.select_canvas(x, y, self.a))
        self.assertEqual(self.tracker.selected["box"], (20, 40, 8, 8))
        self.tracker.update(frame([(24, 42, 190), (100, 70, 254)], 10.1), 10.1)
        self.assertEqual(self.tracker.selected["box"], (24, 42, 8, 8))
        self.assertEqual(self.tracker.misses, 0)
        self.assertIn("HOT #1", self.tracker.status())

    def test_missing_target_becomes_lost_without_switching_to_distant_hotter_blob(self):
        self.tracker.update(frame([(20, 40, 200), (100, 70, 250)]), 10.)
        x, y = self.canvas_point(23, 43)
        self.assertTrue(self.tracker.select_canvas(x, y, self.a))
        for i in range(1, 4):
            self.tracker.update(frame([(100, 70, 254)], 10 + i * .1), 10 + i * .1)
        self.assertEqual(self.tracker.mode, "LOST")
        self.assertIsNone(self.tracker.selected)
        self.tracker.update(frame([(20, 40, 200)], 10.4), 10.4)
        self.assertEqual(self.tracker.mode, "LOST")
        self.assertTrue(self.tracker.select_canvas(x, y, self.a))
        self.assertEqual(self.tracker.mode, "TRACK")

    def test_stale_and_invalid_thermal_never_expose_a_temperature(self):
        first = frame([(20, 40, 200)])
        self.tracker.update(first, 10.)
        x, y = self.canvas_point(23, 43)
        self.assertTrue(self.tracker.select_canvas(x, y, self.a))
        self.tracker.update(first, 10.7)
        self.assertEqual(self.tracker.status(), "HOT WAIT THERMAL")
        self.tracker.update(frame([(20, 40, 200)], 10.75, valid=False), 10.75)
        self.assertEqual(self.tracker.status(), "HOT WAIT THERMAL")
        self.tracker.update(first, 11.)
        self.assertEqual(self.tracker.mode, "LOST")

    def test_touch_ui_selects_after_hot_button_without_changing_saved_alignment(self):
        ui = m.TouchUI(self.a, self.tracker)
        self.tracker.clear()
        labels = {name: bounds for name, bounds in ui.buttons()}
        bx, by, _, _ = labels["HOT"]
        self.assertFalse(ui.touch(bx + 2, by + 2, True))
        ui.touch(0, 0, False)
        self.assertEqual(self.tracker.mode, "PICK")
        self.tracker.update(frame([(20, 40, 200)]), 10.)
        x, y = self.canvas_point(23, 43)
        ui.touch(x, y, True)
        self.assertEqual(self.tracker.mode, "TRACK")
        self.assertFalse(self.a.profiles)
        self.assertFalse(self.a.adjustments)

    def test_draw_marks_selected_region_only_while_thermal_is_visible(self):
        self.tracker.update(frame([(20, 40, 200)]), 10.)
        x, y = self.canvas_point(23, 43)
        self.assertTrue(self.tracker.select_canvas(x, y, self.a))
        blank = np.zeros((self.c["height"], self.c["width"], 3), np.uint8)
        self.tracker.draw(blank, self.a, False)
        self.assertFalse(blank.any())
        self.tracker.draw(blank, self.a, True)
        self.assertTrue(blank.any())


if __name__ == "__main__":
    unittest.main(verbosity=2)
