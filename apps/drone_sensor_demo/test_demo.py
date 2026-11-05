import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from core import DEFAULT, Fusion, observation, validate, save_config, load_config, thermal_candidates


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.c = json.loads(json.dumps(DEFAULT))
        self.f = Fusion(self.c)

    def packet(self, kind="vision", t=0, score=.94, label="drone", x=.5, token=None):
        return observation(kind, x, .5, score, t, t if token is None else token, label)

    def confirmed(self):
        for t in (0, .1, .2):
            result, _ = self.f.update([self.packet(t=t)], t)
        return result[0]

    def test_visual_only_can_confirm(self):
        row = self.confirmed()
        self.assertEqual(row["state"], "VISUAL_CONFIRMED")
        self.assertAlmostEqual(row["score"], .7*.94)

    def test_auxiliary_never_confirms(self):
        for t in (0, .1, .2, .3):
            rows, _ = self.f.update([self.packet(k, t, 1) for k in ("sound", "thermal")], t)
        self.assertEqual(rows[0]["state"], "CANDIDATE")
        self.assertAlmostEqual(rows[0]["score"], .3)

    def test_weak_vision_not_promoted_by_auxiliary(self):
        for t in (0, .1, .2):
            rows, _ = self.f.update([self.packet(t=t, score=.6), self.packet("sound", t, 1), self.packet("thermal", t, 1)], t)
        self.assertEqual(rows[0]["state"], "CANDIDATE")

    def test_duplicate_frame_does_not_confirm(self):
        for i in range(8):
            rows, _ = self.f.update([self.packet()], i*.02)
        self.assertEqual(rows[0]["hits"], 1)

    def test_wrong_label_cannot_confirm(self):
        for t in (0, .1, .2):
            rows, _ = self.f.update([self.packet(t=t, label="bird")], t)
        self.assertEqual(rows[0]["score"], 0)
        self.assertEqual(rows[0]["state"], "CANDIDATE")

    def test_stale_and_future_rejected(self):
        rows, rejected = self.f.update([self.packet(t=-2), self.packet("sound", t=1)], 0)
        self.assertFalse(rows)
        self.assertEqual(len(rejected), 2)

    def test_uncalibrated_does_not_fuse(self):
        p = self.packet("thermal")
        p["calibrated"] = False
        rows, _ = self.f.update([p], 0)
        self.assertFalse(rows)

    def test_wrong_direction_separate_target(self):
        rows, _ = self.f.update([self.packet(), self.packet("sound", x=.9)], 0)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["contributions"]["sound"], 0)

    def test_ambiguous_auxiliary_rejected(self):
        self.f.update([self.packet(x=.45, token="a"), self.packet(x=.55, token="b")], 0)
        rows, rejected = self.f.update([self.packet("sound", t=.1)], .1)
        self.assertIn("sound: ambiguous association", rejected)
        self.assertTrue(all(t["contributions"]["sound"] == 0 for t in rows))

    def test_hold_loses_confirmation_and_expires(self):
        self.confirmed()
        rows, _ = self.f.update([], .6)
        self.assertEqual(rows[0]["state"], "HOLD_UNCONFIRMED")
        self.assertEqual(rows[0]["score"], 0)
        rows, _ = self.f.update([], 1.1)
        self.assertFalse(rows)

    def test_invalid_weights(self):
        for weights in (dict(vision=.4, sound=.3, thermal=.3), dict(vision=float("nan"), sound=.15, thermal=.15)):
            self.c["weights"] = weights
            with self.assertRaises(ValueError):
                validate(self.c)

    def test_configuration_round_trip(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as d:
            p = str(Path(d)/"config.json")
            save_config(p, self.c)
            self.assertEqual(load_config(p), self.c)

    def test_small_thermal_blob_and_flat_frame(self):
        image = np.full((120, 160), 40, np.uint8)
        self.assertEqual(thermal_candidates(image), [])
        image[59:61, 79:81] = 160
        candidates = thermal_candidates(image)
        self.assertTrue(any(abs(c["x"]-79.5) < 2 and abs(c["y"]-59.5) < 2 for c in candidates))

    def test_board_bundle_import_has_no_hardware_side_effects(self):
        import runpy
        api = runpy.run_path(str(Path(__file__).with_name("main.py")), run_name="test_bundle")
        self.assertTrue(callable(api["run_board"]))
        self.assertIsNotNone(api["sound_api"]["SoundDirectionSensor"])

    def test_board_entry_with_fake_devices_and_cleanup(self):
        import runpy
        import types
        from unittest.mock import patch
        api = runpy.run_path(str(Path(__file__).with_name("main.py")), run_name="test_bundle")
        state = dict(loops=0, shown=0, closed=0)
        class Cam:
            def __init__(self, *args, **kwargs): pass
            def read(self, **kwargs): return np.zeros((480, 640, 3), np.uint8)
            def close(self): state["closed"] += 1
        class Model:
            labels = ["drone"]
            def __init__(self, **kwargs): pass
            def input_width(self): return 640
            def input_height(self): return 480
            def detect(self, *args, **kwargs):
                return [types.SimpleNamespace(x=280, y=200, w=40, h=40, score=.94, class_id=0)]
        class Disp:
            def show(self, image): state["shown"] += 1
        class Sound:
            def open_uart(self): pass
            def poll(self, **kwargs): pass
            def snapshot(self): return dict(valid=False, reason="no_data")
            def close(self): state["closed"] += 1
        class Thermal:
            def __init__(self, *args): pass
            def start(self): pass
            def snapshot(self): return None, "OFFLINE", 0
            def close(self): state["closed"] += 1
        def stop():
            state["loops"] += 1
            return state["loops"] > 4
        fake = types.ModuleType("maix")
        fake.app = types.SimpleNamespace(need_exit=stop)
        fake.camera = types.SimpleNamespace(Camera=Cam)
        fake.display = types.SimpleNamespace(Display=Disp)
        fake.nn = types.SimpleNamespace(YOLO11=Model)
        fake.image = types.SimpleNamespace(Format=types.SimpleNamespace(FMT_RGB888=0),
                    image2cv=lambda x, **kw: x, cv2image=lambda x, **kw: x)
        api["thermal_api"]["ThermalReader"] = Thermal
        api["sound_api"]["SoundDirectionSensor"] = Sound
        with patch.dict("sys.modules", maix=fake):
            api["run_board"](str(Path(__file__).parent / "nonexistent_test_config.json"))
        self.assertEqual(state["shown"], 4)
        self.assertEqual(state["closed"], 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
