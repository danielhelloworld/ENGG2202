"""Software-loop integration and profile persistence using synthetic tracks."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import test_tracker as app

m = app.m


class SoftwareAutoTuneTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.c = copy.deepcopy(m.CONFIG)
        self.driver = app.FakeDriver()
        self.driver.confirm_reference()
        self.driver.request_arm(True)
        self.hub = m.TrackingHub(self.driver, self.c, lambda: self.now)
        self.tracks = [app.track(box=(240, 180, 160, 120))]
        self.hub.offer_frame(self.tracks, 640, 480, self.now)
        self.hub.tracking.select((0, 1), self.tracks, self.now)

    def test_manual_input_restores_original_profile_and_clears_offsets(self):
        original = {key: self.c[key] for key in m.SoftwareAutoTuner.KEYS}
        self.hub.software_autotune.start(1)
        for _ in range(15):
            self.now += .05
            self.hub.offer_frame(self.tracks, 640, 480, self.now)
            self.hub.tick()
        self.assertTrue(self.hub.software_autotune.active)
        self.assertNotEqual(self.hub.tracking.tuning_offset, (0., 0.))
        self.hub.local_action(vector=(.2, 0.))
        self.assertFalse(self.hub.software_autotune.active)
        self.assertEqual(self.hub.tracking.tuning_offset, (0., 0.))
        self.assertEqual({key: self.c[key] for key in original}, original)
        self.assertTrue(self.driver.armed)

    def test_far_target_is_rejected_before_experiment_motion(self):
        self.hub.offer_frame([app.track()], 640, 480, self.now)
        with self.assertRaises(RuntimeError):
            self.hub.software_autotune.start(0)
        self.assertFalse(self.hub.software_autotune.active)
        self.assertEqual(self.hub.tracking.tuning_offset, (0., 0.))

    def test_web_handoff_leaves_onboard_experiment_running(self):
        token = self.hub.connect()["token"]
        self.hub.command(dict(token=token, seq=1, action="autotune", axis=1, x=0, y=0))
        self.hub.background(token)
        self.now += .1
        self.hub.offer_frame(self.tracks, 640, 480, self.now)
        self.hub.tick()
        self.assertTrue(self.hub.software_autotune.active)
        self.assertTrue(self.driver.armed)
        self.assertEqual(self.hub.state()["control_owner"], "LOCAL")

    def test_target_loss_restores_baseline_and_cancels_the_experiment(self):
        self.hub.software_autotune.start(1)
        self.now += .05
        self.hub.offer_frame([], 640, 480, self.now)
        self.assertFalse(self.hub.software_autotune.active)
        self.assertEqual(self.hub.tracking.tuning_offset, (0., 0.))
        self.assertIsNone(self.hub.tracking.selected)

    def test_profile_file_roundtrip_and_configuration_fingerprint(self):
        tuner = self.hub.software_autotune
        with tempfile.TemporaryDirectory(dir=app.ROOT) as folder:
            self.c["software_autotune_profile"] = str(Path(folder)/"profile.json")
            tuner.start(1)
            with self.assertRaises(RuntimeError):
                tuner.save()
            # Exercise persistence with a completed-search fixture. Search and
            # verification dynamics are tested separately with a first-order plant.
            tuner.session.active = False
            tuner.session.state = "COMPLETE"
            saved = tuner.save()
            self.assertEqual(saved, tuner.saved_path)
            self.assertTrue(tuner.load())
            data = json.loads(Path(saved).read_text(encoding="utf-8"))
            data["model_path"] = "different.mud"
            Path(saved).write_text(json.dumps(data), encoding="utf-8")
            before = self.c["ema"]
            with self.assertRaises(ValueError):
                tuner.load()
            self.assertEqual(self.c["ema"], before)


if __name__ == "__main__":
    unittest.main()
