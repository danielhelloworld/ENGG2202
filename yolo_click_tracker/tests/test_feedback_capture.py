"""A disconnected motor's stale target must never become tuning evidence."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1]/"tools/read_feedback.py"
spec = importlib.util.spec_from_file_location("readonly_feedback", path)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class CaptureTests(unittest.TestCase):
    def test_disabled_stale_target_and_degraded_samples_are_excluded(self):
        row = dict(armed=False,commanded_deg=None,feedback_age_s=.01,response=dict(error_deg=[100.,-100.]))
        bad = dict(row,armed=True,commanded_deg=[0.,0.],feedback_degraded=True)
        stale = dict(bad,feedback_degraded=False,feedback_age_s=.2)
        paused = dict(bad,feedback_degraded=False,motion_paused=True)
        result = capture.summarize(dict(samples=[row,bad,stale,paused]))
        self.assertEqual(result["eligible_samples"], 0)
        self.assertIsNone(result["rms_command_feedback_error_deg"])

    def test_real_armed_feedback_is_reported_with_degree_units(self):
        row = dict(armed=True,commanded_deg=[0.,0.],feedback_age_s=.01,feedback_degraded=False,
                   requested_rate_deg_s=[5.,0.],response=dict(error_deg=[1.,-2.]))
        result = capture.summarize(dict(samples=[row,row]))
        self.assertEqual(result["eligible_samples"], 2)
        self.assertEqual(result["moving_samples"], 2)
        self.assertEqual(result["rms_command_feedback_error_deg"], [1.,2.])


if __name__ == "__main__":
    unittest.main()
