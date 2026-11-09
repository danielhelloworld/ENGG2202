"""Regression for the live LOST Unsafe-single-turn fault, not a hardware test."""
import unittest
from unittest.mock import patch
import test_tracker as app
import test_separate_controls as controls

m = app.m


class LossHoldReplanTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable

    def test_fast_loss_hold_feedback_refresh_replans_without_disarm(self):
        for context in ("LOST","HOLD","COAST"):
            with self.subTest(context=context):
                d,bus = self.make_driver((-30.5,-24.1))
                self.enable(d)
                d.set_response_context(context)
                d.sent_target = (-28.7,-22.7)
                d.position_units = [3313,3373]
                d.stream_velocity = (-63.,-49.)
                d._release_hold()
                d.phase = "ARMED"
                count = [0]
                original = d._require_reference
                def moving_feedback(fresh=False):
                    original(fresh)
                    if fresh:
                        count[0] += 1
                        if count[0] == 2:
                            d.angles = (-34.5,-24.1)
                            bus.physical[1] = -34.5
                before = len([p for p in bus.writes if p[2] == 3])
                with patch.object(d,"_require_reference",side_effect=moving_feedback):
                    d._advance(d.manual_target,d.c["max_rate"])
                self.assertTrue(d.armed)
                self.assertTrue(d.hold_pending)
                self.assertIsNone(d.fault)
                self.assertEqual(d.stream_target,d.angles)
                self.assertEqual(len([p for p in bus.writes if p[2] == 3]),before)
                self.clock.sleep(.02)
                d._measure()
                d._advance(d.manual_target,d.c["max_rate"])
                self.assertTrue(d.armed)
                self.assertFalse(d.hold_pending)
                self.assertEqual(d.stream_velocity,(0.,0.))
                d._disable_all()

    def test_loss_replan_does_not_allow_outside_work_limit(self):
        d,bus = self.make_driver((0.,29.))
        self.enable(d)
        d.set_response_context("LOST")
        with self.assertRaisesRegex(RuntimeError,"Unsafe single-turn step"):
            d._write(m.set_single_turn_position(2,31.))

    def test_gentler_tracking_profile(self):
        self.assertEqual(m.CONFIG["tracking_rate"],(75.,60.))
        self.assertEqual(m.CONFIG["tracking_accel_deg_s2"],(180.,150.))
        self.assertEqual(m.CONFIG["tracking_position_lead_deg"],(6.,5.))
        self.assertEqual(m.CONFIG["tracking_jerk_deg_s3"],(1200.,1000.))


if __name__ == "__main__":
    unittest.main()
