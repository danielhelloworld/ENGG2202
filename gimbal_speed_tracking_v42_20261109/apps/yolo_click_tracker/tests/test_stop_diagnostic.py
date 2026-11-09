"""Normal DISABLED must be distinguished from angle/reference FAULT."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m = app.m


class StopDiagnosticTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable

    def test_signed_driver_angles_keep_saved_zero_and_one_input_inversion(self):
        d,bus = self.make_driver((-.1, -10.))
        self.assertEqual(d.angles,(-.1,-10.))
        self.assertEqual(d.c["motor_signs"],(1,1))
        d.input(.1,.1)
        self.assertEqual(d.joystick,(-.1,-.1))
        self.assertAlmostEqual(m.signed_mechanical(359.9),-.1)
        self.assertEqual(d.reference_zero,(0.,0.))
        self.assertFalse(any(p[2] in (8,9,10) for p in bus.writes))

    def test_visual_timeout_records_normal_disable_reason_without_angle_fault(self):
        d,bus = self.make_driver((0.,0.))
        self.enable(d)
        hub = m.TrackingHub(d,dict(m.CONFIG),self.clock.monotonic)
        hub.offer_frame([],640,480,self.clock.now)
        self.clock.sleep(2.1)
        hub.tick()
        self.assertFalse(d.arm_requested)
        self.assertIn("VISION_TIMEOUT",d.last_disable_reason["reason"])
        self.assertIsNone(d.fault)
        self.assertEqual(d.last_disable_reason["actual_deg"],(0.,0.))

    def test_disabled_measurement_does_not_report_enabled_control_pause(self):
        d,bus = self.make_driver((0.,0.))
        self.clock.pump = None
        self.clock.sleep(.7)
        d._measure()
        self.assertFalse(d.motion_paused)
        self.assertFalse(d.armed)


if __name__ == "__main__":
    unittest.main()
