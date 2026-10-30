"""Gentler manual ramps and command/feedback phase: synthetic motor tests."""
import unittest
import test_tracker as app
import test_separate_controls as controls
import test_fast_manual as fast

m = app.m


class ManualFaultRegressionTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable
    move = fast.FastManualTests.move

    def manual_driver(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        driver.set_response_context("MANUAL")
        driver.phase = "MANUAL"
        self.clock.pump = lambda:driver.input(-1.,0.)
        driver.input(-1.,0.)  # Inverted mount maps to positive mechanical yaw.
        return driver,bus

    def test_half_speed_gentler_acceleration_without_extending_stall_time(self):
        self.assertEqual(m.CONFIG["manual_rate"],(240.,180.))
        self.assertEqual(m.CONFIG["command_accel_deg_s2"],(900.,750.))
        self.assertEqual(m.CONFIG["manual_jerk_deg_s3"],(6000.,5000.))
        self.assertEqual(m.CONFIG["tracking_rate"],(150.,120.))
        self.assertEqual(m.CONFIG["home_rate"],(108.,90.))
        self.assertAlmostEqual(m.CONFIG["motion_grace_s"]+m.CONFIG["motion_window_s"],1.8)

    def test_first_100ms_is_gentler_than_v5(self):
        speeds = []
        for config,rate,accel in ((dict(m.CONFIG,manual_jerk_deg_s3=(24000.,20000.),manual_response_s=.08),480.,1530.),
                                  (m.CONFIG,240.,765.)):
            ramp = m.ManualRateRamp(config)
            velocity = 0.
            for _ in range(10):
                velocity = ramp.advance(0,velocity,rate,accel,.01,rate)
            speeds.append(velocity)
        self.assertGreater(speeds[1],0.)
        self.assertLess(speeds[1],speeds[0]*.5)

    def test_feedback_phase_can_report_a_valid_held_target_in_one_sample(self):
        driver,bus = self.manual_driver()
        old_stamp = driver.feedback_cache[0][1]
        driver._write(m.set_single_turn_position(1,12.))
        driver.sent_target = (12.,0.)
        self.clock.sleep(.02)
        driver._measure()
        self.assertEqual(driver.angles[0],12.)
        self.assertFalse(any(driver.feedback_issues))
        self.assertGreater(driver.feedback_cache[0][1],old_stamp)
        self.assertTrue(driver.armed)

    def test_phase_credit_does_not_accept_an_uncommanded_jump_after_waiting(self):
        driver,bus = self.manual_driver()
        old_stamp = driver.feedback_cache[0][1]
        self.clock.sleep(.2)
        bus.physical[1] = 20.
        driver._measure()
        self.assertTrue(driver.feedback_issues[0])
        self.assertEqual(driver.feedback_cache[0][1],old_stamp)
        with self.assertRaises(RuntimeError):
            for _ in range(100):
                self.clock.sleep(.01)
                driver._measure()
        self.assertFalse(driver.armed)

    def test_small_feedback_retreat_does_not_retract_a_forward_held_target(self):
        driver,bus = self.manual_driver()
        bus.stuck = True
        driver._write(m.set_single_turn_position(1,12.))
        driver.sent_target = driver.stream_target = (12.,0.)
        driver.position_units = [120,0]
        driver.stream_velocity = (100.,0.)
        bus.physical[1] = -.2
        self.clock.sleep(.02)
        driver._measure()
        before = len([p for p in bus.writes if p[2] == 3])
        driver._advance(driver.angles,driver.c["max_rate"])
        self.assertEqual(driver.stream_target[0],12.)
        self.assertEqual(len([p for p in bus.writes if p[2] == 3]),before)
        self.assertTrue(driver.armed)

    def test_manual_damping_cannot_reverse_small_input_or_move_an_idle_axis(self):
        driver,bus = self.manual_driver()
        self.clock.pump = lambda:driver.input(-.005,0.)
        driver.input(-.005,0.)
        self.clock.sleep(.02)
        driver.response.correction = (-3.,3.)
        driver._advance(driver.angles,driver.c["max_rate"])
        self.assertEqual(driver.stream_target,(0.,0.))
        self.assertEqual(driver.stream_velocity,(0.,0.))

    def test_small_overshoot_does_not_create_an_opposite_target_while_stick_is_forward(self):
        driver,bus = self.manual_driver()
        driver._write(m.set_single_turn_position(1,6.))
        driver.sent_target = driver.stream_target = (6.,0.)
        driver.position_units = [60,0]
        driver.stream_velocity = (30.,0.)
        bus.physical[1] = 7.
        self.clock.sleep(.05)
        driver._measure()
        driver._advance(driver.angles,driver.c["max_rate"])
        self.assertGreaterEqual(driver.sent_target[0],7.)
        self.assertTrue(driver.armed)

    def test_deliberate_stick_reversal_still_works_without_false_fault(self):
        driver,bus = self.manual_driver()
        self.move(driver,(-1.,0.),count=20)
        rows = self.move(driver,(1.,0.),count=40)
        self.assertLess(rows[-1][2][0],-40.)
        self.assertLess(rows[-1][0][0],rows[0][0][0])
        self.assertIsNone(driver.fault)
        self.assertFalse(any(driver.feedback_issues))
        self.assertTrue(driver.armed)


if __name__ == "__main__":
    unittest.main()
