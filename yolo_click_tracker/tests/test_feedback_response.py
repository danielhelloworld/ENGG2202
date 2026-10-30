"""Loss debounce and bounded response in PC models, never torque measurements."""
import math
import unittest
import test_separate_controls as controls
import test_tracker as app

m = app.m


class FeedbackHealthTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable

    def arm(self, positions=(0., 0.)):
        driver,bus = self.make_driver(positions)
        self.enable(driver)
        return driver,bus

    def test_one_missing_reply_pauses_positions_and_recovers_without_disabling(self):
        driver,bus = self.arm()
        stamp = driver.feedback_cache[0][1]
        bus.drop_feedback = {1}
        driver._measure()
        self.assertTrue(driver.snapshot()["feedback_degraded"])
        self.assertEqual(driver.feedback_cache[0][1], stamp)
        count = len([p for p in bus.writes if p[2] == 3])
        driver.phase = "MANUAL"
        driver.input(1., 0.)
        driver._advance((10., 0.), driver.c["max_rate"])
        self.assertEqual(len([p for p in bus.writes if p[2] == 3]), count)
        self.assertTrue(driver.armed)
        bus.drop_feedback.clear()
        driver._measure()
        self.assertFalse(driver.snapshot()["feedback_degraded"])
        self.assertTrue(driver.armed)
        self.assertEqual(driver.stream_target, driver.angles)

    def test_one_axis_missing_for_100ms_disables_both_even_if_other_axis_replies(self):
        driver,bus = self.arm()
        bus.drop_feedback = {1}
        with self.assertRaises(RuntimeError):
            for _ in range(20):
                self.clock.sleep(.01)
                driver._measure()
        elapsed = driver.last_feedback_fault["failed_query_duration_s"][0]
        self.assertGreaterEqual(elapsed, driver.c["feedback_loss_grace_s"])
        self.assertLess(elapsed, driver.c["feedback_loss_grace_s"]+driver.c["feedback_timeout"]+.02)
        self.assertFalse(driver.armed or driver.arm_requested or driver.reference_valid)
        self.assertFalse(any(bus.enabled.values()))

    def test_cached_feedback_does_not_refresh_the_control_age(self):
        driver,bus = self.arm()
        before = driver.feedback_time
        bus.drop_feedback = {1}
        self.clock.sleep(.02)
        driver._measure()
        self.assertLessEqual(driver.feedback_time, before+.001)
        self.assertGreater(driver.snapshot()["age"], .02)

    def test_one_suspicious_jump_is_rejected_without_changing_zero_or_disabling(self):
        driver,bus = self.arm()
        original = driver.feedback_cache[0]
        bus.physical[1] = 20.
        driver._measure()
        self.assertEqual(driver.feedback_cache[0], original)
        self.assertEqual(driver.angles[0], 0.)
        self.assertTrue(driver.armed)
        bus.physical[1] = 0.
        driver._measure()
        self.assertFalse(driver.snapshot()["feedback_degraded"])
        self.assertTrue(driver.reference_valid)

    def test_persistent_untrusted_jump_expires_instead_of_rebasing_zero(self):
        driver,bus = self.arm()
        bus.physical[1] = 20.
        with self.assertRaises(RuntimeError):
            for _ in range(100):
                self.clock.sleep(.01)
                driver._measure()
        self.assertFalse(driver.reference_valid)
        self.assertEqual(driver.reference_zero, (0., 0.))
        self.assertFalse(any(p[2] in (8, 9, 10) for p in bus.writes))

    def test_small_feedback_margin_does_not_allow_outside_position_commands(self):
        driver,bus = self.arm((0., 29.8))
        bus.physical[2] = 30.2
        driver._measure()
        self.assertTrue(driver.armed)
        with self.assertRaisesRegex(RuntimeError, "Unsafe single-turn step"):
            driver._write(m.set_single_turn_position(2, 30.1))
        bus.physical[2] = 31.
        driver._measure()
        self.assertTrue(driver.armed)
        self.assertIsNotNone(driver.motion_warning)
        self.clock.sleep(.35)
        with self.assertRaisesRegex(RuntimeError, "outside normal work limits"):
            driver._measure()
        self.assertFalse(driver.armed)

    def test_speed_timeout_does_not_count_as_zero_speed_for_home_settling(self):
        driver,bus = self.arm()
        bus.drop_feedback = {1}
        self.assertTrue(math.isnan(driver._feedback(1, m.FEEDBACK_SPEED)))
        self.assertTrue(driver.armed)

    def test_uart_write_failure_is_not_disguised_as_a_transient_missing_reply(self):
        driver,bus = self.arm()
        bus.write = lambda data:len(data)-1
        with self.assertRaisesRegex(RuntimeError, "Incomplete motor UART write"):
            driver._measure()
        self.assertFalse(driver.armed)

    def test_default_saved_zero_is_confirmed_and_recheck_does_not_enable(self):
        driver,bus = self.make_driver()
        self.assertTrue(driver.zero_confirmed)
        driver.request_recheck()
        driver._perform_recheck()
        self.assertTrue(driver.zero_confirmed)
        self.assertFalse(driver.armed or driver.arm_requested)
        self.assertFalse(any(p[2] in (8, 9, 10) for p in bus.writes))

    def test_telemetry_is_bounded_and_disabled_targets_are_not_tuning_samples(self):
        driver,bus = self.make_driver(telemetry_samples=20)
        for _ in range(30):
            self.clock.sleep(.11)
            driver._measure()
        rows = driver.telemetry()["samples"]
        self.assertEqual(len(rows), 20)
        self.assertTrue(all(r["commanded_deg"] is None and not r["armed"] for r in rows))
        self.assertTrue(all(r["response"]["drive_scale"] == (1., 1.) for r in rows))

    def test_repeated_short_packet_gaps_cannot_erase_persistent_stall_detection(self):
        driver,bus = self.arm()
        bus.stuck = True
        driver.phase = "MANUAL"
        self.clock.pump = lambda:driver.input(1., 0.)
        caught = None
        for i in range(180):
            self.clock.sleep(.02)
            bus.drop_feedback = {1} if i % 15 == 5 else set()
            try:
                driver._measure()
                driver._advance((0., 0.), driver.c["max_rate"])
            except RuntimeError as exc:
                caught = exc
                driver._record_fault(exc)
                break
        self.assertIsNotNone(caught)
        self.assertIn("SUSPECTED_BLOCK", str(caught))
        self.assertFalse(driver.armed)
        self.assertFalse(any(bus.enabled.values()))

    def test_feedback_loss_threshold_can_be_changed_in_one_setting(self):
        driver,bus = self.make_driver(feedback_loss_grace_s=.2)
        m.validate_visual_config(driver.c)
        self.enable(driver)
        self.clock.sleep(.13)
        driver._measure()
        self.assertTrue(driver.armed)

    def test_loss_coast_is_not_amplified_by_adaptive_rate_compensation(self):
        driver,bus = self.arm()
        driver.phase = "MANUAL"
        driver.set_response_context("COAST")
        self.clock.pump = lambda:driver.input(1., 0.)
        for _ in range(12):
            self.clock.sleep(.02)
            driver._measure()
            driver.response.scale = (1.15, 1.15)
            driver.response.correction = (-3., 0.)
            driver._advance((0., 0.), driver.c["max_rate"])
            self.assertLessEqual(abs(driver.stream_velocity[0]), 4.+1e-9)


class ResponseTunerTests(unittest.TestCase):
    def update_motion(self, lag, speed, request, enabled=True):
        tuner = m.ResponseTuner(dict(m.CONFIG))
        for i in range(51):
            actual = i*.04*speed
            tuner.update(1.+i*.04, (actual, 0.), (actual+lag, 0.), (request, 0.), enabled)
        return tuner

    def test_real_progress_with_lag_gets_bounded_compensation(self):
        tuner = self.update_motion(.9, 5., 8.)
        self.assertGreater(tuner.scale[0], 1.)
        self.assertLessEqual(tuner.scale[0], 1.15)
        self.assertGreater(tuner.correction[0], 0.)
        self.assertLessEqual(tuner.accel[0], m.CONFIG["command_accel_deg_s2"][0])

    def test_overshoot_reduces_response_instead_of_pushing_harder(self):
        tuner = self.update_motion(-.6, 10., 5.)
        self.assertLess(tuner.scale[0], 1.)
        self.assertLess(tuner.accel[0], m.CONFIG["command_accel_deg_s2"][0]*.85)
        self.assertLess(tuner.correction[0], 0.)

    def test_stuck_axis_never_escalates_gain_or_acceleration(self):
        tuner = self.update_motion(2., 0., 10.)
        self.assertLessEqual(tuner.scale[0], 1.)
        self.assertEqual(tuner.correction[0], 0.)
        self.assertEqual(tuner.status[0], "NO_PROGRESS")

    def test_disabled_or_unhealthy_feedback_cannot_tune_from_old_error(self):
        tuner = self.update_motion(.9, 5., 8., enabled=False)
        self.assertEqual(tuner.scale, (1., 1.))
        self.assertEqual(tuner.error, (0., 0.))
        tuner = self.update_motion(.9, 5., 8.)
        tuner.update(4., (0., 0.), (2., 0.), (10., 0.), True, healthy=False)
        self.assertEqual(tuner.scale, (1., 1.))

    def test_gain_changes_are_slew_limited_and_no_motor_pid_or_torque_is_claimed(self):
        tuner = m.ResponseTuner(dict(m.CONFIG))
        last = 1.
        for i in range(40):
            tuner.update(1.+i*.04, (i*.2, 0.), (i*.2+.9, 0.), (8., 0.), True)
            self.assertLessEqual(abs(tuner.scale[0]-last), m.CONFIG["adaptive_gain_slew_s"]*.04+1e-9)
            last = tuner.scale[0]
        self.assertFalse(tuner.snapshot()["torque_feedback_available"])
        self.assertFalse(tuner.snapshot()["driver_pid_modified"])

    def test_center_crossing_does_not_keep_the_previous_visual_push(self):
        core = m.ClickTracker(dict(m.CONFIG, prediction_enabled=False))
        right = app.track(box=(400, 200, 80, 80))
        left = app.track(box=(160, 200, 80, 80))
        core.select((0, 1), [right], 1.)
        core.step([right], 1.1)
        self.assertGreater(core.vector[0], 0.)
        core.step([left], 1.2)
        self.assertLess(core.vector[0], 0.)


if __name__ == "__main__":
    unittest.main()
