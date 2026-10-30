"""Synthetic motion only: local constant-velocity prediction, not real accuracy."""
import unittest
import test_tracker as app

m = app.m


class PredictionTests(unittest.TestCase):
    def moving_core(self):
        core = m.ClickTracker(dict(m.CONFIG))
        core.select((0, 1), [app.track()], 1.)
        for i in range(6):
            tr = app.track(box=(320+i*6.4, 160, 80, 80))
            core.step([tr], 1.+i*.1, (0., 0.))
        return core

    def test_moving_target_has_a_bounded_lead_after_stable_samples(self):
        core = self.moving_core()
        self.assertTrue(core.prediction_active)
        actual_x = (320+5*6.4+40)/640
        self.assertGreater(core.prediction_point[0], actual_x)
        self.assertLessEqual(core.prediction_point[0]-actual_x, m.CONFIG["prediction_max_shift"])

    def test_own_camera_rotation_is_not_estimated_as_target_motion(self):
        predictor = m.MotionPredictor(dict(m.CONFIG))
        for i in range(6):
            predictor.observe((.1+i/87., 0.), (float(i), 0.), 1.+i*.1)
        self.assertTrue(predictor.ready)
        self.assertEqual(predictor.motion_velocity, (0., 0.))

    def test_outlier_jump_resets_velocity_instead_of_extrapolating(self):
        p = self.moving_core().predictor
        p.observe((-.49, .4), (0., 0.), 1.6)
        self.assertFalse(p.ready)
        self.assertEqual(p.motion_velocity, (0., 0.))

    def test_direction_reversal_does_not_keep_predicting_the_old_direction(self):
        p = self.moving_core().predictor
        previous = p.previous[0]
        p.observe((previous[0]-.02, previous[1]), (0., 0.), 1.6)
        self.assertLess(p.motion_velocity[0], 0.)

    def test_long_gap_or_missing_pose_resets_estimate(self):
        p = self.moving_core().predictor
        p.observe((.2, .1), (0., 0.), 2.)
        self.assertFalse(p.ready)
        p.observe((.21, .1), None, 2.1)
        self.assertEqual(p.samples, 0)

    def test_low_confidence_does_not_support_prediction(self):
        p = self.moving_core().predictor
        p.observe((.2, .1), (0., 0.), 1.6, score=.5)
        self.assertFalse(p.ready)

    def test_coast_expires_without_new_frames_and_does_not_extend_lost_timer(self):
        core = self.moving_core()
        last_seen = core.last_seen
        core.step([], last_seen+.05, (0., 0.))
        self.assertEqual(core.state, "COAST")
        self.assertTrue(any(core.vector))
        for v,cap,rate in zip(core.vector, m.CONFIG["prediction_coast_rate"], m.CONFIG["max_rate"]):
            self.assertLessEqual(abs(v)*rate, cap)
        self.assertEqual(core.drive(last_seen+.16), (0., 0.))
        self.assertEqual(core.state, "LOST")
        self.assertEqual(core.last_seen, last_seen)
        self.assertEqual(core.step([], last_seen+2.01), "home")

    def test_clear_removes_prediction_and_coast_history(self):
        core = self.moving_core()
        core.clear(arm_c=True)
        self.assertFalse(core.prediction_active)
        self.assertFalse(core.predictor.ready)
        self.assertEqual(core.vector, (0., 0.))
        self.assertIsNone(core.step([], 1.6))

    def test_predictor_can_be_disabled_for_board_comparison(self):
        core = m.ClickTracker(dict(m.CONFIG, prediction_enabled=False))
        core.select((0, 1), [app.track()], 1.)
        for i in range(10):
            core.step([app.track(box=(320+i*6.4, 160, 80, 80))], 1.+i*.1, (0., 0.))
        self.assertFalse(core.prediction_active)

    def test_feedback_rate_does_not_scale_with_detection_frequency(self):
        rates = []
        for fps in (10, 15):
            core = m.ClickTracker(dict(m.CONFIG, prediction_enabled=False))
            tracks = [app.track(box=(.54*640-40, 200, 80, 80))]
            core.select((0, 1), tracks, 1.)
            for i in range(fps*4):
                core.step(tracks, 1.+i/fps, (0., 0.))
            rates.append(core.vector[0]*m.CONFIG["max_rate"][0])
        self.assertAlmostEqual(rates[0], rates[1], delta=.05)

    def test_hub_tick_stops_coast_between_detector_frames(self):
        now = [1.]
        driver = app.FakeDriver()
        driver.confirmed = driver.armed = driver.arm_requested = True
        hub = m.TrackingHub(driver, dict(m.CONFIG), lambda:now[0])
        first = hub.offer_frame([app.track()], 640, 480, now[0], (0., 0.))
        hub.local_action(click=(first["id"], .625, .375))
        for i in range(6):
            now[0] = 1.+i*.1
            hub.offer_frame([app.track(box=(320+i*6.4, 120, 160, 120))], 640, 480, now[0], (0., 0.))
        now[0] = 1.55
        hub.offer_frame([], 640, 480, now[0], (0., 0.))
        hub.tick()
        self.assertTrue(any(driver.inputs[-1]))
        now[0] = 1.67
        hub.tick()
        self.assertEqual(driver.inputs[-1], (0., 0.))

