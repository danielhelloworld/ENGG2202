"""Delayed moving-target prediction; synthetic timing and protocol evidence."""
import unittest
import test_tracker as app
import test_native_tracking as native

m=app.m


class PursuitTests(unittest.TestCase):
    setUp=native.NativeTrackingTests.setUp
    enable=native.NativeTrackingTests.enable
    driver=native.NativeTrackingTests.driver
    tick=native.NativeTrackingTests.tick

    def test_fast_consistent_motion_converges_within_240ms(self):
        p=m.MotionPredictor(dict(m.CONFIG))
        for i in range(7):p.observe((.6*i*.04,0.),(0.,0.),1.+i*.04)
        self.assertTrue(p.ready)
        self.assertGreater(p.motion_velocity[0],.54)
        p.observe((.6*6*.04,0.),(0.,0.),1.28)
        p.observe((.6*6*.04,0.),(0.,0.),1.32)
        p.observe((.6*6*.04,0.),(0.,0.),1.36)
        self.assertEqual(p.motion_velocity,(0.,0.))

    def test_reversal_discards_forward_prediction_on_first_reverse_sample(self):
        p=m.MotionPredictor(dict(m.CONFIG))
        for i in range(7):p.observe((.6*i*.04,0.),(0.,0.),1.+i*.04)
        p.observe((.6*5*.04,0.),(0.,0.),1.28)
        self.assertLess(p.motion_velocity[0],0.)

    def test_native_world_aim_has_no_image_prediction_added_twice(self):
        core=m.ClickTracker(dict(m.CONFIG));tr=app.track()
        core.select(tr['key'],[tr],1.)
        core.predictor.samples=3;core.predictor.velocity=(.6,0.)
        core.predictor.observe=lambda *args:None
        core.step([tr],1.1,pose=(0.,0.),current_pose=(-5.,0.),latency=.1)
        self.assertAlmostEqual(core.aim_goal[0],-.125*87.)
        self.assertAlmostEqual(core.aim_velocity[0],-.6*87.)

    def test_incomplete_velocity_history_does_not_drive_feedforward(self):
        core=m.ClickTracker(dict(m.CONFIG));tr=app.track()
        core.select(tr['key'],[tr],1.)
        core.step([tr],1.,pose=(0.,0.),current_pose=(0.,0.))
        tr.update(cx=tr['cx']+.02)
        core.step([tr],1.04,pose=(0.,0.),current_pose=(0.,0.))
        self.assertFalse(core.predictor.ready)
        self.assertEqual(core.aim_velocity,(0.,0.))

    def test_uart_compensates_send_time_age_and_keeps_static_goal_unchanged(self):
        d,bus=self.driver()
        stamp=self.clock.now-.08
        d.offer_tracking_goal((10.,0.),stamp,(True,False),velocity=(40.,0.))
        self.tick(d)
        self.assertGreaterEqual(d.sent_target[0],14.)
        self.assertGreater(d.native_tracker.pursuit[0],4.)
        d.offer_tracking_goal((10.,0.),self.clock.now,(True,False))
        self.tick(d)
        self.assertEqual(d.sent_target[0],10.)
        self.assertEqual(d.native_tracker.pursuit,[0.,0.])

    def test_coast_and_zenith_plans_can_disable_additional_extrapolation(self):
        d,bus=self.driver()
        d.offer_tracking_goal((10.,0.),self.clock.now-.08,(True,False),velocity=(40.,0.),extrapolate=False)
        self.tick(d)
        self.assertEqual(d.sent_target[0],10.)
        self.assertEqual(d.native_tracker.pursuit,[0.,0.])

    def test_local_hub_extrapolates_only_real_low_elevation_tracking(self):
        d,bus=self.driver()
        hub=m.TrackingHub(d,d.c,self.clock.monotonic)
        hub.latest={'stamp':self.clock.now}
        core=hub.tracking;core.selected=(0,1);core.state='TRACK'
        core.last_seen=self.clock.now;core.aim_goal=(10.,0.)
        core.aim_velocity=(40.,0.);core.native_active_axes=(True,False)
        hub.tick()
        self.assertTrue(d.native_tracker.message[-1])
        core.overhead.state='GEOMETRY'
        hub.tick()
        self.assertFalse(d.native_tracker.message[-1])
        core.overhead.state='NORMAL';core.state='COAST'
        core.last_tracked_goal=(10.,0.);core.last_tracked_velocity=(40.,0.)
        hub.tick()
        self.assertFalse(d.native_tracker.message[-1])

    def test_prediction_is_bounded_and_pitch_never_flips(self):
        c=dict(m.CONFIG)
        self.assertEqual(m.pursuit_shift(1000.,5.,0,c),5.)
        self.assertEqual(m.pursuit_shift(-1000.,5.,1,c),-3.)
        d,bus=self.driver((0.,-85.))
        d.offer_tracking_goal((0.,-89.),self.clock.now-.1,(False,True),velocity=(0.,-80.))
        self.tick(d)
        self.assertGreaterEqual(d.sent_target[1],-90.)

    def test_new_prediction_bounds_reject_nan_and_excessive_values(self):
        for key,value in [('native_pursuit_lead_s',float('nan')),('native_prediction_age_s',.4),
                          ('native_pursuit_max_deg',(20.,5.)),('prediction_fast_tau_s',0.)]:
            with self.subTest(key=key):
                with self.assertRaises(ValueError):m.validate_visual_config(dict(m.CONFIG,**{key:value}))
