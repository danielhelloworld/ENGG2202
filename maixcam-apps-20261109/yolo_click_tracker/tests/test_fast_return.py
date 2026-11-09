"""Faster HOME/holding driver caps; synthetic motion is not hardware timing."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m=app.m


class FastReturnTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    make_driver=controls.SeparateControlTests.make_driver
    enable=controls.SeparateControlTests.enable

    def test_home_acceleration_is_not_silently_reduced_by_response_diagnostics(self):
        d,bus=self.make_driver((0.,-75.));self.enable(d)
        d.request_home();d._begin_home()
        d.response.accel=(100.,100.)
        self.assertEqual(d._effective_accel(),(450.,350.))

    def test_fixed_hold_programs_faster_return_speed_without_changing_reference_or_enable(self):
        d,bus=self.make_driver((0.,0.));self.enable(d);d.set_response_context('HOLD')
        before=len(bus.writes);self.clock.sleep(.02);d._measure();d._advance(d.manual_target,d.c['max_rate'])
        self.assertEqual(d.programmed_rpm,[30,25])
        self.assertEqual(d.hold_target,(0.,0.));self.assertEqual(d.sent_target,(0.,0.))
        self.assertFalse(any(p[2] in (3,5,6,8,9,10) for p in bus.writes[before:]))

    def test_updated_home_model_is_faster_than_previous_effective_profile(self):
        durations=[]
        for settings in (dict(home_rate=(108.,90.),home_accel_deg_s2=(153.,127.5),home_jerk_deg_s3=(5000.,5000.)),{}):
            d,bus=self.make_driver((0.,-75.),**settings);self.enable(d);d.request_home()
            started=self.clock.now;d._home_and_clear();durations.append(self.clock.now-started)
            self.assertTrue(d.armed);self.assertEqual(d.phase,'HOME_HOLD')
            self.assertTrue(all(lo<=q<=hi for q,(lo,hi) in zip(d.angles,d.c['limits'])))
            d._disable_all()
        self.assertLess(durations[1],durations[0])

    def test_home_rate_ramp_bounds_acceleration_and_jerk(self):
        d,bus=self.make_driver((0.,-75.));self.enable(d)
        ramp=d.home_ramp;v=0.;previous_a=0.
        for _ in range(100):
            v=ramp.advance(1,v,120.,350.,.02,120.)
            a=ramp.acceleration[1]
            self.assertLessEqual(abs(a),350.+1e-8)
            self.assertLessEqual(abs(a-previous_a),2000.*.02+1e-8)
            self.assertLessEqual(abs(v),120.+1e-8);previous_a=a

    def test_inertia_input_is_not_clipped_to_normal_nine_degree_feedback_lead(self):
        d,bus=self.make_driver((0.,0.));self.enable(d);bus.stuck=True
        d.set_response_context('HOLD');d.request_angle_move(0.,-75.,60.,inertia_test=True)
        for _ in range(45):
            self.clock.sleep(.02);d._measure();d._advance(d.manual_target,d.c['max_rate'])
        self.assertLess(d.sent_target[1],-9.)
        self.assertGreaterEqual(d.sent_target[1],-75.)
        self.assertTrue(d.inertia_test_active);self.assertTrue(d.armed)
        d._cancel_angle('test done');self.assertFalse(d.inertia_test_active)
