"""Full yaw rotation uses unwrapped feedback and modulo-360 wire targets."""
import json
import unittest
import test_native_tracking as native
import test_separate_controls as controls
import test_tracker as app

m=app.m


class ContinuousYawTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    driver=native.NativeTrackingTests.driver
    tick=native.NativeTrackingTests.tick
    enable=controls.SeparateControlTests.enable

    def test_native_motion_crosses_positive_and_negative_wrap_in_the_same_direction(self):
        for start,goal in ((170.,195.),(-170.,-195.)):
            with self.subTest(start=start):
                d,bus=self.driver((start,0.),continuous_yaw=True)
                d.offer_tracking_goal((goal,0.),self.clock.now,(True,False))
                for _ in range(25):
                    self.tick(d)
                    d.offer_tracking_goal((goal,0.),self.clock.now,(True,False))
                self.assertAlmostEqual(d.sent_target[0],goal)
                self.assertAlmostEqual(d.angles[0],goal,delta=.5)
                self.assertIsNone(d.fault)
                self.assertTrue(d.armed)
                self.assertGreater(d.angles[0]*start,0.)
                self.assertFalse(d.snapshot()['outside_work_limits'])
                d._disable_now()

    def test_manual_motion_does_not_stop_at_old_limit_or_wrap_boundary(self):
        d,bus=self.driver((145.,0.),continuous_yaw=True)
        d.set_response_context('MANUAL')
        self.clock.pump=lambda:d.input(-.5,0.)
        for _ in range(65):
            d.input(-.5,0.)  # Configured input sign -> positive mechanical yaw.
            self.tick(d)
        self.assertGreater(d.angles[0],210.)
        self.assertGreater(d.sent_target[0],210.)
        self.assertIsNone(d.fault)

    def test_home_after_crossing_wrap_uses_nearest_zero_without_unwinding(self):
        for start,goal,expected in ((170.,195.,360.),(-170.,-195.,-360.)):
            with self.subTest(goal=goal):
                d,bus=self.driver((start,0.),continuous_yaw=True)
                d.offer_tracking_goal((goal,0.),self.clock.now,(True,False))
                for _ in range(25):
                    self.tick(d);d.offer_tracking_goal((goal,0.),self.clock.now,(True,False))
                d.set_response_context('HOLD');d.input(0.,0.);d.request_home()
                d._home_and_clear()
                self.assertEqual(d.phase,'HOME_HOLD')
                self.assertAlmostEqual(d.angles[0],0.,delta=1.)
                self.assertAlmostEqual(bus.physical[1],expected,delta=1.)
                self.assertTrue(d.armed)
                self.assertFalse(any(p[2] in (8,9,10) for p in bus.writes))
                d._disable_now()

    def test_home_rebases_multiple_turns_without_sending_a_calibration_or_move(self):
        d,bus=self.driver((170.,0.),continuous_yaw=True)
        # Equivalent software history after more than a complete revolution.
        d.angles=d.manual_target=d.sent_target=d.stream_target=(550.,0.)
        d.mechanical=(550.,0.)
        stamp=self.clock.now
        d.feedback_cache=[(550.,stamp),(0.,stamp)]
        d.mechanical_samples=[(stamp,550.),(stamp,0.)]
        d.position_intent=[550.,0.]
        before=len(bus.writes)
        d._rebase_yaw_for_home()
        self.assertEqual(d.angles,(-170.,0.))
        self.assertEqual(d.sent_target,(-170.,0.))
        self.assertEqual(d.feedback_cache[0],(-170.,stamp))
        self.assertEqual(d.mechanical_samples[0],(stamp,-170.))
        self.assertEqual(len(bus.writes),before)

    def test_heading_request_chooses_nearest_equivalent_turn_and_pitch_stays_limited(self):
        d,bus=self.driver((175.,0.),continuous_yaw=True)
        d.set_response_context('HOLD')
        d.request_angle_move(-175.,0.,30.)
        self.assertEqual(d.angle_pending[0],185.)
        d.angle_pending=None
        with self.assertRaisesRegex(ValueError,'限位'):
            d.request_angle_move(200.,31.,30.)

    def test_json_contains_no_infinite_limits_and_wire_encoding_remains_single_turn(self):
        d,bus=self.driver((170.,0.),continuous_yaw=True)
        m.validate_config(d.c)
        state=d.snapshot()
        self.assertEqual(state['limits'],((None,None),(-100.,30.)))
        self.assertTrue(state['continuous_yaw'])
        json.dumps(state,allow_nan=False)
        hub=m.TrackingHub(d,dict(m.CONFIG),self.clock.monotonic)
        hub.offer_frame([app.track()],640,480,self.clock.now,d.angles)
        json.dumps(hub.telemetry(),allow_nan=False)
        packet=m.set_single_turn_position(1,190.)
        self.assertEqual(int.from_bytes(packet[3:5],'big'),1900)
        self.assertEqual(m.decoded_position(d.c,0,1900,181.),190.)
        self.assertEqual(m.decoded_position(d.c,0,1700,-181.),-190.)


if __name__=='__main__':unittest.main()
