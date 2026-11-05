"""Image jitter, latency compensation and rate braking: synthetic evidence."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m = app.m


def target(error):
    t = app.track()
    t.update(cx=.5+error,cy=.5)
    return t


class StableAimTests(unittest.TestCase):
    def core(self):
        c=m.ClickTracker(dict(m.CONFIG,prediction_enabled=False,native_tracking=False))
        c.select((0,1),[target(0.)],1.)
        return c

    def test_center_box_jitter_does_not_restart_motor(self):
        c=self.core()
        for i,e in enumerate((0.,.01,.022,-.024,.02,-.01)):
            c.step([target(e)],1+i*.1)
            self.assertEqual(c.vector[0],0.)
        c.step([target(.05)],1.7)
        self.assertGreater(c.vector[0],0.)

    def test_capture_to_current_rotation_removes_already_corrected_error(self):
        c=self.core()
        # Right target .1 image width, negative yaw moved by exactly that angle.
        c.step([target(.1)],1.1,pose=(0.,0.),current_pose=(-8.7,0.))
        self.assertAlmostEqual(c.vector[0],0.)

    def test_current_pose_compensation_supports_opposite_motor_sign_mapping(self):
        c=m.ClickTracker(dict(m.CONFIG,prediction_enabled=False,joystick_signs=(1,1)))
        c.select((0,1),[target(.1)],1.)
        c.step([target(.1)],1.1,pose=(0.,0.),current_pose=(8.7,0.))
        self.assertAlmostEqual(c.vector[0],0.)

    def test_missing_current_pose_uses_original_image_error(self):
        c=self.core()
        c.step([target(.1)],1.1,pose=(0.,0.),current_pose=None)
        self.assertGreater(c.vector[0],0.)

    def test_approaching_center_reduces_rate_before_deadzone(self):
        far=self.core();near=self.core()
        for i in range(10):
            far.step([target(.2)],1+i*.1)
            near.step([target(.025)],1+i*.1)
        near_rate=abs(near.vector[0])*m.CONFIG["max_rate"][0]
        self.assertLess(near_rate,2.)
        self.assertLess(near_rate,abs(far.vector[0])*m.CONFIG["max_rate"][0])

    def test_delayed_feedback_plant_converges_without_repeated_center_crossings(self):
        c=self.core()
        angle,velocity=15.,0.
        history=[angle]*4
        angles=[]
        # 150ms delayed 20Hz image stream, first-order plant with acceleration limit.
        for i in range(200):
            delayed=history[-4]
            c.step([target(-delayed/87.)],1+i*.05,pose=(-delayed,0.),current_pose=(-angle,0.))
            desired=c.vector[0]*m.CONFIG["max_rate"][0]
            velocity+=m.clip((desired-velocity)*.05/.12,-180*.05,180*.05)
            angle+=velocity*.05
            history.append(angle);angles.append(angle)
        self.assertLess(abs(angles[-1]),3.)
        self.assertGreater(min(angles),-1.)


class NoSecondGainTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    make_driver=controls.SeparateControlTests.make_driver
    enable=controls.SeparateControlTests.enable

    def test_zero_tracking_request_does_not_receive_position_error_push(self):
        d,bus=self.make_driver((0.,0.));self.enable(d)
        d.set_response_context("TRACK")
        self.clock.pump=lambda:d.input(0.,0.)
        self.clock.sleep(.05)
        d.response.scale=(1.15,1.15);d.response.correction=(3.,-3.)
        d._advance(d.angles,d.c["max_rate"])
        self.assertEqual(d.stream_velocity,(0.,0.))
        self.assertEqual(d.stream_target,(0.,0.))


if __name__=="__main__":
    unittest.main()
