"""Latched reference under external displacement; synthetic position-loop plant."""
import unittest
import test_tracker as app
import test_separate_controls as controls
import test_fast_manual as fast

m=app.m


class FixedHoldTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    make_driver=controls.SeparateControlTests.make_driver
    enable=controls.SeparateControlTests.enable
    move=fast.FastManualTests.move

    def tick(self,d):
        self.clock.sleep(.02);d._measure();d._advance(d.manual_target,d.c['max_rate'])

    def test_hand_displacement_does_not_move_reference_or_disable_after_old_1_8s_window(self):
        d,bus=self.make_driver((20.,-20.));self.enable(d)
        d.set_response_context('HOLD');anchor=d.hold_target
        before=len([p for p in bus.writes if p[2]==3])
        bus.physical={1:26.,2:-25.}  # External hand holds the displaced plant.
        for _ in range(150):self.tick(d)
        self.assertTrue(d.armed);self.assertIsNone(d.fault)
        self.assertEqual(d.hold_target,anchor)
        self.assertEqual(d.sent_target,anchor)
        self.assertEqual(d.manual_target,anchor)
        self.assertEqual(len([p for p in bus.writes if p[2]==3]),before)
        # On hand release, a synthetic native position loop follows its still-
        # latched target. This does not measure real available motor torque.
        for _ in range(40):
            for mid in (1,2):
                goal=m.signed_mechanical(bus.target[mid])
                bus.physical[mid]+=m.clip(goal-bus.physical[mid],-.4,.4)
            self.tick(d)
        self.assertEqual(tuple(bus.physical.values()),anchor)
        self.assertEqual(d.hold_target,anchor)

    def test_short_feedback_pause_does_not_rebase_hold_to_the_displaced_angle(self):
        d,bus=self.make_driver((0.,0.));self.enable(d);d.set_response_context('HOLD')
        bus.physical[1]=4.;bus.drop_feedback={2}
        self.tick(d);self.assertTrue(d.motion_paused)
        bus.drop_feedback.clear();self.tick(d)
        self.assertFalse(d.motion_paused)
        self.assertEqual(d.hold_target,(0.,0.))
        self.assertEqual(d.manual_target,(0.,0.))
        self.assertEqual(d.sent_target,(0.,0.))

    def test_reported_14_5_degree_pitch_displacement_is_reconfirmed_without_disarm(self):
        d,bus=self.make_driver((0.,0.));self.enable(d);d.set_response_context('HOLD')
        bus.physical[2]=14.5
        self.tick(d)
        self.assertTrue(d.feedback_issues[1])  # One large sample is not trusted.
        self.tick(d)
        self.assertFalse(any(d.feedback_issues))
        self.assertEqual(d.angles,(0.,14.5))
        for _ in range(120):self.tick(d)
        self.assertTrue(d.armed);self.assertIsNone(d.fault)
        self.assertEqual(d.hold_target,(0.,0.));self.assertEqual(d.sent_target,(0.,0.))

    def test_one_large_bad_sample_cannot_replace_the_hold_reference_or_feedback(self):
        d,bus=self.make_driver((0.,0.));self.enable(d);d.set_response_context('HOLD')
        bus.physical[2]=25.;self.tick(d)
        self.assertEqual(d.angles[1],0.)
        bus.physical[2]=0.;self.tick(d)
        self.assertFalse(any(d.feedback_issues));self.assertEqual(d.hold_target,(0.,0.))

    def test_home_zero_overshoot_inside_working_range_is_not_a_path_escape(self):
        d,bus=self.make_driver((20.,15.));self.enable(d)
        d.request_home();d._begin_home();d.phase='RETURNING'
        d.sent_target=d.stream_target=(0.,0.)
        d.position_intent=[0.,0.];bus.physical={1:0.,2:2.}
        d._measure()
        bus.physical[2]=-10.3
        self.clock.sleep(.05);d._measure()
        self.clock.sleep(.05);d._measure()
        self.assertTrue(d.armed);self.assertIsNone(d.fault)
        self.assertIsNone(d.motion_warning)
        self.assertEqual(d.angles[1],-10.3)

    def test_sustained_no_return_progress_still_faults_and_disables(self):
        d,bus=self.make_driver((0.,0.));self.enable(d);d.set_response_context('HOLD')
        bus.physical[1]=4.
        with self.assertRaisesRegex(RuntimeError,'fixed hold') as caught:
            for _ in range(430):self.tick(d)
        d._record_fault(caught.exception)
        self.assertFalse(d.armed or d.arm_requested)
        self.assertFalse(any(bus.enabled.values()))
        self.assertIsNone(d.hold_target)

    def test_small_2_3_degree_firmware_residual_does_not_fault_or_rebase_hold(self):
        d,bus=self.make_driver((0.,0.));self.enable(d);d.set_response_context('HOLD')
        anchor=d.hold_target;bus.physical[1]=2.3
        for _ in range(500):self.tick(d)
        self.assertTrue(d.armed);self.assertIsNone(d.fault)
        self.assertEqual(d.hold_target,anchor);self.assertEqual(d.sent_target,anchor)
        self.assertIsNone(d.hold_error_since[0])

    def test_small_residual_resets_an_existing_blockage_timer(self):
        d,bus=self.make_driver((0.,0.));self.enable(d);d.set_response_context('HOLD')
        bus.physical[1]=4.
        for _ in range(100):self.tick(d)
        self.assertIsNotNone(d.hold_error_since[0])
        bus.physical[1]=2.3
        for _ in range(500):self.tick(d)
        self.assertTrue(d.armed);self.assertIsNone(d.fault)
        self.assertIsNone(d.hold_error_since[0]);self.assertEqual(d.hold_target,(0.,0.))

    def test_hold_does_not_exempt_working_limit_violations(self):
        d,bus=self.make_driver((0.,28.));self.enable(d);d.set_response_context('HOLD')
        bus.physical[2]=31.
        with self.assertRaises(m.MotionProtectionFault):
            for _ in range(25):self.tick(d)
        self.assertFalse(d.armed)

    def test_manual_motion_releases_old_anchor_and_home_latches_zero(self):
        d,bus=self.make_driver((0.,0.));self.enable(d)
        old=d.hold_target
        self.move(d,(-.2,0.),count=15)
        self.assertIsNone(d.hold_target);self.assertNotEqual(d.sent_target,old)
        self.clock.pump=lambda:d.input(0.,0.);d.input(0.,0.)
        d.request_home();d._home_and_clear()
        self.assertEqual(d.hold_target,(0.,0.));self.assertTrue(d._fixed_hold_active())
