"""Tracking moving-target transients versus actual stalls: offline only."""
import math
import unittest
from unittest.mock import patch
import test_tracker as app
import test_separate_controls as controls

m = app.m


class TrackingToleranceTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable

    def test_back_and_forth_physical_tracking_is_not_a_stall(self):
        g = m.TrackingMotionGuard(m.CONFIG)
        for i in range(160):
            t = 1+i*.05
            p = 8*math.sin(t*5)
            g.update_tracking(t,(p,0.),(p+8,0.),(0.,0.))
        self.assertIsNone(g.last_event)

    def test_alternating_targets_cannot_hide_a_physically_stuck_motor(self):
        g = m.TrackingMotionGuard(m.CONFIG)
        with self.assertRaisesRegex(RuntimeError,"no physical movement"):
            for i in range(60):
                g.update_tracking(1+i*.05,(0.,0.),(8. if i%2 else -8.,0.),(0.,0.))

    def test_centered_stationary_target_does_not_fault(self):
        g = m.TrackingMotionGuard(m.CONFIG)
        for i in range(150):
            g.update_tracking(1+i*.05,(0.,0.),(.4,0.),(0.,0.))
        self.assertIsNone(g.last_event)

    def test_five_degree_tracking_error_is_below_new_stall_threshold(self):
        g = m.TrackingMotionGuard(m.CONFIG)
        for i in range(150):
            g.update_tracking(1+i*.05,(0.,0.),(5.,0.),(0.,0.))
        self.assertIsNone(g.last_event)

    def test_eight_degree_inertia_retreat_is_observed_without_reverse_fault(self):
        g = m.TrackingMotionGuard(m.CONFIG)
        for i in range(45):
            p = -min(8.,i*.5)
            g.update_tracking(1+i*.05,(p,0.),(20.,0.),(30.,0.))
        # The 8-degree retreat has stopped; continuing no progress must still
        # fault later, but it is below the new 12-degree reverse threshold.
        self.assertIsNone(g.last_event)

    def test_short_inertia_reversal_then_progress_does_not_disable(self):
        g = m.TrackingMotionGuard(m.CONFIG)
        for i in range(50):
            t = i*.05
            p = -10*t if t < .4 else -4+20*(t-.4)
            g.update_tracking(1+t,(p,0.),(p+8,0.),(30.,0.))
        self.assertIsNone(g.last_event)

    def test_persistent_motion_opposite_stable_plan_still_faults(self):
        g = m.TrackingMotionGuard(m.CONFIG)
        with self.assertRaisesRegex(RuntimeError,"opposite planned"):
            for i in range(100):
                g.update_tracking(1+i*.05,(-i*.5,0.),(10.,0.),(30.,0.))

    def test_feedback_pause_does_not_count_toward_stall(self):
        g = m.TrackingMotionGuard(m.CONFIG)
        g.update_tracking(1.,(0.,0.),(8.,0.),(0.,0.))
        g.pause(1.5)
        g.resume(4.)
        g.update_tracking(5.29,(0.,0.),(8.,0.),(0.,0.))
        with self.assertRaisesRegex(RuntimeError,"no physical movement"):
            g.update_tracking(5.31,(0.,0.),(8.,0.),(0.,0.))

    def test_obsolete_tracking_step_replans_without_disabling(self):
        d,bus = self.make_driver((0.,0.))
        self.enable(d)
        d.phase = "MANUAL"
        d.set_response_context("TRACK")
        self.clock.pump = lambda:d.input(-1.,0.)
        d.input(-1.,0.)
        self.clock.sleep(.05)
        real_write = d._write
        def changed_feedback(packet):
            if packet[2] == 3:
                raise m.TrackingReplan("refresh local target")
            return real_write(packet)
        with patch.object(d,"_write",side_effect=changed_feedback):
            d._advance(d.angles,d.c["max_rate"])
        self.assertTrue(d.armed)
        self.assertIsNone(d.fault)
        self.assertEqual(d.stream_target,d.angles)
        self.assertEqual(d.stream_velocity,(0.,0.))

    def test_tracking_soft_replan_does_not_bypass_hard_work_limits(self):
        d,bus = self.make_driver((0.,29.9))
        self.enable(d)
        d.set_response_context("TRACK")
        before = len(bus.writes)
        with self.assertRaisesRegex(RuntimeError,"Unsafe single-turn step"):
            d._write(m.set_single_turn_position(2,30.1))
        self.assertEqual(len(bus.writes),before)

    def test_repeated_acceleration_and_reversal_with_continuous_motor_feedback(self):
        import test_speed_multipliers as speed
        import test_adapter as adapters
        bus = speed.ContinuousMotorModel(self.clock,(0.,0.))
        d = m.VisualMotorService(dict(m.CONFIG,startup_auto_home=False,native_tracking=False),lambda:bus)
        d.serial,d.stop_event = bus,adapters.fixtures.Event(self.clock)
        d.sent_target = d.manual_target = d._initialize()
        d.ready,d.phase = True,"DISABLED"
        self.clock.pump = lambda:d.input(.4 if int(self.clock.now/.4)%2 else -.4,0.)
        d.input(.4,0.)
        self.enable(d)
        d.set_response_context("TRACK")
        positions = []
        for _ in range(200):
            self.clock.sleep(.02)
            d._measure()
            d._advance(d.angles,d.c["max_rate"])
            positions.append(d.angles[0])
            self.assertTrue(d.armed)
            self.assertFalse(any(d.feedback_issues))
            self.assertLessEqual(d._effective_accel()[0],600.)
        self.assertGreater(max(positions)-min(positions),2.)
        self.assertIsNone(d.fault)


if __name__ == "__main__":
    unittest.main()
