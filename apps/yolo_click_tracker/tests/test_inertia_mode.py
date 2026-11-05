"""Temporary inertia experiment bypasses heuristics, retains usable angle gate."""
import unittest
from unittest.mock import patch
import test_angle_move as angle

m=angle.m


class InertiaModeTests(unittest.TestCase):
    setUp=angle.AngleMoveTests.setUp
    make_driver=angle.AngleMoveTests.make_driver
    enable=angle.AngleMoveTests.enable
    setup_hub=angle.AngleMoveTests.setup_hub
    advance=angle.AngleMoveTests.advance
    send=angle.AngleMoveTests.send

    def test_large_pitch_move_skips_heuristics_then_restores_them(self):
        d,bus,h,token=self.setup_hub()
        self.send(h,token,action='inertia_start',pitch=-75.,speed=60.)
        self.assertTrue(d.inertia_test_active)
        with patch.object(d.guard,'update',side_effect=AssertionError('ordinary motion guard used')),patch.object(d.guard,'check_request',side_effect=AssertionError('request guard used')):
            for _ in range(300):
                self.advance(d,h)
                if d.angle_move.state=='COMPLETE':break
        self.assertEqual(d.angle_move.state,'COMPLETE')
        self.assertFalse(d.inertia_test_active)
        self.assertTrue(d.armed);self.assertAlmostEqual(bus.physical[2],-75.)
        self.assertEqual(d.programmed_rpm[1],10)
        self.assertEqual(d.hold_target,(0.,-75.))

    def test_cancel_disarm_and_fault_restore_normal_mode(self):
        for action in ('angle_cancel','disarm','fault'):
            with self.subTest(action=action):
                d,bus,h,token=self.setup_hub()
                self.send(h,token,action='inertia_start',pitch=-75.,speed=60.)
                if action=='angle_cancel':
                    self.send(h,token,2,action=action);self.advance(d,h)
                elif action=='disarm':d._disable_now()
                else:d._latch_fault('test fault')
                self.assertFalse(d.inertia_test_active)

    def test_current_angle_outside_work_range_still_disables(self):
        d,bus,h,token=self.setup_hub()
        self.send(h,token,action='inertia_start',pitch=-75.,speed=60.)
        bus.physical[2]=30.1
        with self.assertRaisesRegex(m.MotionProtectionFault,'INERTIA_ANGLE_LIMIT'):d._measure()
        self.assertFalse(d.armed or d.inertia_test_active)
        self.assertFalse(any(bus.enabled.values()))

    def test_vision_age_and_soft_heartbeat_cannot_interrupt_accepted_board_experiment(self):
        d,bus,h,token=self.setup_hub()
        self.send(h,token,action='inertia_start',pitch=-75.,speed=60.)
        self.clock.now+=3.
        h.tick()
        self.assertTrue(d.armed);self.assertIsNone(d.fault)
        d.command_time=self.clock.now-5.
        d._heartbeat_check()
        d.request_arm(False)
        with self.assertRaises(m.UserDisarm):d._heartbeat_check()

    def test_target_outside_range_is_never_accepted(self):
        d,bus,h,token=self.setup_hub()
        with self.assertRaises(m.ControlError):self.send(h,token,action='inertia_start',pitch=-101.,speed=60.)
        self.assertFalse(d.inertia_test_active);self.assertTrue(d.armed)
