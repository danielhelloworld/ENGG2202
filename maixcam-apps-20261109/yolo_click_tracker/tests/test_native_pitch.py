"""Native T profile experiment stays UART-owned and restores direct mode."""
import unittest
import test_angle_move as angle

m=angle.m


class NativePitchTests(unittest.TestCase):
    setUp=angle.AngleMoveTests.setUp
    make_driver=angle.AngleMoveTests.make_driver
    enable=angle.AngleMoveTests.enable
    setup_hub=angle.AngleMoveTests.setup_hub
    advance=angle.AngleMoveTests.advance
    send=angle.AngleMoveTests.send

    def test_native_move_sends_one_goal_and_restores_mode_without_enable_toggles(self):
        d,bus,h,token=self.setup_hub();before=len(bus.writes)
        self.send(h,token,action='native_start',pitch=-10.,speed=30.)
        self.assertEqual(len(bus.writes),before)
        for _ in range(150):
            self.advance(d,h)
            if d.angle_move.state=='COMPLETE':break
        frames=bus.writes[before:]
        modes=[int.from_bytes(p[3:5],'big') for p in frames if p[2]==0]
        self.assertEqual(modes,[2,4])
        self.assertFalse(any(p[1]==1 and p[2]==0 for p in frames))
        self.assertFalse(any(p[2] in (5,6,8,9,10) for p in frames))
        self.assertLessEqual(len([p for p in frames if p[2]==3]),4)
        self.assertEqual(d.angle_move.profile,'native_t')
        self.assertEqual(d.angle_move.state,'COMPLETE');self.assertTrue(d.armed)
        self.assertFalse(d.inertia_test_active or d.native_pitch_engaged)
        self.assertEqual(d.hold_target,(0.,-10.))

    def test_cancel_restores_direct_mode_and_holds_current_pose(self):
        d,bus,h,token=self.setup_hub();bus.stuck=True
        self.send(h,token,action='native_start',pitch=-75.,speed=60.)
        self.advance(d,h,4);self.assertTrue(d.native_pitch_engaged)
        before=len(bus.writes);self.send(h,token,2,action='angle_cancel')
        self.assertEqual(len(bus.writes),before)
        self.advance(d,h)
        self.assertFalse(d.inertia_test_active or d.native_pitch_engaged)
        self.assertTrue(d.armed);self.assertEqual(d.sent_target,(0.,0.))

    def test_home_mailbox_restores_native_mode_on_worker_before_homing(self):
        d,bus,h,token=self.setup_hub();bus.stuck=True
        self.send(h,token,action='native_start',pitch=-75.,speed=60.);self.advance(d,h,4)
        before=len(bus.writes);self.send(h,token,2,action='home')
        self.assertEqual(len(bus.writes),before)
        d._begin_home()
        self.assertFalse(d.native_pitch_engaged or d.inertia_test_active)
        self.assertTrue(d.recovery_active)

    def test_angle_limit_fault_disables_then_restores_direct_mode(self):
        d,bus,h,token=self.setup_hub();bus.stuck=True
        self.send(h,token,action='native_start',pitch=-75.,speed=60.);self.advance(d,h,4)
        bus.physical[2]=30.1
        with self.assertRaises(m.MotionProtectionFault):d._measure()
        self.assertFalse(d.armed or d.inertia_test_active or d.native_pitch_engaged)
        self.assertFalse(any(bus.enabled.values()))
        modes=[int.from_bytes(p[3:5],'big') for p in bus.writes if p[2]==0]
        self.assertEqual(modes[-1],4)

    def test_native_mode_is_forbidden_without_the_explicit_pitch_experiment(self):
        d,bus,h,token=self.setup_hub()
        for mid in (1,2):
            with self.assertRaises(RuntimeError):d._write(m.select_mode(mid,2))
