"""No hardware: pitch recovery direction and larger-test trajectory constraints."""
import sys
from pathlib import Path
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pitch_serial_tune as pitch


class PitchTrajectoryTests(unittest.TestCase):
    def make_session(self,q):
        s=pitch.PitchSession.__new__(pitch.PitchSession)
        s.data={};s.phase='TEST';s.target=q;s.profile=None
        s.experiment_goal=q;s.planned_velocity=0.
        clock=[1.];actual=[q];frames=[]
        def angle():
            clock[0]+=.005
            return clock[0],actual[0]
        def write(frame):
            self.assertEqual(frame[1],2)
            frames.append(frame)
        s.angle=angle;s.write=write;s.feedback=lambda kind:0
        return s,clock,actual,frames

    def test_large_trials_are_ramped_inside_bounds_with_lagging_plant(self):
        s,clock,actual,frames=self.make_session(0.)
        for goal in (10.,0.,-10.,0.,15.,0.,-15.,0.):
            s.offset(goal)
            for _ in range(150):
                before=actual[0]
                s.advance_goal(.02,before)
                self.assertLessEqual(abs(s.target-before),4.051)
                actual[0]+=(s.target-before)*(.02/.08)
                self.assertGreaterEqual(s.target,-15.)
                self.assertLessEqual(s.target,15.)
                self.assertGreaterEqual(actual[0],-15.)
                self.assertLessEqual(actual[0],15.)
            self.assertAlmostEqual(actual[0],goal,delta=.1)
        self.assertTrue(frames)
        self.assertTrue(all(f[2]==3 for f in frames))

    def test_outside_recovery_cannot_command_outward_or_skip_to_zero(self):
        s,clock,actual,frames=self.make_session(100.)
        s.position(95.,recovery=True)
        for goal in (101.,0.,-1.):
            with self.assertRaises(RuntimeError):s.position(goal,recovery=True)
        actual[0]=29.
        with self.assertRaises(RuntimeError):s.position(31.)
        self.assertEqual(len(frames),1)

    def test_full_outside_home_only_moves_inward_without_pi_or_zero(self):
        s,clock,actual,frames=self.make_session(100.)
        def write(frame):
            self.assertEqual(frame[1],2);self.assertEqual(frame[2],3)
            frames.append(frame)
            actual[0]=pitch.signed(int.from_bytes(frame[3:5],'big')/10.)
        s.write=write
        with patch.object(pitch.time,'perf_counter',lambda:clock[0]),patch.object(
                pitch.time,'sleep',lambda dt:clock.__setitem__(0,clock[0]+dt)):
            s.home()
        targets=[pitch.signed(int.from_bytes(f[3:5],'big')/10.) for f in frames]
        self.assertTrue(all(0<=q<=100 for q in targets))
        self.assertTrue(all(b<=a+.051 for a,b in zip(targets,targets[1:])))
        self.assertAlmostEqual(actual[0],0.)


if __name__=='__main__':unittest.main()
