import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detector import Detector, detect
from adapter import observations


def frame(i, targets=((30, 40),), stamp=None):
    p = [[30]*160 for _ in range(120)]
    for x, y in targets:
        p[y][x] = 150
    return dict(frame_id=i, pixels=p, received_at=i*.1 if stamp is None else stamp,
                lo=100, hi=400, valid_temperature=True)


class DetectorTests(unittest.TestCase):
    def test_point_repeat_and_confirmation(self):
        d = Detector()
        a = d.update(frame(0), 0)
        self.assertTrue(any(c['area'] == 1 for c in a['candidates']))
        self.assertEqual(d.update(frame(0), .01)['status'], 'DUPLICATE')
        self.assertEqual(d.tracks[0]['hit_count'], 1)
        d.update(frame(1), .1)
        d.update(frame(2), .2)
        self.assertEqual(d.tracks[0]['track_state'], 'CONFIRMED')

    def test_invalid_stale_future_and_dropout(self):
        d = Detector()
        self.assertEqual(d.update(frame(0), 1)['status'], 'STALE')
        self.assertEqual(d.update(frame(2), 0)['status'], 'FUTURE')
        f = frame(0); f['valid_temperature'] = False
        self.assertEqual(d.update(f, 0)['status'], 'INVALID_TEMPERATURE')
        d.update(frame(0), 0)
        r = d.update(None, .2)
        self.assertFalse(r['valid'])
        self.assertFalse(r['tracks'][0]['detected_this_frame'])
        self.assertEqual(d.update(None, 1)['tracks'], [])

    def test_hold_and_reacquire(self):
        d = Detector()
        for i in range(3): d.update(frame(i), i*.1)
        ident = d.tracks[0]['track_id']
        lost = d.update(frame(3, ()), .3)
        self.assertTrue(lost['tracks'][0]['predicted'])
        self.assertEqual(lost['tracks'][0]['hit_count'], 3)
        got = d.update(frame(4), .4)
        self.assertEqual(got['tracks'][0]['track_id'], ident)
        self.assertTrue(got['tracks'][0]['detected_this_frame'])

    def test_ambiguity_and_calibration(self):
        d = Detector({'gate_px': 12})
        d.update(frame(0, ((30, 40), (36, 40))), 0)
        old = [t['hit_count'] for t in d.tracks]
        d.update(frame(1, ((33, 40),)), .1)
        self.assertEqual([t['hit_count'] for t in d.tracks[:2]], old)
        r = Detector().update(frame(0), 0)
        self.assertEqual(observations(r, None, (320, 240), 5), [])
        bad = dict(verified=True, raw_unflipped=True, thermal_size=[160,120],
                   resolution=[320,240], fixed_camera=True, distance_range_m=[4,6],
                   thermal_to_rgb=[[0,0,2],[0,0,2],[0,0,1]])
        for i in range(1,3): r = d.update(frame(i, ((30,40),)), i*.1)
        self.assertEqual(observations(r, bad, (320,240), 5), [])

    def test_valid_adapter_and_motion_suppression(self):
        d = Detector()
        r = None
        for i in range(3): r = d.update(frame(i), i*.1)
        cal = dict(verified=True, raw_unflipped=True, thermal_size=[160,120],
                   resolution=[320,240], fixed_camera=True, distance_range_m=[4,6],
                   thermal_to_rgb=[[1/159,0,0],[0,1/119,0],[0,0,1]])
        obs = observations(r, cal, (320,240), 5)
        self.assertEqual(len(obs), 1)
        self.assertEqual(obs[0]['kind'], 'thermal')
        self.assertEqual(observations(r, cal, (640,480), 5), [])
        self.assertEqual(observations(r, cal, (320,240), 10), [])
        moved = frame(3)
        for y in range(20,100):
            for x in range(20,140): moved['pixels'][y][x] += 25
        r = d.update(moved, .3)
        self.assertEqual(r['status'], 'MOTION_UNCERTAIN')
        self.assertEqual(observations(r, cal, (320,240), 5), [])

    def test_global_gain(self):
        d = Detector()
        a = frame(0); d.update(a, 0)
        b = frame(1); b['pixels'] = [[v+10 for v in row] for row in b['pixels']]
        self.assertEqual(d.update(b, .1)['motion_quality'], 1)

if __name__ == '__main__': unittest.main()
