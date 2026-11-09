"""Synthetic integration checks; no claims about device accuracy/performance."""
import copy
import json
import runpy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import types
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
m = types.SimpleNamespace(**runpy.run_path(str(ROOT / 'main.py')))
a = m.thermal_api


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.c = copy.deepcopy(m.DEFAULT)
        self.c['alignment_path'] = ''
        self.alignment = m.make_alignment(self.c, (640, 480))
        self.h = np.array([[2.4, .18, 85], [-.12, 2.5, 65], [.0004, -.0003, 1.]])
        self.fit = np.array([[12,24], [65,14], [139,20], [145,69],
                             [130,106], [78,104], [18,101], [15,60]], np.float64)
        self.check = np.array([[48,46], [104,81], [85,60]], np.float64)

    def project(self, pts, h=None):
        return cv2.perspectiveTransform(np.asarray(pts, np.float64).reshape(-1,1,2),
                                       self.h if h is None else h).reshape(-1,2)

    def calibrated(self):
        h, stats = a['fit_registration'](self.fit, self.project(self.fit), (640,480))
        stats.update(a['check_registration'](h, self.check, self.project(self.check), (640,480), stats))
        self.alignment.set_registration(h, stats)
        return self.alignment

    def frame(self, i, point=True):
        pixels = np.full((120,160), 30, np.uint8)
        if point:
            pixels[46,48] = 150
        return dict(pixels=pixels, received_at=i*.1, lo=100, hi=400, valid_temperature=True)

    def test_canvas_normalization_letterbox_and_flips(self):
        self.c['thermal_alignment'] = dict(width=800, height=500)
        for size in ((640,480), (1280,720)):
            al = m.make_alignment(self.c, size)
            matrix = m.normalized_overlay_matrix(al)
            p = self.project([[0,0], [79,59], [159,119]], matrix)
            expected = self.project([[0,0], [79,59], [159,119]], al.matrix)
            x,y,_,_,sx,sy = al.viewport
            expected = (expected-[x+(sx-1)/2, y+(sy-1)/2])/[sx,sy]/size
            np.testing.assert_allclose(p, expected)

    def test_checked_mapping_not_flipped_twice(self):
        al = self.calibrated()
        cal, reason = m.fusion_calibration(al, True)
        self.assertTrue(cal['verified'])
        np.testing.assert_allclose(self.project(self.check, np.array(cal['thermal_to_rgb'])),
                                   self.project(self.check)/[640,480], atol=1e-5)
        self.assertEqual(cal['distance_range_m'], [1.,1.])

    def test_manual_only_and_moving_camera_never_fuse(self):
        integration = m.ThermalIntegration(self.c, self.alignment)
        for i in range(3):
            result, obs, _ = integration.update(self.frame(i), i, i*.1)
        self.assertTrue(result['candidates'])
        self.assertEqual(obs, [])
        self.calibrated()
        self.c['fixed_camera'] = False
        self.assertFalse(m.fusion_calibration(self.alignment, False)[0]['verified'])

    def test_detector_confirm_duplicate_stale_and_hold(self):
        integration = m.ThermalIntegration(self.c, self.calibrated())
        for i in range(3):
            result, obs, _ = integration.update(self.frame(i), i, i*.1)
            self.assertEqual(len(obs), int(i == 2))
        self.assertEqual(result['tracks'][0]['hit_count'], 3)
        for frame, count, now, status in ((self.frame(2),2,.21,'DUPLICATE'),
                (self.frame(2),2,1.,'STALE'), (None,3,1.1,'NO_FRAME')):
            result, obs, _ = integration.update(frame,count,now)
            self.assertEqual(result['status'], status)
            self.assertEqual(obs, [])
        # A confirmed held track must not generate fresh evidence.
        integration = m.ThermalIntegration(self.c, self.calibrated())
        for i in range(3): integration.update(self.frame(i),i,i*.1)
        result, obs, _ = integration.update(self.frame(3,False),3,.3)
        self.assertEqual(result['tracks'][0]['track_state'], 'HOLD')
        self.assertEqual(obs, [])

    def test_thermal_only_cannot_confirm_uav(self):
        integration = m.ThermalIntegration(self.c, self.calibrated())
        engine = m.Fusion(self.c)
        for i in range(4):
            _, obs, _ = integration.update(self.frame(i),i,i*.1)
            tracks, _ = engine.update(obs,i*.1)
        self.assertEqual(tracks[0]['state'], 'CANDIDATE')
        self.assertLessEqual(tracks[0]['score'], .15)

    def test_three_sensor_fusion_uses_same_position(self):
        integration = m.ThermalIntegration(self.c, self.calibrated())
        engine = m.Fusion(self.c)
        x,y = self.project([[48,46]])[0]/[640,480]
        for i in range(4):
            _, obs, _ = integration.update(self.frame(i),i,i*.1)
            obs += [m.observation('vision',x,y,.95,i*.1,i,'drone'),
                    m.observation('sound',x,y,.8,i*.1,i)]
            tracks, _ = engine.update(obs,i*.1)
        self.assertEqual(len(tracks),1)
        self.assertEqual(tracks[0]['state'],'VISUAL_CONFIRMED')
        self.assertGreater(tracks[0]['contributions']['thermal'],0)

    def test_calibration_pause_emits_no_observations(self):
        integration = m.ThermalIntegration(self.c, self.calibrated())
        for i in range(4):
            _, obs, reason = integration.update(self.frame(i),i,i*.1,paused=True)
            self.assertEqual(obs,[])
        self.assertIn('PAUSED',reason)

    def test_manual_tuning_rechecked_and_reset(self):
        al = self.calibrated()
        key = m.ThermalIntegration(self.c,al).geometry_key()
        for _ in range(10): al.adjust('RIGHT')
        self.assertNotEqual(key,m.ThermalIntegration(self.c,al).geometry_key())
        self.assertFalse(m.fusion_calibration(al,True)[0]['verified'])
        al.adjust('RESET')
        self.assertTrue(m.fusion_calibration(al,True)[0]['verified'])
        al.next_distance()
        self.assertFalse(m.fusion_calibration(al,True)[0]['verified'])

    def test_old_schema4_save_reload_and_mode_rejection(self):
        al = self.calibrated()
        with tempfile.TemporaryDirectory(dir=ROOT/'validation') as td:
            al.path = str(Path(td)/'alignment.json')
            al.save()
            self.c['alignment_path'] = al.path
            restored = m.make_alignment(self.c,(640,480))
            self.assertTrue(m.fusion_calibration(restored,True)[0]['verified'])
            self.c['camera_fps'] = 30
            self.assertIsNone(m.make_alignment(self.c,(640,480)).homography)
            self.c['camera_fps'] = 60
            self.assertIsNone(m.make_alignment(self.c,(320,240)).homography)

    def test_overlay_roi_and_observation_match_raw_hot_pixel(self):
        al = self.calibrated()
        frame = self.frame(0)
        p = self.project([[48,46]], al.matrix)[0]
        stats = a['roi_temperature'](frame, (*list(p-.1), .2,.2), al)
        self.assertEqual(stats['samples'],1)
        self.assertAlmostEqual(stats['max'],10+30*150/254,places=5)
        canvas,_,_ = a['compose'](np.zeros((480,640,3),np.uint8),[],frame,al,0,0)
        self.assertTrue(canvas[round(p[1]),round(p[0])].any())

    def test_stale_overlay_hidden(self):
        al = self.calibrated()
        rgb = np.full((480,640,3),20,np.uint8)
        canvas,_,status = a['compose'](rgb,[],self.frame(0),al,2,2)
        self.assertEqual(status,'THERMAL STALE')
        np.testing.assert_array_equal(canvas[130:400,100:500],rgb[130:400,100:500])

    def test_candidate_display_duplicate_cache_expires_and_desync_hides(self):
        integration = m.ThermalIntegration(self.c, self.calibrated())
        integration.update(self.frame(0),0,0)
        integration.update(self.frame(0),0,.01)
        canvas = np.zeros((480,640,3),np.uint8)
        integration.draw(canvas,.01,0)
        self.assertTrue(canvas.any())
        for now, captured in ((1.,1.), (.4,.4)):
            canvas[:] = 0
            integration.draw(canvas,now,captured)
            self.assertFalse(canvas.any())

    def test_board_snapshot_and_cleanup_on_display_failure(self):
        module = runpy.run_path(str(ROOT/'main.py'))
        g = module['run_board'].__globals__
        events=[]
        class Resource:
            def __init__(self,*args,**kwargs): pass
            def close(self): events.append('close:'+type(self).__name__)
        class Reader(Resource):
            def start(self): events.append('start')
            def snapshot(self): events.append('snapshot'); return None,'WAIT',0
        class Sensor(Resource):
            def open_uart(self): pass
            def poll(self,**kwargs): pass
            def snapshot(self): return dict(valid=False,reason='WAIT')
        class Frame:
            def format(self): return 'RGB'
        class Camera(Resource):
            def read(self,**kwargs): return Frame()
        class Display:
            def show(self,*args,**kwargs): raise RuntimeError('display test failure')
        class Model:
            labels=['drone']
            def __init__(self,**kwargs): pass
            def input_width(self): return 640
            def input_height(self): return 480
            def detect(self,*args,**kwargs): return []
        fake=types.SimpleNamespace(app=types.SimpleNamespace(need_exit=lambda:False),
            camera=types.SimpleNamespace(Camera=Camera),display=types.SimpleNamespace(Display=Display),
            nn=types.SimpleNamespace(YOLO11=Model),
            touchscreen=types.SimpleNamespace(TouchScreen=lambda:None),
            image=types.SimpleNamespace(Format=types.SimpleNamespace(FMT_RGB888='RGB'),
                Fit=types.SimpleNamespace(FIT_CONTAIN=0),
                image2cv=lambda *args,**kw:np.zeros((480,640,3),np.uint8),
                cv2image=lambda x,**kw:x))
        g['thermal_api']['ThermalReader']=Reader
        g['sound_api']['SoundDirectionSensor']=Sensor
        with patch.dict(sys.modules,maix=fake), tempfile.TemporaryDirectory() as td:
            with self.assertRaisesRegex(RuntimeError,'display test failure'):
                module['run_board'](str(Path(td)/'absent.json'))
        self.assertEqual(events.count('snapshot'),1)
        self.assertEqual(events.count('start'),1)
        self.assertEqual(set(e for e in events if e.startswith('close:')),
                         {'close:Reader','close:Sensor','close:Camera'})


if __name__ == '__main__':
    unittest.main()
