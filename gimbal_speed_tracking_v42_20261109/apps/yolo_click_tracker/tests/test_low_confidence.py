"""Verify SDK input eligibility, not a reimplementation of native ByteTrack."""
import types
import unittest
from unittest.mock import patch
import test_tracker as app

m=app.m


class LowConfidenceTests(unittest.TestCase):
    def test_low_score_detection_reaches_tracker_while_new_track_threshold_stays_high(self):
        calls={}
        class Model:
            labels=['person']
            def __init__(self,**kwargs):pass
            def detect(self,img,conf_th,iou_th):
                calls['detect_threshold']=conf_th
                return [types.SimpleNamespace(class_id=0,x=10,y=10,w=20,h=20,score=.3)]
        class Tracker:
            def __init__(self,*args):calls['tracker_args']=args
            def update(self,objects):calls['objects']=objects;return []
        fake=types.SimpleNamespace(nn=types.SimpleNamespace(YOLO11=Model),
            tracker=types.SimpleNamespace(ByteTracker=Tracker,Object=lambda x,y,w,h,c,s:types.SimpleNamespace(score=s)))
        with patch.dict('sys.modules',{'maix':fake}):
            vision=m.NativeVision(m.CONFIG)
            vision.detect(types.SimpleNamespace(width=lambda:640,height=lambda:480))
        self.assertEqual(calls['detect_threshold'],.25)
        self.assertEqual(calls['objects'][0].score,.3)
        self.assertEqual(calls['tracker_args'][1:3],(.5,.5))


if __name__=='__main__':unittest.main()
