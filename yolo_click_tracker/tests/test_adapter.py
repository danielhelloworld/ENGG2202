"""Real generated motor service driven by a physical-position MODEL; no UART."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import test_tracker as app_tests

m = app_tests.m
path = Path(__file__).resolve().parents[2] / "motor_joystick/tests/test_demo.py"
spec = importlib.util.spec_from_file_location("motor_fixtures", path)
fixtures = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixtures)


class MotorBridgeTests(unittest.TestCase):
    def test_click_track_c_clear_c_home_holds_with_actual_generated_motor_worker(self):
        clock = fixtures.Clock()
        bus = fixtures.MechanicalMotorModel((0, 0))
        driver = m.VisualMotorService(dict(m.CONFIG), lambda: bus)
        driver.stop_event = fixtures.Event(clock)
        hub = m.TrackingHub(driver, dict(m.CONFIG), clock.monotonic)
        stage, steps, last_frame, held, tracked = [0], [0], [0.], [], []

        def camera_loop():
            if clock.now-last_frame[0] >= .08:
                hub.offer_frame([app_tests.track()], 640, 480, clock.now)
                last_frame[0] = clock.now
            hub.tick()

        def schedule():
            steps[0] += 1
            if stage[0] == 0 and driver.phase == "HOME_HOLD" and driver.ready:
                stage[0] = 1
            elif stage[0] == 1 and driver.phase == "HOME_HOLD":
                hub.local_action(click=(hub.latest["id"], .625, .375))
                stage[0] = 2
            elif stage[0] == 2 and steps[0] > 45:
                tracked.append(tuple(bus.physical.values()))
                hub.local_action("c")
                stage[0] = 3
            elif stage[0] == 3 and steps[0] > 55:
                hub.local_action("c")
                stage[0] = 4
            elif stage[0] == 4 and driver.phase == "HOME_HOLD":
                held.append(driver.snapshot())
                self.assertTrue(all(bus.enabled.values()))
                driver.stop_event.set()
            if driver.phase == "FAULT" or steps[0] > 1200:
                driver.stop_event.set()

        clock.pump = camera_loop
        driver.stop_event.callback = schedule
        with patch.object(m.time, "monotonic", clock.monotonic), patch.object(m.time, "sleep", clock.sleep), contextlib.redirect_stdout(io.StringIO()):
            driver._run()
        self.assertIsNone(driver.fault)
        self.assertTrue(tracked)
        self.assertLess(tracked[0][0], 0)  # Rightward target uses inverted yaw input.
        self.assertLess(tracked[0][1], 0)  # Upward image target now reverses pitch too.
        self.assertTrue(held)
        self.assertTrue(held[0]["armed"])
        self.assertTrue(all(abs(v) <= .5 for v in held[0]["angles"]))
        self.assertFalse(any(bus.enabled.values()))  # Shutdown always disables.
        self.assertFalse(any(d[2] in (2, 8, 9, 10) for d in bus.writes))
        self.assertTrue(all(d[3] in (0, 2) for d in bus.writes if d[2] == 0x0E))


class NativeVisionTests(unittest.TestCase):
    def test_per_class_id_keys_and_lost_tracks_are_filtered(self):
        objects = [types.SimpleNamespace(class_id=i, x=10, y=20, w=30, h=40, score=.9) for i in (0, 1)]
        settings = {}
        class Model:
            labels = ["person", "bicycle"]
            def __init__(self, **kwargs):
                settings.update(kwargs)
            def detect(self, *args, **kwargs):
                return objects
        class Tracker:
            def __init__(self, *args):
                pass
            def update(self, objs):
                if not objs:
                    return []
                return [types.SimpleNamespace(id=1, lost=False, history=[objs[0]], score=.9),
                        types.SimpleNamespace(id=2, lost=True, history=[objs[0]], score=.8)]
        api = types.SimpleNamespace(ByteTracker=Tracker,
                                    Object=lambda x,y,w,h,c,s:types.SimpleNamespace(x=x,y=y,w=w,h=h))
        maix = types.SimpleNamespace(nn=types.SimpleNamespace(YOLO11=Model), tracker=api)
        with patch.dict(sys.modules, {"maix": maix}):
            vision = m.NativeVision(dict(m.CONFIG))
            tracks = vision.detect(types.SimpleNamespace(width=lambda:640,height=lambda:480))
        self.assertEqual({t["key"] for t in tracks}, {(0, 1), (1, 1)})
        self.assertEqual(len(tracks), 2)
        self.assertTrue(settings["dual_buff"])
        self.assertEqual(settings["model"], "/root/models/yolo11n.mud")


if __name__ == "__main__":
    unittest.main()
