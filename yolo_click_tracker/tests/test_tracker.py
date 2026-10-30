"""PC synthetic tracks, control mailboxes and real loopback HTTP; no hardware."""
import importlib.util
import json
from pathlib import Path
import threading
import unittest
from urllib.request import Request, urlopen
from urllib.error import HTTPError

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("click_tracker_bundle", ROOT / "main.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def track(key=(0, 1), box=(320, 120, 160, 120)):
    x, y, w, h = box
    return dict(key=key, box=box, cx=(x+w/2)/640, cy=(y+h/2)/480,
                label="person", score=.9, trail=[])


class FakeDriver:
    def __init__(self):
        self.c = dict(m.CONFIG)
        self.arm_requested = self.armed = self.confirmed = self.home = False
        self.inputs = []
        self.fault = None
        self.recheck = False

    def input(self, x, y):
        self.inputs.append((x, y))

    def request_arm(self, enabled):
        if enabled and (not self.confirmed or self.fault):
            raise RuntimeError("Confirm zero before ARM")
        self.armed = self.arm_requested = enabled
        if not enabled:
            self.home = False

    def confirm_reference(self):
        self.confirmed = True

    def request_home(self):
        if not self.armed or self.home:
            raise RuntimeError("ARM first; home cannot repeat while active")
        self.home = True

    def request_recheck(self):
        if self.armed:
            raise RuntimeError("DISARM first")
        self.recheck = True

    def telemetry(self):
        return dict(schema=1, samples=[])

    def snapshot(self):
        return dict(ready=not self.fault and not self.recheck, fault=self.fault,
                    armed=self.armed, angles=(10, -5), phase="ARMED" if self.armed else "DISABLED",
                    zero_confirmed=self.confirmed, reference_valid=not self.fault,
                    home_requested=self.home, age=.01, can_recheck=not self.armed)


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.core = m.ClickTracker(m.CONFIG)

    def select(self):
        self.core.select((0, 1), [track()], 1)

    def test_detection_alone_does_not_choose_or_move(self):
        self.core.step([track()], 1)
        self.assertIsNone(self.core.selected)
        self.assertEqual(self.core.vector, (0, 0))

    def test_selected_target_drives_only_its_id(self):
        self.select()
        self.core.step([track((2, 7), (0, 0, 80, 80)), track()], 1.1)
        self.assertTrue(all(v > 0 for v in self.core.vector))

    def test_lost_same_id_recovers_before_two_seconds(self):
        self.select()
        self.assertIsNone(self.core.step([], 2.9))
        self.assertEqual(self.core.state, "LOST")
        self.assertEqual(self.core.vector, (0, 0))
        self.assertIsNone(self.core.step([track()], 2.95))
        self.assertEqual(self.core.state, "TRACK")

    def test_timeout_releases_and_homes_like_mainweb(self):
        self.select()
        self.assertIsNone(self.core.step([], 3.0))
        self.assertEqual(self.core.step([], 3.01), "home")
        self.assertIsNone(self.core.selected)

    def test_different_id_or_class_cannot_replace_lost_target(self):
        self.select()
        self.core.step([track((1, 1)), track((0, 2))], 1.1)
        self.assertEqual(self.core.state, "LOST")
        self.assertEqual(self.core.selected, (0, 1))

    def test_deadzone_stops_and_filter_is_reset_on_reselection(self):
        self.select()
        self.core.step([track()], 1.1)
        self.core.step([track(box=(300, 220, 40, 40))], 1.2)
        self.assertEqual(self.core.vector, (0, 0))
        self.select()
        self.assertEqual(self.core.filtered, [0, 0])

    def test_vector_and_rate_are_bounded(self):
        self.select()
        for i in range(20):
            self.core.step([track(box=(600, 0, 40, 40))], 1.1+i*.03)
            self.assertLessEqual(m.math.hypot(*self.core.vector), 1)
            for v, cap, manual in zip(self.core.vector, m.CONFIG["tracking_rate"], m.CONFIG["max_rate"]):
                self.assertLessEqual(abs(v)*manual, cap+1e-8)

    def test_screen_letterbox_mapping_rejects_padding(self):
        rect = m.contain_rect(640, 360)
        self.assertIsNone(m.screen_to_normalized(100, 50, rect))
        self.assertEqual(m.screen_to_normalized(rect[0]+rect[2]/2, rect[1]+rect[3]/2, rect), (.5, .5))


class HubTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.driver = FakeDriver()
        self.hub = m.TrackingHub(self.driver, dict(m.CONFIG), lambda: self.now)
        self.frame = self.hub.offer_frame([track()], 640, 480, self.now)
        self.token = self.hub.connect()["token"]
        self.seq = 0

    def send(self, action="", **extra):
        self.seq += 1
        return self.hub.command(dict(token=self.token, seq=self.seq, x=0, y=0, action=action, **extra))

    def arm(self):
        self.send("zero")
        self.send("arm")

    def select(self, frame=None):
        self.send("select", frame_id=(frame or self.frame)["id"], nx=.625, ny=.375)

    def test_detect_before_enable_and_manual_adjust_before_select(self):
        self.assertFalse(self.driver.armed)
        self.arm()
        self.seq += 1
        self.hub.command(dict(token=self.token, seq=self.seq, x=1, y=0, action=""))
        self.hub.tick()
        self.assertEqual(self.driver.inputs[-1], (m.CONFIG["manual_rate"][0]/m.CONFIG["max_rate"][0], 0))
        self.select()
        self.assertEqual(self.hub.tracking.selected, (0, 1))

    def test_no_click_to_track_before_arm(self):
        with self.assertRaises(m.ControlError):
            self.select()

    def test_old_display_click_does_not_select_new_object_at_same_pixel(self):
        self.arm()
        self.hub.offer_frame([track((0, 9))], 640, 480, self.now)
        with self.assertRaises(m.ControlError):
            self.select()
        self.assertIsNone(self.hub.tracking.selected)

    def test_delayed_click_can_select_original_id_at_its_new_location(self):
        self.arm()
        self.hub.offer_frame([track((0, 9)), track((0, 1), (0, 0, 80, 80))], 640, 480, self.now)
        self.select()
        self.assertEqual(self.hub.tracking.selected, (0, 1))

    def test_click_uses_smallest_overlapping_box(self):
        self.arm()
        frame = self.hub.offer_frame([track((0, 2), (0, 0, 640, 480)), track()], 640, 480, self.now)
        self.select(frame)
        self.assertEqual(self.hub.tracking.selected, (0, 1))

    def test_expired_click_is_rejected(self):
        self.arm()
        self.now += .46
        with self.assertRaises(m.ControlError):
            self.select()

    def test_first_c_clears_second_c_homes_without_disabling(self):
        self.arm()
        self.select()
        self.send("c")
        self.assertIsNone(self.hub.tracking.selected)
        self.assertFalse(self.driver.home)
        self.assertTrue(self.hub.state()["c_next_home"])
        self.send("c")
        self.assertTrue(self.driver.home)
        self.assertTrue(self.driver.armed)
        self.assertFalse(self.hub.state()["c_next_home"])

    def test_manual_input_cancels_lock_and_c_sequence(self):
        self.arm()
        self.select()
        self.send("c")
        self.seq += 1
        self.hub.command(dict(token=self.token, seq=self.seq, x=-1, y=0, action=""))
        self.assertFalse(self.hub.tracking.c_next_home)
        self.assertIsNone(self.hub.tracking.selected)

    def test_manual_move_is_not_queued_behind_home(self):
        self.arm()
        self.send("home")
        self.seq += 1
        self.hub.command(dict(token=self.token, seq=self.seq, x=1, y=0, action=""))
        self.assertEqual(self.hub.manual, (0, 0))

    def test_http_heartbeat_does_not_feed_motor_when_app_loop_is_stuck(self):
        self.arm()
        for _ in range(5):
            self.send()
        self.assertFalse(self.driver.inputs)
        self.hub.tick()
        self.assertEqual(self.driver.inputs[-1], (0, 0))

    def test_vision_delay_holds_then_persistent_loss_disarms(self):
        self.arm()
        self.now += .46
        self.send()
        self.hub.tick()
        self.assertTrue(self.driver.armed)
        self.assertEqual(self.driver.inputs[-1], (0., 0.))
        self.now += m.CONFIG["vision_disable_age_s"]
        self.hub.tick()
        self.assertFalse(self.driver.armed)

    def test_web_timeout_releases_keys_and_retains_onboard_tracking(self):
        self.arm()
        self.select()
        self.now += .61
        self.hub.tick()
        self.assertTrue(self.driver.armed)
        self.assertEqual(self.hub.tracking.selected, (0, 1))
        self.assertIsNone(self.hub.token)
        self.assertEqual(self.hub.manual, (0., 0.))

    def test_loss_timeout_homes_and_releases_target(self):
        self.arm()
        self.select()
        for i in range(1, 23):
            self.now = 1+i*.1
            self.send()
            self.hub.offer_frame([], 640, 480, self.now)
            self.hub.tick()
        self.assertTrue(self.driver.home)
        self.assertIsNone(self.hub.tracking.selected)
        self.assertTrue(self.driver.armed)

    def test_screen_and_web_share_target_and_have_one_control_owner(self):
        self.arm()
        with self.assertRaises(m.ControlError):
            self.hub.local_action(click=(self.frame["id"], .625, .375))
        self.hub.local_action("stop")
        self.assertIsNone(self.hub.token)
        self.hub.local_action("zero")
        self.hub.local_action("arm")
        self.hub.local_action(click=(self.frame["id"], .625, .375))
        self.assertEqual(self.hub.state()["selected"], (0, 1))
        self.assertEqual(self.hub.state()["control_owner"], "LOCAL")

    def test_jpeg_frame_id_is_bound_to_exact_published_frame(self):
        self.hub.publish_media(self.frame, b"jpeg-one")
        self.hub.offer_frame([track((0, 2))], 640, 480, self.now)
        self.assertEqual(self.hub.media()["id"], self.frame["id"])
        self.assertEqual(self.hub.media()["jpeg"], b"jpeg-one")
        self.now += .5
        self.assertIsNone(self.hub.media())

    def test_frame_history_is_bounded(self):
        for _ in range(40):
            self.hub.offer_frame([track()], 640, 480, self.now)
        self.assertEqual(len(self.hub.frames), m.CONFIG["frame_history"])

    def test_background_preserves_target_and_rejects_old_queued_keys(self):
        self.arm()
        self.select()
        self.hub.offer_frame([track()], 640, 480, self.now)
        old = self.token
        self.hub.background(old)
        self.assertEqual(self.hub.state()["control_owner"], "LOCAL")
        self.assertTrue(self.driver.armed)
        self.assertEqual(self.hub.tracking.selected, (0, 1))
        with self.assertRaises(m.ControlError):
            self.hub.command(dict(token=old, seq=self.seq+1, x=1, y=0, action=""))
        # Browser timer suspension must not expire a revoked browser lease.
        self.now += .7
        self.hub.offer_frame([track()], 640, 480, self.now)
        self.hub.tick()
        self.assertTrue(self.driver.armed)
        self.assertTrue(any(self.driver.inputs[-1]))
        new = self.hub.connect()["token"]
        self.assertNotEqual(new, old)
        self.assertEqual(self.hub.tracking.selected, (0, 1))

    def test_background_releases_manual_input_without_disabling(self):
        self.arm()
        self.seq += 1
        self.hub.command(dict(token=self.token, seq=self.seq, x=1, y=0, action=""))
        self.hub.background(self.token)
        self.hub.tick()
        self.assertEqual(self.driver.inputs[-1], (0., 0.))
        self.assertTrue(self.driver.armed)

    def test_background_does_not_remove_visual_watchdog(self):
        self.arm()
        self.hub.background(self.token)
        self.now += m.CONFIG["vision_disable_age_s"]+.01
        self.hub.tick()
        self.assertFalse(self.driver.armed)

    def test_viewer_or_old_token_cannot_background_another_session(self):
        for token in (None, "wrong"):
            with self.assertRaises(m.ControlError):
                self.hub.background(token)
        self.assertEqual(self.hub.token, self.token)

    def test_explicit_stop_survives_background_handoff_race(self):
        self.arm()
        self.hub.background(self.token)
        self.hub.stop(self.token)
        self.assertFalse(self.driver.armed)

    def test_detached_session_cannot_stop_a_new_web_controller(self):
        self.arm()
        old = self.token
        self.hub.background(old)
        new = self.hub.connect()["token"]
        with self.assertRaises(m.ControlError):
            self.hub.stop(old)
        self.assertEqual(self.hub.token, new)
        self.assertTrue(self.driver.armed)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.driver = FakeDriver()
        self.hub = m.TrackingHub(self.driver, dict(m.CONFIG))
        self.hub.offer_frame([track()], 640, 480, m.time.monotonic())
        self.hub.publish_media(self.hub.latest, b"\xff\xd8jpeg\xff\xd9")
        self.server = m.make_http_server(self.hub, ("127.0.0.1", 0))
        self.url = "http://127.0.0.1:%d" % self.server.server_port
        self.worker = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": .02})
        self.worker.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.worker.join(1)

    def request(self, path, data=None, origin=None):
        return urlopen(Request(self.url+path, data=None if data is None else json.dumps(data).encode(),
                       headers={"Origin": origin or self.url, "Content-Type": "application/json"}), timeout=2)

    def test_page_and_actual_jpeg_bytes_have_frame_id(self):
        with self.request("/") as r:
            self.assertIn(b'id="video"', r.read())
        with self.request("/api/frame") as r:
            self.assertEqual(r.headers["Content-Type"], "image/jpeg")
            self.assertEqual(r.headers["X-Frame-Id"], "1")
            self.assertTrue(r.read().startswith(b"\xff\xd8"))

    def test_readonly_telemetry_endpoint_does_not_enable_or_move_motors(self):
        with self.request("/api/telemetry") as r:
            data = json.load(r)
        self.assertEqual(data["schema"], 1)
        self.assertEqual(data["samples"], [])
        self.assertFalse(self.driver.armed or self.driver.arm_requested)
        self.assertEqual(self.driver.inputs, [])

    def test_end_to_end_select_clear_and_home(self):
        with self.request("/api/connect", {}) as r:
            token = json.load(r)["token"]
        for seq, action in enumerate(("zero", "arm", "select", "c", "c"), 1):
            with self.request("/api/control", dict(token=token, seq=seq, x=0, y=0, action=action,
                                                   frame_id=1, nx=.625, ny=.375)) as r:
                state = json.load(r)
        self.assertTrue(self.driver.home)
        self.assertIsNone(state["selected"])
        self.assertTrue(self.driver.armed)

    def test_browser_can_take_over_completed_startup_hold_without_disabling(self):
        self.driver.armed = self.driver.arm_requested = self.driver.confirmed = True
        original = self.driver.snapshot
        self.driver.snapshot = lambda:dict(original(), phase="HOME_HOLD")
        with self.request("/api/connect", {}) as r:
            value = json.load(r)
        self.assertTrue(value["token"])
        self.assertTrue(value["state"]["armed"])
        self.assertEqual(value["state"]["control_owner"], "WEB")
        self.assertTrue(self.driver.arm_requested)

    def test_background_endpoint_revokes_only_its_session_and_preserves_home(self):
        with self.request("/api/connect", {}) as r:
            token = json.load(r)["token"]
        self.driver.confirmed = self.driver.armed = self.driver.arm_requested = self.driver.home = True
        with self.request("/api/background", dict(token=token)) as r:
            value = json.load(r)
        self.assertTrue(value["armed"])
        self.assertTrue(value["home_requested"])
        self.assertEqual(value["control_owner"], "LOCAL")
        with self.request("/api/connect", {}) as r:
            self.assertNotEqual(json.load(r)["token"], token)
        with self.assertRaises(HTTPError) as ctx:
            self.request("/api/background", dict(token=token))
        self.assertEqual(ctx.exception.code, 403)

    def test_cross_origin_and_raw_file_access_are_rejected(self):
        with self.assertRaises(HTTPError) as ctx:
            self.request("/api/connect", {}, "http://foreign.invalid")
        self.assertEqual(ctx.exception.code, 403)
        with self.assertRaises(HTTPError) as ctx:
            self.request("/../main.py")
        self.assertEqual(ctx.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
