"""PC control/session and real loopback HTTP tests; no physical motor access."""
import importlib.util
import json
import math
from pathlib import Path
import threading
import unittest
from urllib.request import Request, urlopen
from urllib.error import HTTPError

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("motor_web_bundle", ROOT / "main_web.py")
w = importlib.util.module_from_spec(spec)
spec.loader.exec_module(w)


class FakeMotor:
    def __init__(self):
        self.c = dict(w.CONFIG)
        self.arm_requested = False
        self.inputs = []
        self.confirmed = False
        self.home = False
        self.recheck_pending = False

    def request_recheck(self):
        if self.arm_requested:
            raise RuntimeError("DISARM before rechecking")
        self.recheck_pending = True
        self.confirmed = False

    def input(self, x, y):
        self.inputs.append((x, y))

    def request_arm(self, arm):
        if arm and not self.confirmed:
            raise RuntimeError("Confirm zero first")
        self.arm_requested = arm
        if not arm:
            self.home = False

    def confirm_reference(self):
        self.confirmed = True

    def request_home(self):
        if not self.arm_requested:
            raise RuntimeError("ARM first")
        self.home = True

    def snapshot(self):
        return dict(ready=True, armed=self.arm_requested, fault=None, angles=(12., -6.),
                    phase="ARMED" if self.arm_requested else "DISABLED", age=0.01,
                    reference_valid=True, zero_confirmed=self.confirmed, home_requested=self.home,
                    can_recheck=not self.arm_requested and not self.recheck_pending)


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.motor = FakeMotor()
        self.control = w.BrowserControl(self.motor, lambda: self.now)
        self.token = self.control.connect()["token"]
        self.seq = 0

    def send(self, action="", x=0, y=0):
        self.seq += 1
        return self.control.command(dict(token=self.token, seq=self.seq, x=x, y=y, action=action))

    def arm(self):
        self.send("zero")
        self.send("arm")

    def test_connect_does_not_enable(self):
        self.assertFalse(self.motor.arm_requested)
        self.assertEqual(self.motor.inputs[-1], (0, 0))

    def test_fault_blocks_manual_enable_session(self):
        self.control.stop(self.token)
        before = self.motor.snapshot()
        before.update(ready=False, fault="PITCH ID=2 116.2 deg, allowed -100..100")
        self.motor.snapshot = lambda: before
        with self.assertRaisesRegex(w.ControlError, "116.2"):
            self.control.connect()
        self.assertFalse(self.motor.arm_requested)

    def test_arm_movement_and_release(self):
        self.arm()
        self.send(x=1)
        self.assertEqual(self.motor.inputs[-1], (1, 0))
        self.send()
        self.assertEqual(self.motor.inputs[-1], (0, 0))
        self.assertTrue(self.motor.arm_requested)

    def test_timeout_disarms_and_rejects_delayed_command(self):
        self.arm()
        self.now += 0.61
        with self.assertRaises(w.ControlError):
            self.send(x=1)
        self.assertFalse(self.motor.arm_requested)
        self.assertIsNone(self.control.token)

    def test_state_polling_cannot_renew_lease(self):
        self.arm()
        for _ in range(10):
            self.now += 0.1
            self.control.state()
        self.assertFalse(self.motor.arm_requested)

    def test_timeout_cancels_home(self):
        self.arm()
        self.send("home")
        self.assertTrue(self.motor.home)
        self.now += 0.61
        self.control.poll()
        self.assertFalse(self.motor.home)
        self.assertFalse(self.motor.arm_requested)

    def test_stop_revokes_lease_and_no_automatic_rearm(self):
        self.arm()
        self.control.stop(self.token)
        with self.assertRaises(w.ControlError):
            self.send("arm")
        fresh = self.control.connect()
        self.assertNotEqual(self.token, fresh["token"])
        self.assertFalse(self.motor.arm_requested)

    def test_second_client_cannot_take_over_or_stop_owner(self):
        self.arm()
        with self.assertRaises(w.ControlError):
            self.control.connect()
        with self.assertRaises(w.ControlError):
            self.control.stop("other")
        self.assertTrue(self.motor.arm_requested)

    def test_duplicate_or_reordered_packet_cannot_refresh_heartbeat(self):
        self.send()
        count = len(self.motor.inputs)
        with self.assertRaises(w.ControlError):
            self.control.command(dict(token=self.token, seq=1, x=1, y=0, action=""))
        self.assertEqual(len(self.motor.inputs), count)

    def test_nonfinite_or_excess_vector_disarms(self):
        self.arm()
        with self.assertRaises(w.ControlError):
            self.send(x=math.nan)
        self.assertFalse(self.motor.arm_requested)

    def test_arm_with_held_input_is_rejected(self):
        self.send("zero")
        with self.assertRaises(w.ControlError):
            self.send("arm", x=1)
        self.assertFalse(self.motor.arm_requested)

    def test_recheck_revokes_old_control_token_without_enabling(self):
        self.control.recheck(self.token)
        self.assertTrue(self.motor.recheck_pending)
        self.assertIsNone(self.control.token)
        with self.assertRaises(w.ControlError):
            self.send("arm")
        self.assertFalse(self.motor.arm_requested)

    def test_recheck_cannot_interrupt_active_or_other_controller(self):
        self.arm()
        with self.assertRaises(w.ControlError):
            self.control.recheck(self.token)
        with self.assertRaises(w.ControlError):
            self.control.recheck(None)
        self.assertTrue(self.motor.arm_requested)

    def test_stop_without_control_session_can_cancel_check(self):
        self.control.recheck(self.token)
        self.control.stop(None)
        self.assertFalse(self.motor.arm_requested)

    def test_continuous_input_requires_release_after_8_seconds(self):
        self.arm()
        for _ in range(85):
            self.now += 0.1
            self.send(x=1)
        self.assertTrue(self.control.blocked)
        self.assertEqual(self.motor.inputs[-1], (0, 0))
        self.send()
        self.send(x=1)
        self.assertFalse(self.control.blocked)
        self.assertEqual(self.motor.inputs[-1], (1, 0))


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.motor = FakeMotor()
        self.control = w.BrowserControl(self.motor)
        self.server = w.make_http_server(self.control, ("127.0.0.1", 0))
        self.url = "http://127.0.0.1:%d" % self.server.server_port
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": .02})
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(1)

    def request(self, path, body=None, origin=None):
        headers = {"Content-Type": "application/json", "Origin": origin or self.url}
        req = Request(self.url+path, data=None if body is None else json.dumps(body).encode(), headers=headers)
        with urlopen(req, timeout=2) as response:
            return response.status, response.read()

    def test_page_and_status_are_read_only(self):
        _, page = self.request("/")
        self.assertIn("电脑控制".encode(), page)
        self.request("/api/state")
        self.assertFalse(self.motor.inputs)

    def test_http_control_then_stop(self):
        _, body = self.request("/api/connect", {})
        token = json.loads(body)["token"]
        for seq, action in enumerate(("zero", "arm", ""), 1):
            self.request("/api/control", dict(token=token, seq=seq, action=action, x=0, y=0))
        self.assertTrue(self.motor.arm_requested)
        self.request("/api/stop", {"token": token})
        self.assertFalse(self.motor.arm_requested)

    def test_cross_origin_cannot_claim_control(self):
        with self.assertRaises(HTTPError) as ctx:
            self.request("/api/connect", {}, "http://elsewhere.invalid")
        self.assertEqual(ctx.exception.code, 403)
        self.assertIsNone(self.control.token)

    def test_recheck_endpoint_without_active_session_does_not_enable(self):
        _, body = self.request("/api/recheck", {})
        self.assertEqual(json.loads(body)["can_recheck"], False)
        self.assertTrue(self.motor.recheck_pending)
        self.assertFalse(self.motor.arm_requested)

    def test_get_cannot_execute_commands_or_read_files(self):
        for path in ("/api/connect", "/api/stop", "/../../README.md"):
            with self.assertRaises(HTTPError) as ctx:
                self.request(path)
            self.assertEqual(ctx.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
