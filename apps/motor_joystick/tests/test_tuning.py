"""Software/transport models only. No claim about real motor PID dynamics."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.request import urlopen

import test_demo as fixtures
import test_web

w = test_web.w
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("feedback_bridge", ROOT/"feedback_bridge.py")
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class TuningTests(unittest.TestCase):
    initialize = fixtures.MotorTests.initialize
    arm = fixtures.MotorTests.arm
    commands = fixtures.MotorTests.commands

    def setUp(self):
        self.clock = fixtures.Clock()
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(w.time, "monotonic", self.clock.monotonic))
        self.stack.enter_context(patch.object(w.time, "sleep", self.clock.sleep))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.c = copy.deepcopy(w.CONFIG)
        self.bus = fixtures.MechanicalMotorModel((8, -12))
        self.s = w.TuningMotorService(self.c, lambda: self.bus)
        self.s.serial = self.bus
        self.s.stop_event = fixtures.Event(self.clock)

    def stationary(self, n=24):
        for _ in range(n):
            self.clock.sleep(.02)
            self.s._measure()

    def test_connect_and_manual_arm_never_overwrite_unknown_pi(self):
        self.arm()
        self.assertFalse(self.commands(0x0F)+self.commands(0x10))
        self.assertEqual(self.s.pi_last_sent, [None, None])
        self.assertEqual([d[1] for d in self.commands(6)], [2, 1])
        self.assertEqual(self.bus.physical, {1: 8, 2: -12})

    def test_pi_mailbox_only_writes_requested_parameter_on_selected_axis(self):
        self.arm()
        self.stationary()
        before = len(self.bus.writes)
        self.s.request_speed_pi(1, kp=12)
        self.assertEqual(len(self.bus.writes), before)
        self.s._measure()
        self.assertEqual(self.commands(0x0F), [w._frame(2, 0x0F, b"\x00\x0c")])
        self.assertFalse(self.commands(0x10))
        self.assertTrue(all(self.bus.enabled.values()))
        self.assertEqual(self.s.pi_last_sent[1], [12, None])
        self.s.request_speed_pi(1, ki=7)
        self.s._measure()
        self.assertEqual(self.s.pi_last_sent[1], [12, 7])
        self.assertFalse(self.commands(8)+self.commands(9)+self.commands(10))

    def test_invalid_pi_is_rejected_before_any_parameter_write(self):
        self.initialize()
        for axis, kp, ki in ((1, 12, 257), (True, 12, 7), (0, True, None), (1, None, None), (0, float("nan"), 7)):
            with self.assertRaises(ValueError):
                self.s.request_speed_pi(axis, kp, ki)
        self.assertFalse(self.commands(0x0F)+self.commands(0x10))

    def test_invalid_web_tuning_request_keeps_existing_hold(self):
        self.arm()
        self.s.home_holding = self.s.startup_waiting_owner = True
        self.s.phase = "HOME_HOLD"
        control = w.BrowserControl(self.s, self.clock.monotonic)
        token = control.connect()["token"]
        state = control.command(dict(token=token, seq=1, x=0, y=0, action="pi", axis=1, kp=999))
        self.assertTrue(state["connected"] and state["armed"] and self.s.arm_requested)
        self.assertIn("0..256", state["tuning_error"])
        self.assertFalse(self.commands(0x0F))

    def test_moving_load_or_nonzero_speed_rejects_pi_without_disarming(self):
        self.arm()
        self.stationary()
        self.bus.nonzero_speed = True
        self.s.request_speed_pi(0, kp=11)
        self.s._measure()
        self.assertFalse(self.commands(0x0F))
        self.assertTrue(all(self.bus.enabled.values()))
        self.assertIn("not zero", self.s.pi_status)
        self.s.input(.1, 0)
        with self.assertRaises(RuntimeError):
            self.s.request_speed_pi(0, kp=11)

    def test_disarm_between_pair_writes_stops_second_write_and_marks_unknown(self):
        self.arm()
        self.stationary()
        original = self.bus.write

        def cancel(data):
            value = original(data)
            if data[2] == 0x0F:
                self.s.request_arm(False)
            return value

        self.bus.write = cancel
        self.s.request_speed_pi(1, 11, 8)
        with self.assertRaises(w.UserDisarm):
            self.s._measure()
        self.assertEqual(len(self.commands(0x0F)), 1)
        self.assertFalse(self.commands(0x10))
        self.assertIsNone(self.s.pi_last_sent[1])

    def test_startup_homes_before_controller_and_transfer_keeps_torque(self):
        observed = []
        control = w.BrowserControl(self.s, self.clock.monotonic)

        def schedule():
            if self.s.startup_waiting_owner:
                before = len(self.commands(5))
                joined = control.connect()
                observed.append((self.s.snapshot(), before, len(self.commands(5))))
                self.assertTrue(joined["token"])
                self.s.stop_event.set()
            elif self.s.fault:
                self.s.stop_event.set()

        self.s.stop_event.callback = schedule
        self.s._run()
        self.assertTrue(observed, self.s.fault)
        state, before, after = observed[0]
        self.assertIsNone(state["fault"])
        self.assertTrue(state["armed"])
        self.assertEqual(state["phase"], "HOME_HOLD")
        self.assertTrue(all(abs(v) <= .5 for v in state["angles"]))
        self.assertEqual(before, after)  # No drop during controller handover.
        self.assertFalse(self.commands(8)+self.commands(9)+self.commands(10))

    def test_startup_can_be_cancelled_and_does_not_automatically_retry(self):
        cancelled = []

        def schedule():
            if self.s.startup_active and self.s.phase == "RETURNING":
                self.s.request_arm(False)
            if self.s.startup_status == "CANCELLED":
                cancelled.append(self.s.phase)
                self.s.stop_event.set()
            if self.s.fault:
                self.s.stop_event.set()

        self.s.stop_event.callback = schedule
        self.s._run()
        self.assertTrue(cancelled)
        self.assertEqual(self.s.startup_status, "CANCELLED")
        self.assertFalse(any(self.bus.enabled.values()))

    def test_disarm_before_startup_feedback_prevents_automatic_enable(self):
        self.s.request_arm(False)
        self.s.stop_event.callback = self.s.stop_event.set
        self.s._run()
        self.assertEqual(self.s.startup_status, "CANCELLED")
        self.assertFalse(self.commands(6)+self.commands(3))

    def test_manual_owner_timeout_still_disables_after_startup_handover(self):
        self.arm()
        self.s.home_holding = self.s.startup_waiting_owner = True
        self.s.phase = "HOME_HOLD"
        control = w.BrowserControl(self.s, self.clock.monotonic)
        control.connect()
        self.clock.pump = None
        self.clock.sleep(.61)
        control.poll()
        self.assertFalse(self.s.arm_requested)
        with self.assertRaises(w.UserDisarm):
            self.s._heartbeat_check()

    def test_each_stage_applies_individual_axis_profile(self):
        tuning = copy.deepcopy(w.TUNING_CONFIG)
        tuning["profiles"]["startup"].update(rpm=(3, 2), rate=(12, 8), acceleration=(6, 4))
        tuning["profiles"]["manual"].update(rpm=(5, 4), rate=(20, 12), acceleration=(10, 8))
        tuning["profiles"]["home"].update(rpm=(2, 2), rate=(8, 6), acceleration=(4, 3))
        self.s = w.TuningMotorService(self.c, lambda: self.bus, tuning)
        self.s.serial = self.bus
        self.s.stop_event = fixtures.Event(self.clock)
        self.arm()
        self.assertEqual([int.from_bytes(d[3:5], "big") for d in self.commands(1)], [3, 2])
        self.s._advance(self.s.angles, (100, 100))
        self.assertEqual(self.s.drive_profile, "manual")
        self.assertEqual([int.from_bytes(d[3:5], "big") for d in self.commands(1)][-2:], [5, 4])
        self.s.phase = "RETURNING"
        self.s._advance((0, 0), (100, 100))
        self.assertEqual(self.s.drive_profile, "home")
        self.assertEqual([int.from_bytes(d[3:5], "big") for d in self.commands(1)][-2:], [2, 2])

    def test_recording_is_bounded_and_only_keeps_accepted_feedback(self):
        self.initialize()
        self.s.tuning_config["record_samples"] = 100
        self.s.start_tuning_capture("payload baseline, original PI unknown")
        before = len(self.commands(3)+self.commands(6))
        self.stationary(115)
        result = self.s.export_tuning()
        self.assertFalse(result["recording"])
        self.assertEqual(len(result["samples"]), 100)
        self.assertEqual(result["end_reason"], "recording limit")
        self.assertEqual(len(self.commands(3)+self.commands(6)), before)
        self.assertIsNone(result["summary"]["axes"][0]["holding_peak_to_peak_deg"])
        self.s.start_tuning_capture()
        self.bus.drop_feedback = {2}
        with self.assertRaises(RuntimeError):
            self.s._measure()
        self.s._record_fault(RuntimeError("missing mechanical feedback"))
        failed = self.s.export_tuning()
        self.assertEqual(failed["samples"], [])
        self.assertIn("fault", failed["end_reason"])

    def test_export_http_is_read_only_and_has_download_header(self):
        self.initialize()
        self.s.start_tuning_capture()
        self.stationary()
        self.s.stop_tuning_capture()
        server = w.make_http_server(w.BrowserControl(self.s), ("127.0.0.1", 0))
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01})
        thread.start()
        before = len(self.bus.writes)
        try:
            with urlopen("http://127.0.0.1:%d/api/tuning/export" % server.server_port) as response:
                self.assertIn("attachment", response.headers["Content-Disposition"])
                result = json.load(response)
            self.assertGreater(len(result["samples"]), 10)
            self.assertEqual(len(self.bus.writes), before)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(1)


class MetricsTests(unittest.TestCase):
    def samples(self, gap=False, moving=False):
        return [dict(time_s=i*.1+(1 if gap and i>5 else 0), armed=True,
                     target_deg=(i*.1 if moving else 0, 0),
                     angle_deg=(.1 if i%2 else -.1, .2)) for i in range(12)]

    def test_hold_jitter_is_measured_against_sent_target(self):
        value = w.summarize_tuning_trace(self.samples())
        self.assertAlmostEqual(value["axes"][0]["holding_peak_to_peak_deg"], .2)
        self.assertAlmostEqual(value["axes"][1]["holding_mean_error_deg"], .2)
        self.assertAlmostEqual(value["sample_rate_hz"], 10)

    def test_gaps_or_varying_targets_do_not_count_as_static_holding(self):
        for rows in (self.samples(gap=True), self.samples(moving=True)):
            value = w.summarize_tuning_trace(rows)
            self.assertIsNone(value["axes"][0]["holding_peak_to_peak_deg"])

    def test_bridge_rejects_ambiguous_or_unbounded_commands(self):
        for body in ({"action":"pulse", "x":float("nan"),"seconds":.2},
                     {"action":"pulse", "x":1,"seconds":20},
                     {"action":"pi","axis":1,"kp":True}, {"action":"pi","axis":0}):
            with self.assertRaises(ValueError):
                bridge.validate_bridge_command(body)
        self.assertEqual(bridge.validate_bridge_command({"action":"pi","axis":1,"kp":12})["kp"],12)


if __name__ == "__main__":
    unittest.main()
