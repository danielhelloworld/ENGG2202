"""Regression checks for connection recovery and the shared pitch limit."""

import queue
import time
import unittest

from f32c_protocol import (
    FEEDBACK_TOTAL_ANGLE, F32CGimbal, F32CLimitError, F32CTimeout
)
from gimbal_gui import GimbalWorker, X_UI_MAX_DEG, X_UI_MIN_DEG
from k230_f32c_demo import K230F32CGimbal, K230Tracker
from test_feedback_diagnostics import SilentTransport
from test_safe_driver import RecordingTransport
from tracking_controller import ImageTrackingController, TrackingConfig


class SwitchableTransport(RecordingTransport):
    def __init__(self, silent=False):
        super().__init__()
        self.silent = silent

    def write(self, data):
        if self.silent and len(data) == 6 and data[2] == 0x0E:
            self.frames.append(bytes(data))
            return len(data)
        return super().write(data)


class ConnectionRegressionTests(unittest.TestCase):
    def test_garbled_frame_is_skipped_before_valid_feedback(self):
        transport = RecordingTransport()
        transport.position[1] = 456
        transport.rx.extend(bytes.fromhex("00 7A 01 01 00 00 00 00 00 7B"))
        bus = F32CGimbal(transport, feedback_timeout_s=0.05)
        self.assertEqual(bus.request_feedback(1, FEEDBACK_TOTAL_ANGLE), 456)

    def test_silent_start_does_not_enable_motors(self):
        transport = SilentTransport()
        bus = F32CGimbal(
            transport, feedback_timeout_s=0.005, feedback_retries=2
        )
        with self.assertRaises(F32CTimeout):
            bus.start(power_on_delay_s=0)
        self.assertFalse(bus.enabled)
        self.assertFalse(bus.referenced)
        self.assertFalse(
            any(len(frame) >= 3 and frame[2] == 0x06 for frame in transport.frames)
        )

    def _wait_for(self, events, kind, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                event_kind, payload = events.get(timeout=0.1)
            except queue.Empty:
                continue
            if event_kind == "error":
                self.fail(payload)
            if event_kind == kind:
                return payload
        self.fail("missing worker event: %s" % kind)

    def test_silent_gui_connection_recovers_without_reopening_port(self):
        events = queue.Queue()
        transport = SwitchableTransport(silent=True)
        worker = GimbalWorker(None, 20, 100, events, demo_mode=True)
        worker._open_transport = lambda: transport
        worker.start()
        try:
            port, zero, enabled = self._wait_for(events, "connected")
            self.assertIsNone(zero)
            self.assertFalse(enabled)
            self.assertTrue(worker.is_alive())
            self.assertFalse(any(
                len(frame) >= 3 and frame[2] == 0x06
                for frame in transport.frames
            ))
            transport.silent = False
            self.assertEqual(self._wait_for(events, "referenced"), (0.0, 0.0))
            self._wait_for(events, "position")
            self.assertFalse(worker.motor_enabled)
            self.assertEqual(
                {frame[1] for frame in transport.frames
                 if len(frame) >= 5 and frame[2] == 0x00},
                {1, 2},
            )
            worker.set_enabled(True)
            self.assertTrue(self._wait_for(events, "motor_enabled"))
            self.assertTrue(worker.is_alive())
        finally:
            worker.request_stop()
            worker.join(3)

    def test_feedback_loss_disables_but_keeps_serial_session(self):
        events = queue.Queue()
        transport = SwitchableTransport()
        worker = GimbalWorker(None, 20, 100, events, demo_mode=True)
        worker._open_transport = lambda: transport
        worker.start()
        try:
            self._wait_for(events, "connected")
            self._wait_for(events, "position")
            transport.silent = True
            self._wait_for(events, "feedback_issue")
            self.assertTrue(worker.is_alive())
            self.assertFalse(worker.motor_enabled)
            transport.silent = False
            self._wait_for(events, "feedback_restored")
            self._wait_for(events, "position")
            self.assertFalse(worker.motor_enabled)
        finally:
            worker.request_stop()
            worker.join(3)


class LimitRegressionTests(unittest.TestCase):
    def test_reenable_preloads_manually_moved_pose(self):
        transport = RecordingTransport()
        bus = F32CGimbal(transport)
        bus.start(power_on_delay_s=0)
        bus.disable()
        transport.position[1], transport.position[2] = 125, -250
        before = len(transport.frames)
        self.assertEqual(bus.enable_at_current_position(), (12.5, -25.0))
        commands = [frame[2] for frame in transport.frames[before:] if len(frame) >= 3]
        self.assertLess(commands.index(0x02), commands.index(0x06))
        self.assertEqual(transport.position, {1: 125, 2: -250})
        bus.stop()

    def test_reenable_refuses_pose_beyond_y_limit(self):
        transport = RecordingTransport()
        bus = F32CGimbal(transport)
        bus.start(power_on_delay_s=0)
        bus.disable()
        transport.position[2] = 1001
        before = len(transport.frames)
        with self.assertRaises(F32CLimitError):
            bus.enable_at_current_position()
        self.assertFalse(bus.enabled)
        self.assertFalse(any(
            frame[2] in (0x02, 0x06)
            for frame in transport.frames[before:] if len(frame) >= 3
        ))

    def test_gui_x_range_matches_label(self):
        self.assertEqual((X_UI_MIN_DEG, X_UI_MAX_DEG), (-360.0, 360.0))

    def test_tracking_cannot_widen_y_limit(self):
        controller = ImageTrackingController(
            TrackingConfig(y_dead_zone_width_deg=120.0)
        )
        self.assertEqual(controller.safe_y(200.0), 100.0)
        self.assertEqual(controller.safe_y(-200.0), -100.0)
        self.assertEqual((K230Tracker().y_min, K230Tracker().y_max), (-100, 100))

    def test_k230_direct_command_rejects_overtravel_before_write(self):
        class UART:
            def __init__(self):
                self.frames = []

            def write(self, data):
                self.frames.append(data)

        uart = UART()
        gimbal = K230F32CGimbal(uart)
        gimbal.enabled = True
        with self.assertRaises(ValueError):
            gimbal.set_angles(0, 100.1)
        self.assertEqual(uart.frames, [])


if __name__ == "__main__":
    unittest.main()
