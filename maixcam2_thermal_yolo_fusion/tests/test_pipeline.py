"""Pipeline IPC/scheduling tests. Fake detector results are never hardware evidence."""
from pathlib import Path
import runpy
import socket
import struct
import subprocess
import sys
import threading
import time
import types
import unittest

import numpy as np

APP = Path(__file__).resolve().parents[1]
m = types.SimpleNamespace(**runpy.run_path(str(APP / "main.py")))


def reply(channel, header, data):
    m.ipc_send(channel, {"event": "result", "captured_at": header["captured_at"], "inference_ms": 10,
        "detections": [{"box": [1, 1, 5, 5], "score": 1., "label": str(data[0]),
                        "captured_at": header["captured_at"]}]})


def await_condition(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError("Condition did not complete")


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.c = dict(m.CONFIG)

    def test_latest_frame_replaces_pending_without_blocking_producer(self):
        left, right = socket.socketpair()
        detector = m.AsyncDetector(self.c, left)
        first_seen, release = threading.Event(), threading.Event()
        received = []
        def worker():
            try:
                for index in range(2):
                    h, data = m.ipc_receive(right)
                    received.append(data[0])
                    if index == 0:
                        first_seen.set()
                        release.wait(2)
                    reply(right, h, data)
            finally:
                right.close()
        thread = threading.Thread(target=worker)
        thread.start()
        try:
            detector.submit(np.full((8, 8, 3), 1, np.uint8), time.monotonic())
            self.assertTrue(first_seen.wait(2))
            detector.submit(np.full((8, 8, 3), 2, np.uint8), time.monotonic())
            detector.submit(np.full((8, 8, 3), 3, np.uint8), time.monotonic())
            self.assertEqual(detector.replaced, 1)
            release.set()
            await_condition(lambda: detector.completed == 2)
            self.assertEqual(received, [1, 3])
        finally:
            release.set()
            detector.close()
            thread.join(2)

    def test_future_and_stale_detections_hidden(self):
        left, right = socket.socketpair()
        detector = m.AsyncDetector(self.c, left)
        try:
            detector.latest = {"captured_at": 10., "inference_ms": 30., "detections": [{"label": "test"}]}
            self.assertEqual(detector.snapshot(9.)[0], [])
            self.assertEqual(detector.snapshot(11.)[0], [])
            self.assertEqual(len(detector.snapshot(10.1)[0]), 1)
        finally:
            detector.close()
            right.close()

    def test_disabled_inflight_result_cannot_reappear_after_calibration(self):
        left, right = socket.socketpair()
        detector = m.AsyncDetector(self.c, left)
        right.settimeout(2)
        try:
            detector.submit(np.zeros((8, 8, 3), np.uint8), time.monotonic())
            h, data = m.ipc_receive(right)
            detector.set_enabled(False)
            detector.set_enabled(True)
            reply(right, h, data)
            await_condition(lambda: detector.completed == 1)
            self.assertEqual(detector.snapshot(time.monotonic())[0], [])
        finally:
            detector.close()
            right.close()

    def test_worker_error_clears_result_and_disables_further_submission(self):
        left, right = socket.socketpair()
        detector = m.AsyncDetector(self.c, left)
        right.settimeout(2)
        try:
            detector.submit(np.zeros((8, 8, 3), np.uint8), time.monotonic())
            m.ipc_receive(right)
            m.ipc_send(right, {"event": "error", "error": "TEST failure"})
            await_condition(lambda: bool(detector.error))
            detector.submit(np.zeros((8, 8, 3), np.uint8), time.monotonic())
            self.assertIsNone(detector.pending)
            self.assertEqual(detector.snapshot(time.monotonic())[0], [])
        finally:
            detector.close()
            right.close()

    def test_ipc_rejects_oversize_header(self):
        left, right = socket.socketpair()
        try:
            right.sendall(struct.pack('>I', 1000000000))
            with self.assertRaises(ValueError):
                m.ipc_receive(left)
        finally:
            left.close()
            right.close()

    def test_cached_heat_reused_and_invalidated_by_frame_or_geometry(self):
        a = m.Alignment(self.c, (640, 480))
        rgb, thermal, _ = m.demo_frame()
        first = a.heat_image(thermal)
        self.assertIs(a.heat_image(thermal), first)
        a.adjust("FAR")
        second = a.heat_image(thermal)
        self.assertIsNot(first, second)
        self.assertIsNot(a.heat_image(dict(thermal)), second)
        stale, _, _ = m.compose(rgb, [], thermal, a, thermal["received_at"] + 2, thermal["received_at"] + 2)
        np.testing.assert_array_equal(stale[200, 200], rgb[200, 200])

    def test_detection_temperature_requires_matching_capture_time(self):
        a = m.Alignment(self.c, (640, 480))
        rgb, thermal, detections = m.demo_frame()
        now = thermal["received_at"]
        detections[0]["captured_at"] = now - .25
        _, values, _ = m.compose(rgb, detections, thermal, a, now, now)
        self.assertIsNone(values[0])
        detections[0]["captured_at"] = now - .03
        _, values, _ = m.compose(rgb, detections, thermal, a, now, now)
        self.assertIsNotNone(values[0])

    def test_unlimited_render_and_inference_fps_are_valid(self):
        m.validate_config(dict(self.c, loop_fps=0, inference_fps=0, stream_fps=60))
        with self.assertRaises(ValueError):
            m.validate_config(dict(self.c, loop_fps=-1))

    def test_rate_meter_counts_new_frames_not_refreshes(self):
        meter = m.RateMeter()
        meter.at = 10.
        self.assertEqual(meter.update(20, 11.), 20.)
        self.assertEqual(meter.update(20, 12.), 0.)

    def test_real_child_process_protocol_while_parent_renders(self):
        # Uses a local test-only TCP socket for Windows portability. Board uses socketpair/pass_fds.
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(10)
        child = subprocess.Popen([sys.executable, '-B', __file__, '--fake-worker', str(listener.getsockname()[1])])
        detector = None
        try:
            channel, _ = listener.accept()
            channel.settimeout(3)
            detector = m.AsyncDetector(self.c, channel, child)
            rgb, thermal, _ = m.demo_frame()
            a = m.Alignment(self.c, (640, 480))
            detector.submit(rgb, time.monotonic())
            frames = 0
            deadline = time.monotonic() + 3
            while detector.completed == 0 and time.monotonic() < deadline:
                now = time.monotonic()
                m.compose(rgb, [], thermal, a, now, now)
                frames += 1
            self.assertEqual(detector.completed, 1)
            self.assertGreater(frames, 0)
            self.assertEqual(detector.snapshot(time.monotonic())[1]['error'], '')
        finally:
            if detector:
                detector.close()
            else:
                child.terminate()
                child.wait(3)
            listener.close()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--fake-worker':
        with socket.create_connection(('127.0.0.1', int(sys.argv[2])), timeout=5) as channel:
            h, data = m.ipc_receive(channel)
            deadline = time.monotonic() + .15
            while time.monotonic() < deadline:
                sum(i * i for i in range(1000))  # Deliberately holds this child's Python GIL.
            reply(channel, h, data)
            try:
                channel.recv(1)
            except OSError:
                pass
    else:
        unittest.main(verbosity=2)
