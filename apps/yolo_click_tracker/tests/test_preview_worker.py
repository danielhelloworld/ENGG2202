"""Slow displays must neither build a FIFO nor advance the click frame early."""
import threading
import unittest
import test_tracker as app

m = app.m


class PreviewTests(unittest.TestCase):
    def test_slow_renderer_keeps_only_latest_pending_and_clicks_last_displayed(self):
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        rendered = []
        def render(frame, image, selected, prediction):
            if frame["id"] == 1:
                entered.set()
                if not release.wait(1):
                    raise RuntimeError("test render timeout")
            rendered.append(frame["id"])
            if frame["id"] == 3:
                finished.set()
        worker = m.LatestPreview(render, 100, m.StageTimings())
        try:
            worker.offer(dict(id=1, stamp=m.time.monotonic()), object(), None, None)
            worker.thread.start()
            self.assertTrue(entered.wait(1))
            worker.offer(dict(id=2, stamp=m.time.monotonic()), object(), None, None)
            worker.offer(dict(id=3, stamp=m.time.monotonic()), object(), None, None)
            self.assertIsNone(worker.displayed)
            self.assertEqual(worker.pending[0]["id"], 3)
            release.set()
            self.assertTrue(finished.wait(1))
        finally:
            release.set()
            worker.close()
        self.assertEqual(rendered[:2], [1, 3])
        self.assertEqual(worker.displayed["id"], 3)
        self.assertEqual(worker.replaced, 1)
        self.assertIsNone(worker.pending)
        self.assertFalse(worker.thread.is_alive())

    def test_renderer_failure_is_visible_to_runtime_and_closes(self):
        failed = threading.Event()
        def render(*args):
            failed.set()
            raise RuntimeError("display failed")
        worker = m.LatestPreview(render, 30, m.StageTimings())
        worker.offer(dict(id=1, stamp=m.time.monotonic()), object(), None, None)
        worker.thread.start()
        self.assertTrue(failed.wait(1))
        worker.close()
        self.assertIsInstance(worker.error, RuntimeError)
        self.assertIsNone(worker.displayed)

    def test_timings_are_bounded_and_reject_invalid_measurements(self):
        timings = m.StageTimings(3)
        for value in [1., 2., 3., 4., float("nan"), -1.]:
            timings.add("test", value)
        data = timings.snapshot()["test"]
        self.assertEqual(data, dict(samples=3, last_ms=4., p50_ms=3., p95_ms=4.))
        timings.add("test", 5.)
        self.assertEqual(data["last_ms"], 4.)  # Snapshot has no live mutable samples.


if __name__ == "__main__":
    unittest.main()
