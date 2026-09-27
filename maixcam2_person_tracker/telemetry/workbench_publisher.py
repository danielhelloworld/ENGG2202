"""Best-effort asynchronous telemetry publisher for the desktop workbench."""

import json
import queue
import threading
from urllib.request import Request, urlopen


class WorkbenchPublisher:
    """Post the newest tracker snapshot without blocking the camera loop."""

    def __init__(self, endpoint: str = "") -> None:
        self.endpoint = endpoint.strip()
        self.enabled = bool(self.endpoint)
        self._queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        if not self.enabled:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="workbench-telemetry",
            daemon=True,
        )
        self._thread.start()

    def publish(self, payload: dict) -> None:
        if not self.enabled:
            return
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                pass

    def _run(self) -> None:
        while not self._stop.is_set() or not self._queue.empty():
            try:
                payload = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                request = Request(
                    self.endpoint,
                    data=body,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urlopen(request, timeout=0.5) as response:
                    response.read(128)
            except Exception:
                # Device-side vision and control must keep running if the host is
                # disconnected or the workbench server is not available.
                pass
            finally:
                self._queue.task_done()

    def close(self) -> None:
        if not self.enabled or self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=1.0)
