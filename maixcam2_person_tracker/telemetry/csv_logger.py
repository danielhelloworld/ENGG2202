"""Bounded asynchronous CSV logging for closed-loop tuning."""

import csv
import os
import queue
import threading
from typing import Optional

from config import TelemetryConfig
from domain import FrameResult


class AsyncCsvLogger:
    HEADER = (
        "timestamp_ms",
        "mode",
        "detection_confidence",
        "bbox_center_x",
        "bbox_center_y",
        "target_azimuth_deg",
        "target_elevation_deg",
        "target_azimuth_rate_deg_s",
        "target_elevation_rate_deg_s",
        "azimuth_sigma_deg",
        "elevation_sigma_deg",
        "gimbal_pan_deg",
        "gimbal_tilt_deg",
        "command_pan_deg",
        "command_tilt_deg",
        "status",
    )

    def __init__(self, config: TelemetryConfig) -> None:
        self.config = config
        self.events = queue.Queue(maxsize=config.queue_capacity)
        self.stop_event = threading.Event()
        self.worker: Optional[threading.Thread] = None

    def start(self) -> None:
        if not self.config.enabled:
            return
        directory = os.path.dirname(self.config.csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def submit(self, result: FrameResult) -> None:
        if not self.config.enabled:
            return
        try:
            self.events.put_nowait(result)
        except queue.Full:
            # Control timing has priority over diagnostic completeness.
            pass

    def _row(self, result: FrameResult):
        detection = result.detection
        track = result.track
        command = result.command
        return (
            result.gimbal.timestamp_ms,
            result.mode.value,
            "" if detection is None else detection.confidence,
            "" if detection is None else detection.box.center_x,
            "" if detection is None else detection.box.center_y,
            "" if track is None else track.azimuth_deg,
            "" if track is None else track.elevation_deg,
            "" if track is None else track.azimuth_rate_deg_s,
            "" if track is None else track.elevation_rate_deg_s,
            "" if track is None else track.azimuth_sigma_deg,
            "" if track is None else track.elevation_sigma_deg,
            result.gimbal.pan_deg,
            result.gimbal.tilt_deg,
            "" if command is None else command.pan_target_deg,
            "" if command is None else command.tilt_target_deg,
            result.status_message,
        )

    def _run(self) -> None:
        file_exists = os.path.exists(self.config.csv_path)
        with open(self.config.csv_path, "a", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            if not file_exists or os.path.getsize(self.config.csv_path) == 0:
                writer.writerow(self.HEADER)
            pending_flush = 0
            while not self.stop_event.is_set() or not self.events.empty():
                try:
                    result = self.events.get(timeout=0.25)
                except queue.Empty:
                    output.flush()
                    continue
                writer.writerow(self._row(result))
                pending_flush += 1
                if pending_flush >= 10:
                    output.flush()
                    pending_flush = 0
            output.flush()

    def close(self) -> None:
        if not self.config.enabled or self.worker is None:
            return
        self.stop_event.set()
        self.worker.join(timeout=2.0)
