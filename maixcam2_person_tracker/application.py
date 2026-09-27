"""Application orchestration for the Stage 1 person-tracking loop."""

import os
import sys
from typing import Optional, Tuple

from maix import app, camera, display, time

from config import AppConfig
from control.gimbal_controller import GimbalController
from control.search_pattern import ExpandingSearchPattern
from control.state_machine import TrackingStateMachine
from domain import (
    AngularObservation,
    FrameResult,
    GimbalCommand,
    TrackState,
    TrackingMode,
)
from drivers.gimbal_uart import F32CGimbal, VisionOnlyGimbal
from estimation.camera_model import CameraModel
from estimation.target_filter import TargetFilter
from perception.detector_yolo import YoloPersonDetector
from perception.target_selector import TargetSelector
from telemetry.csv_logger import AsyncCsvLogger
from telemetry.mjpeg_stream import MjpegStreamer
from telemetry.overlay import draw_overlay
from telemetry.workbench_publisher import WorkbenchPublisher


class TrackerApplication:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.detector = YoloPersonDetector(config.model)
        self.camera = camera.Camera(
            self.detector.input_width,
            self.detector.input_height,
            self.detector.input_format,
            buff_num=config.camera.buffer_count,
        )
        self.display = display.Display() if config.ui.enabled else None
        self.camera_model = CameraModel(
            self.detector.input_width,
            self.detector.input_height,
            config.camera,
        )
        self.selector = TargetSelector(config.selection, self.camera_model)
        self.target_filter = TargetFilter(config.filter)
        self.state_machine = TrackingStateMachine(
            config.state_machine, config.selection.confirmation_frames
        )
        self.controller = GimbalController(config.gimbal)
        self.search_pattern = ExpandingSearchPattern(config.search)
        self.gimbal = (
            F32CGimbal(config.gimbal)
            if config.gimbal.enabled
            else VisionOnlyGimbal()
        )
        self.logger = AsyncCsvLogger(config.telemetry)
        self.streamer = MjpegStreamer()
        self.workbench = WorkbenchPublisher(
            config.workbench.endpoint or os.getenv("MAIX_WORKBENCH_URL", "")
        )
        self.process_started_ms: Optional[int] = None
        self.last_workbench_ms: Optional[int] = None
        self.last_capture_ms: Optional[int] = None
        self.measured_fps: Optional[float] = None
        self.last_command_ms: Optional[int] = None
        self.last_log_ms: Optional[int] = None
        self.last_console_ms: Optional[int] = None
        self.fault_handled = False

    def _desired_angles(
        self,
        now_ms: int,
        track: TrackState,
    ) -> Tuple[float, float]:
        if self.state_machine.mode == TrackingMode.SEARCH:
            entered_ms = self.state_machine.mode_entered_ms or now_ms
            elapsed_s = max(0.0, (now_ms - entered_ms) / 1000.0)
            return self.search_pattern.target(
                track.azimuth_deg,
                track.elevation_deg,
                elapsed_s,
            )
        return self.controller.predicted_target(track)

    def _maybe_command(
        self,
        now_ms: int,
        track: Optional[TrackState],
        gimbal_state,
    ) -> Optional[GimbalCommand]:
        if not self.config.gimbal.enabled:
            return None
        if track is None or not gimbal_state.valid:
            return None
        if self.state_machine.mode not in (
            TrackingMode.TRACK,
            TrackingMode.COAST,
            TrackingMode.SEARCH,
        ):
            return None
        if (
            self.last_command_ms is not None
            and now_ms - self.last_command_ms < self.config.gimbal.command_period_ms
        ):
            return None

        desired_pan, desired_tilt = self._desired_angles(now_ms, track)
        command = self.controller.command(
            now_ms,
            desired_pan,
            desired_tilt,
            gimbal_state,
        )
        self.gimbal.command(command)
        self.last_command_ms = now_ms
        return command

    def _handle_transition(
        self,
        previous_mode: TrackingMode,
        current_mode: TrackingMode,
    ) -> None:
        if previous_mode == TrackingMode.SEARCH and current_mode == TrackingMode.ACQUIRE:
            # Local reacquisition failed. Remove the old identity and return to a
            # true full-frame acquisition rather than gating on a stale prediction.
            self.selector.reset()
            self.target_filter.reset()

    def _status_message(self) -> str:
        if self.state_machine.mode == TrackingMode.FAULT:
            return self.state_machine.fault_reason
        if not self.config.gimbal.enabled:
            return "VISION ONLY - GIMBAL DISABLED"
        return ""

    def _maybe_log(self, now_ms: int, result: FrameResult) -> None:
        if (
            self.last_log_ms is None
            or now_ms - self.last_log_ms >= self.config.telemetry.sample_period_ms
        ):
            self.logger.submit(result)
            self.last_log_ms = now_ms

    def _maybe_report(self, now_ms: int, result: FrameResult, detection_count: int) -> None:
        if (
            self.last_console_ms is not None
            and now_ms - self.last_console_ms < 1000
        ):
            return

        if result.detection is None:
            selected_text = "selected=none"
        else:
            selected_text = (
                f"selected={result.detection.confidence:.2f} "
                f"px=({result.detection.box.center_x:.1f},"
                f"{result.detection.box.center_y:.1f})"
            )

        if result.track is None:
            bearing_text = "bearing=unavailable"
        else:
            bearing_text = (
                f"bearing=({result.track.azimuth_deg:+.2f},"
                f"{result.track.elevation_deg:+.2f})deg "
                f"rate=({result.track.azimuth_rate_deg_s:+.2f},"
                f"{result.track.elevation_rate_deg_s:+.2f})deg/s"
            )

        print(
            f"[{now_ms}] mode={result.mode.value} "
            f"persons={detection_count} {selected_text} {bearing_text} "
            f"gimbal=({result.gimbal.pan_deg:+.2f},"
            f"{result.gimbal.tilt_deg:+.2f})deg"
        )
        self.last_console_ms = now_ms

    def _workbench_snapshot(
        self,
        now_ms: int,
        result: FrameResult,
        detection_count: int,
    ) -> dict:
        detection = result.detection
        track = result.track
        gimbal = result.gimbal
        command = result.command
        return {
            "schema_version": 1,
            "runtime": {
                "running": True,
                "script": os.path.basename(sys.argv[0]) or "main.py",
                "pid": os.getpid(),
                "uptime_seconds": max(
                    0.0,
                    (now_ms - (self.process_started_ms or now_ms)) / 1000.0,
                ),
                "model_path": self.config.model.path,
            },
            "camera": {
                "available": True,
                "width": self.detector.input_width,
                "height": self.detector.input_height,
                "fps": round(self.measured_fps, 1) if self.measured_fps else None,
            },
            "vision": {
                "person_count": detection_count,
                "selected_label": self.config.model.target_label if detection else None,
                "confidence": detection.confidence if detection else None,
                "center_x": detection.box.center_x if detection else None,
                "center_y": detection.box.center_y if detection else None,
                "class_id": detection.class_id if detection else None,
            },
            "tracking": {
                "mode": result.mode.value,
                "azimuth_deg": track.azimuth_deg if track else None,
                "elevation_deg": track.elevation_deg if track else None,
                "azimuth_rate_deg_s": track.azimuth_rate_deg_s if track else None,
                "elevation_rate_deg_s": track.elevation_rate_deg_s if track else None,
                "azimuth_sigma_deg": track.azimuth_sigma_deg if track else None,
                "elevation_sigma_deg": track.elevation_sigma_deg if track else None,
            },
            "gimbal": {
                "enabled": self.config.gimbal.enabled,
                "pan_deg": gimbal.pan_deg,
                "tilt_deg": gimbal.tilt_deg,
                "pan_valid": gimbal.pan_valid,
                "tilt_valid": gimbal.tilt_valid,
                "command_pan_deg": command.pan_target_deg if command else None,
                "command_tilt_deg": command.tilt_target_deg if command else None,
            },
            "system": {
                "camera_available": True,
                "npu_ready": True,
                "status": result.status_message,
            },
            "extensions": {},
        }

    def _maybe_publish(self, now_ms: int, result: FrameResult, detection_count: int) -> None:
        if (
            self.last_workbench_ms is not None
            and now_ms - self.last_workbench_ms < self.config.workbench.publish_period_ms
        ):
            return
        self.workbench.publish(self._workbench_snapshot(now_ms, result, detection_count))
        self.last_workbench_ms = now_ms

    def run(self) -> None:
        self.logger.start()
        self.gimbal.start()
        self.streamer.start()
        self.process_started_ms = time.ticks_ms()
        self.workbench.start()
        normal_exit = False

        try:
            while not app.need_exit():
                frame = self.camera.read()
                capture_ms = time.ticks_ms()
                if self.last_capture_ms is not None:
                    interval_ms = max(1, capture_ms - self.last_capture_ms)
                    instant_fps = 1000.0 / interval_ms
                    self.measured_fps = (
                        instant_fps
                        if self.measured_fps is None
                        else self.measured_fps * 0.8 + instant_fps * 0.2
                    )
                self.last_capture_ms = capture_ms

                self.gimbal.poll(capture_ms)
                gimbal_state = self.gimbal.state(capture_ms)
                feedback_age_ms = self.gimbal.feedback_age_ms(capture_ms)

                predicted_track = self.target_filter.predict_to(capture_ms)
                detections = self.detector.detect(frame, capture_ms)
                association_track = (
                    predicted_track
                    if self.state_machine.mode
                    in (TrackingMode.TRACK, TrackingMode.COAST, TrackingMode.SEARCH)
                    else None
                )
                selected = self.selector.select(
                    detections,
                    association_track,
                    gimbal_state,
                )

                track = predicted_track
                if selected is not None and gimbal_state.valid:
                    azimuth, elevation = self.camera_model.pixel_to_world(
                        selected.box.center_x,
                        selected.box.center_y,
                        gimbal_state,
                    )
                    track = self.target_filter.update(
                        AngularObservation(
                            timestamp_ms=capture_ms,
                            azimuth_deg=azimuth,
                            elevation_deg=elevation,
                            confidence=selected.confidence,
                        )
                    )

                previous_mode = self.state_machine.mode
                current_mode = self.state_machine.update(
                    now_ms=capture_ms,
                    measurement_valid=selected is not None,
                    track=track,
                    gimbal_feedback_valid=gimbal_state.valid,
                    gimbal_feedback_age_ms=feedback_age_ms,
                )
                self._handle_transition(previous_mode, current_mode)

                if current_mode == TrackingMode.FAULT and not self.fault_handled:
                    self.gimbal.enter_fault(capture_ms)
                    self.fault_handled = True

                command = self._maybe_command(capture_ms, track, gimbal_state)
                result = FrameResult(
                    mode=current_mode,
                    detection=selected,
                    track=track,
                    gimbal=gimbal_state,
                    command=command,
                    status_message=self._status_message(),
                )
                self._maybe_log(capture_ms, result)
                self._maybe_report(capture_ms, result, len(detections))
                self._maybe_publish(capture_ms, result, len(detections))

                draw_overlay(
                    frame,
                    result,
                    self.camera_model,
                    self.config.ui.overlay_scale,
                )
                self.streamer.update(frame)
                if self.display is not None:
                    self.display.show(frame)

                time.sleep_ms(1)

            normal_exit = True
        except Exception:
            now_ms = time.ticks_ms()
            if not self.fault_handled:
                self.gimbal.enter_fault(now_ms)
                self.fault_handled = True
            raise
        finally:
            now_ms = time.ticks_ms()
            if normal_exit:
                self.gimbal.shutdown(now_ms)
            self.workbench.close()
            self.streamer.close()
            self.logger.close()
