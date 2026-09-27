"""Explicit operating-state transitions for acquisition and loss recovery."""

from typing import Optional

from config import StateMachineConfig
from domain import TrackState, TrackingMode


class TrackingStateMachine:
    def __init__(self, config: StateMachineConfig, confirmation_frames: int) -> None:
        self.config = config
        self.confirmation_frames = confirmation_frames
        self.mode = TrackingMode.BOOT
        self.started_ms: Optional[int] = None
        self.mode_entered_ms: Optional[int] = None
        self.consecutive_measurements = 0
        self.fault_reason = ""

    def _transition(self, mode: TrackingMode, now_ms: int) -> None:
        self.mode = mode
        self.mode_entered_ms = now_ms
        if mode not in (TrackingMode.ACQUIRE, TrackingMode.SEARCH):
            self.consecutive_measurements = 0

    def set_fault(self, now_ms: int, reason: str) -> TrackingMode:
        self.fault_reason = reason
        self._transition(TrackingMode.FAULT, now_ms)
        return self.mode

    def update(
        self,
        now_ms: int,
        measurement_valid: bool,
        track: Optional[TrackState],
        gimbal_feedback_valid: bool,
        gimbal_feedback_age_ms: int,
    ) -> TrackingMode:
        if self.started_ms is None:
            self.started_ms = now_ms
            self.mode_entered_ms = now_ms

        if self.mode == TrackingMode.FAULT:
            return self.mode

        if self.mode == TrackingMode.BOOT:
            if gimbal_feedback_valid:
                self._transition(TrackingMode.ACQUIRE, now_ms)
            elif now_ms - self.started_ms > self.config.boot_feedback_timeout_ms:
                return self.set_fault(now_ms, "No valid gimbal feedback during boot")
            return self.mode

        if (
            not gimbal_feedback_valid
            or gimbal_feedback_age_ms > self.config.feedback_stale_timeout_ms
        ):
            return self.set_fault(now_ms, "Gimbal feedback became stale")

        if self.mode in (TrackingMode.ACQUIRE, TrackingMode.SEARCH):
            if measurement_valid:
                self.consecutive_measurements += 1
                if self.consecutive_measurements >= self.confirmation_frames:
                    self._transition(TrackingMode.TRACK, now_ms)
            else:
                self.consecutive_measurements = 0

            if (
                self.mode == TrackingMode.SEARCH
                and self.mode_entered_ms is not None
                and now_ms - self.mode_entered_ms > self.config.search_timeout_ms
            ):
                self._transition(TrackingMode.ACQUIRE, now_ms)
            return self.mode

        if self.mode == TrackingMode.TRACK:
            if not measurement_valid:
                self._transition(TrackingMode.COAST, now_ms)
            return self.mode

        if self.mode == TrackingMode.COAST:
            if measurement_valid:
                self._transition(TrackingMode.TRACK, now_ms)
                return self.mode

            coast_elapsed_ms = now_ms - (self.mode_entered_ms or now_ms)
            sigma_too_large = track is None or max(
                track.azimuth_sigma_deg, track.elevation_sigma_deg
            ) > self.config.maximum_coast_sigma_deg
            if coast_elapsed_ms > self.config.coast_timeout_ms or sigma_too_large:
                self._transition(TrackingMode.SEARCH, now_ms)
            return self.mode

        return self.mode
