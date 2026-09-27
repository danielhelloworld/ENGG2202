"""Rate- and acceleration-limited position command generation."""

from typing import Optional, Tuple

from config import AxisConfig, GimbalConfig
from domain import GimbalCommand, GimbalState, TrackState


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class _AxisLimiter:
    def __init__(self, config: AxisConfig) -> None:
        self.config = config
        self.position: Optional[float] = None
        self.velocity = 0.0

    def reset(self, position: float) -> None:
        self.position = _clamp(
            position, self.config.minimum_deg, self.config.maximum_deg
        )
        self.velocity = 0.0

    def update(self, desired_position: float, dt_s: float) -> float:
        desired_position = _clamp(
            desired_position, self.config.minimum_deg, self.config.maximum_deg
        )
        if self.position is None or dt_s <= 0.0:
            self.reset(desired_position)
            return self.position

        desired_velocity = _clamp(
            (desired_position - self.position) / dt_s,
            -self.config.maximum_rate_deg_s,
            self.config.maximum_rate_deg_s,
        )
        velocity_delta = _clamp(
            desired_velocity - self.velocity,
            -self.config.maximum_acceleration_deg_s2 * dt_s,
            self.config.maximum_acceleration_deg_s2 * dt_s,
        )
        self.velocity += velocity_delta
        next_position = self.position + self.velocity * dt_s
        self.position = _clamp(
            next_position, self.config.minimum_deg, self.config.maximum_deg
        )
        if self.position in (self.config.minimum_deg, self.config.maximum_deg):
            self.velocity = 0.0
        return self.position


class GimbalController:
    def __init__(self, config: GimbalConfig) -> None:
        self.config = config
        self.pan = _AxisLimiter(config.pan)
        self.tilt = _AxisLimiter(config.tilt)
        self.last_timestamp_ms: Optional[int] = None

    def reset(self, gimbal: GimbalState) -> None:
        self.pan.reset(gimbal.pan_deg)
        self.tilt.reset(gimbal.tilt_deg)
        self.last_timestamp_ms = gimbal.timestamp_ms

    def predicted_target(self, track: TrackState) -> Tuple[float, float]:
        horizon = self.config.prediction_lookahead_s
        return (
            track.azimuth_deg + track.azimuth_rate_deg_s * horizon,
            track.elevation_deg + track.elevation_rate_deg_s * horizon,
        )

    def command(
        self,
        now_ms: int,
        desired_pan_deg: float,
        desired_tilt_deg: float,
        fallback_gimbal: GimbalState,
    ) -> GimbalCommand:
        if self.last_timestamp_ms is None:
            self.reset(fallback_gimbal)
        dt_ms = now_ms - (self.last_timestamp_ms or now_ms)
        dt_s = max(0.001, min(0.10, dt_ms / 1000.0))
        self.last_timestamp_ms = now_ms
        return GimbalCommand(
            timestamp_ms=now_ms,
            pan_target_deg=self.pan.update(desired_pan_deg, dt_s),
            tilt_target_deg=self.tilt.update(desired_tilt_deg, dt_s),
            enabled=True,
        )
