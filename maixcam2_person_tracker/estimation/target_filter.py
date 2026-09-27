"""Small dependency-free constant-angular-velocity Kalman filter."""

import math
from typing import Optional

from config import FilterConfig
from domain import AngularObservation, TrackState


class _AxisKalman:
    def __init__(self, config: FilterConfig) -> None:
        self.config = config
        self.initialized = False
        self.position = 0.0
        self.rate = 0.0
        self.p00 = config.initial_position_variance
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = config.initial_rate_variance

    def initialize(self, position: float) -> None:
        self.position = position
        self.rate = 0.0
        self.p00 = self.config.initial_position_variance
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = self.config.initial_rate_variance
        self.initialized = True

    def predict(self, dt_s: float) -> None:
        if not self.initialized or dt_s <= 0.0:
            return

        dt = min(dt_s, self.config.maximum_prediction_step_s)
        self.position += self.rate * dt

        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11
        q = self.config.angular_acceleration_noise
        q00 = 0.25 * dt ** 4 * q
        q01 = 0.5 * dt ** 3 * q
        q11 = dt * dt * q

        self.p00 = old_p00 + dt * (old_p10 + old_p01) + dt * dt * old_p11 + q00
        self.p01 = old_p01 + dt * old_p11 + q01
        self.p10 = old_p10 + dt * old_p11 + q01
        self.p11 = old_p11 + q11

    def update(self, measurement: float) -> None:
        if not self.initialized:
            self.initialize(measurement)
            return

        measurement_variance = self.config.measurement_noise_deg ** 2
        innovation = measurement - self.position
        innovation_variance = self.p00 + measurement_variance
        gain_position = self.p00 / innovation_variance
        gain_rate = self.p10 / innovation_variance

        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11

        self.position += gain_position * innovation
        self.rate += gain_rate * innovation
        self.p00 = (1.0 - gain_position) * old_p00
        self.p01 = (1.0 - gain_position) * old_p01
        self.p10 = old_p10 - gain_rate * old_p00
        self.p11 = old_p11 - gain_rate * old_p01

        # Numerical symmetry prevents slow covariance drift on long runs.
        symmetric_cross = 0.5 * (self.p01 + self.p10)
        self.p01 = symmetric_cross
        self.p10 = symmetric_cross

    @property
    def sigma(self) -> float:
        return math.sqrt(max(0.0, self.p00))


class TargetFilter:
    def __init__(self, config: FilterConfig) -> None:
        self.azimuth = _AxisKalman(config)
        self.elevation = _AxisKalman(config)
        self.timestamp_ms: Optional[int] = None

    @property
    def initialized(self) -> bool:
        return self.azimuth.initialized and self.elevation.initialized

    def reset(self) -> None:
        self.azimuth = _AxisKalman(self.azimuth.config)
        self.elevation = _AxisKalman(self.elevation.config)
        self.timestamp_ms = None

    def predict_to(self, timestamp_ms: int) -> Optional[TrackState]:
        if not self.initialized or self.timestamp_ms is None:
            return None

        dt_ms = timestamp_ms - self.timestamp_ms
        if dt_ms < 0:
            dt_ms = 0
        remaining_s = dt_ms / 1000.0
        maximum_step = self.azimuth.config.maximum_prediction_step_s
        while remaining_s > 0.0:
            step = min(remaining_s, maximum_step)
            self.azimuth.predict(step)
            self.elevation.predict(step)
            remaining_s -= step
        self.timestamp_ms = timestamp_ms
        return self.state(timestamp_ms)

    def update(self, observation: AngularObservation) -> TrackState:
        if not self.initialized:
            self.azimuth.initialize(observation.azimuth_deg)
            self.elevation.initialize(observation.elevation_deg)
            self.timestamp_ms = observation.timestamp_ms
        else:
            self.predict_to(observation.timestamp_ms)
            self.azimuth.update(observation.azimuth_deg)
            self.elevation.update(observation.elevation_deg)
        return self.state(observation.timestamp_ms)

    def state(self, timestamp_ms: int) -> TrackState:
        return TrackState(
            timestamp_ms=timestamp_ms,
            azimuth_deg=self.azimuth.position,
            elevation_deg=self.elevation.position,
            azimuth_rate_deg_s=self.azimuth.rate,
            elevation_rate_deg_s=self.elevation.rate,
            azimuth_sigma_deg=self.azimuth.sigma,
            elevation_sigma_deg=self.elevation.sigma,
        )
