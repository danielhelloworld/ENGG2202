"""Image-error to two-axis angle controller for the F32C gimbal."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass


@dataclass
class TrackingConfig:
    horizontal_fov_deg: float = 70.0
    vertical_fov_deg: float = 43.0
    x_direction: float = 1.0
    y_direction: float = -1.0
    proportional_gain: float = 0.65
    pixel_deadband: float = 6.0
    max_rate_deg_s: float = 70.0
    lost_timeout_s: float = 0.35
    y_dead_zone_width_deg: float = 160.0
    y_dead_zone_center_deg: float = 180.0
    y_dead_zone_margin_deg: float = 0.0


def _wrap_180(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class YAxisDeadZone:
    """Represent one forbidden circular sector as one continuous safe interval.

    The safe interval is selected so that logical 0 degrees lies inside it. With
    a 160 degree dead zone centered at 180 degrees, the result is
    ``[-100, +100]`` before applying the safety margin.
    """

    def __init__(self, width_deg: float, center_deg: float, margin_deg: float = 0.0):
        if not 0.0 <= width_deg < 360.0:
            raise ValueError("dead-zone width must be in [0, 360)")
        if margin_deg < 0.0:
            raise ValueError("dead-zone margin cannot be negative")
        self.width_deg = width_deg
        self.center_deg = _wrap_180(center_deg)
        self.margin_deg = margin_deg

        if width_deg == 0.0:
            self.low = -math.inf
            self.high = math.inf
            return

        half = width_deg / 2.0
        base_low = self.center_deg + half
        base_high = self.center_deg - half + 360.0
        candidates = [
            (base_low + shift, base_high + shift)
            for shift in (-720.0, -360.0, 0.0, 360.0, 720.0)
        ]
        containing_zero = [(lo, hi) for lo, hi in candidates if lo <= 0.0 <= hi]
        if not containing_zero:
            raise ValueError("logical zero lies inside the configured Y dead zone")
        self.low, self.high = min(
            containing_zero, key=lambda pair: abs(pair[0]) + abs(pair[1])
        )
        self.low += margin_deg
        self.high -= margin_deg
        if self.low > self.high:
            raise ValueError("dead-zone margin consumes the entire safe interval")

    def clamp(self, angle_deg: float) -> float:
        return min(self.high, max(self.low, angle_deg))


class ImageTrackingController:
    """Stateful proportional visual servo with rate limiting and Y protection."""

    def __init__(self, config: TrackingConfig | None = None) -> None:
        self.config = config or TrackingConfig()
        if self.config.x_direction not in (-1.0, 1.0):
            raise ValueError("x_direction must be +1 or -1")
        if self.config.y_direction not in (-1.0, 1.0):
            raise ValueError("y_direction must be +1 or -1")
        self.y_dead_zone = YAxisDeadZone(
            self.config.y_dead_zone_width_deg,
            self.config.y_dead_zone_center_deg,
            self.config.y_dead_zone_margin_deg,
        )
        self.x_deg = 0.0
        self.y_deg = self.safe_y(0.0)
        self.last_update_s: float | None = None
        self.last_seen_s: float | None = None

    @staticmethod
    def _pixel_error_to_angle(error_px: float, size_px: int, fov_deg: float) -> float:
        if size_px <= 0:
            raise ValueError("frame dimensions must be positive")
        normalized = 2.0 * error_px / float(size_px)
        return math.degrees(
            math.atan(normalized * math.tan(math.radians(fov_deg) / 2.0))
        )

    def safe_y(self, angle_deg: float) -> float:
        # Configurable dead-zone geometry may narrow the usable interval;
        # it may never widen the F32C pitch limit.
        return min(100.0, max(-100.0, self.y_dead_zone.clamp(angle_deg)))

    def update(
        self,
        *,
        target_cx: float | None,
        target_cy: float | None,
        frame_width: int,
        frame_height: int,
        now_s: float | None = None,
    ) -> tuple[float, float, bool]:
        """Return ``(x_deg, y_deg, target_active)`` for one detector result.

        Pass ``None`` coordinates when the detector has no target. The command
        then remains at its last value; ``target_active`` becomes false after
        ``lost_timeout_s`` so the caller can report a lost track.
        """
        now = time.monotonic() if now_s is None else now_s
        if self.last_update_s is None:
            dt = 0.02
        else:
            dt = max(0.001, min(0.25, now - self.last_update_s))
        self.last_update_s = now

        if target_cx is None or target_cy is None:
            active = (
                self.last_seen_s is not None
                and now - self.last_seen_s <= self.config.lost_timeout_s
            )
            return self.x_deg, self.y_deg, active

        self.last_seen_s = now
        error_x = target_cx - frame_width / 2.0
        error_y = target_cy - frame_height / 2.0
        if abs(error_x) < self.config.pixel_deadband:
            error_x = 0.0
        if abs(error_y) < self.config.pixel_deadband:
            error_y = 0.0

        correction_x = self._pixel_error_to_angle(
            error_x, frame_width, self.config.horizontal_fov_deg
        )
        correction_y = self._pixel_error_to_angle(
            error_y, frame_height, self.config.vertical_fov_deg
        )
        desired_x = (
            self.x_deg
            + self.config.x_direction * self.config.proportional_gain * correction_x
        )
        desired_y = (
            self.y_deg
            + self.config.y_direction * self.config.proportional_gain * correction_y
        )

        max_step = self.config.max_rate_deg_s * dt
        self.x_deg += min(max_step, max(-max_step, desired_x - self.x_deg))
        self.y_deg += min(max_step, max(-max_step, desired_y - self.y_deg))
        self.y_deg = self.safe_y(self.y_deg)
        return self.x_deg, self.y_deg, True
