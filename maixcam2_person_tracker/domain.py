"""Shared data types passed between application modules."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class TrackingMode(Enum):
    BOOT = "BOOT"
    ACQUIRE = "ACQUIRE"
    TRACK = "TRACK"
    COAST = "COAST"
    SEARCH = "SEARCH"
    FAULT = "FAULT"


@dataclass(frozen=True)
class BoundingBox:
    x: int
    y: int
    width: int
    height: int

    @property
    def center_x(self) -> float:
        return self.x + self.width * 0.5

    @property
    def center_y(self) -> float:
        return self.y + self.height * 0.5

    @property
    def area(self) -> int:
        return max(0, self.width) * max(0, self.height)


@dataclass(frozen=True)
class Detection:
    timestamp_ms: int
    box: BoundingBox
    class_id: int
    confidence: float


@dataclass(frozen=True)
class GimbalState:
    timestamp_ms: int
    pan_deg: float
    tilt_deg: float
    pan_valid: bool
    tilt_valid: bool

    @property
    def valid(self) -> bool:
        return self.pan_valid and self.tilt_valid


@dataclass(frozen=True)
class AngularObservation:
    timestamp_ms: int
    azimuth_deg: float
    elevation_deg: float
    confidence: float


@dataclass(frozen=True)
class TrackState:
    timestamp_ms: int
    azimuth_deg: float
    elevation_deg: float
    azimuth_rate_deg_s: float
    elevation_rate_deg_s: float
    azimuth_sigma_deg: float
    elevation_sigma_deg: float


@dataclass(frozen=True)
class GimbalCommand:
    timestamp_ms: int
    pan_target_deg: float
    tilt_target_deg: float
    enabled: bool


@dataclass(frozen=True)
class FrameResult:
    mode: TrackingMode
    detection: Optional[Detection]
    track: Optional[TrackState]
    gimbal: GimbalState
    command: Optional[GimbalCommand]
    status_message: str
