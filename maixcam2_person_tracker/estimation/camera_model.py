"""Pinhole camera conversion between image pixels and world bearing angles."""

import math
from typing import Tuple

from config import CameraConfig
from domain import GimbalState


class CameraModel:
    def __init__(self, width: int, height: int, config: CameraConfig) -> None:
        self.width = width
        self.height = height
        self.config = config
        self.cx = width * 0.5
        self.cy = height * 0.5
        self.fx = self.cx / math.tan(math.radians(config.horizontal_fov_deg) * 0.5)
        self.fy = self.cy / math.tan(math.radians(config.vertical_fov_deg) * 0.5)

    def pixel_to_world(
        self, pixel_x: float, pixel_y: float, gimbal: GimbalState
    ) -> Tuple[float, float]:
        pan_ray = math.degrees(math.atan((pixel_x - self.cx) / self.fx))
        tilt_ray = math.degrees(math.atan((pixel_y - self.cy) / self.fy))
        azimuth = gimbal.pan_deg + self.config.pan_pixel_sign * pan_ray
        elevation = gimbal.tilt_deg + self.config.tilt_pixel_sign * tilt_ray
        return azimuth, elevation

    def world_to_pixel(
        self, azimuth_deg: float, elevation_deg: float, gimbal: GimbalState
    ) -> Tuple[float, float]:
        pan_ray = (azimuth_deg - gimbal.pan_deg) / self.config.pan_pixel_sign
        tilt_ray = (elevation_deg - gimbal.tilt_deg) / self.config.tilt_pixel_sign
        pixel_x = self.cx + self.fx * math.tan(math.radians(pan_ray))
        pixel_y = self.cy + self.fy * math.tan(math.radians(tilt_ray))
        return pixel_x, pixel_y
