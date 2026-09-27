"""Draw concise tracking diagnostics on the MaixCAM display frame."""

from typing import Optional

from maix import image

from domain import FrameResult
from estimation.camera_model import CameraModel


GREEN = image.Color.from_rgb(0, 255, 0)
YELLOW = image.Color.from_rgb(255, 220, 0)
RED = image.Color.from_rgb(255, 50, 50)
CYAN = image.Color.from_rgb(0, 220, 255)
WHITE = image.Color.from_rgb(255, 255, 255)


def draw_overlay(
    frame,
    result: FrameResult,
    camera_model: CameraModel,
    scale: int = 1,
) -> None:
    center_x = int(camera_model.cx)
    center_y = int(camera_model.cy)
    frame.draw_rect(center_x - 4, center_y - 4, 8, 8, color=CYAN)

    if result.detection is not None:
        box = result.detection.box
        frame.draw_rect(box.x, box.y, box.width, box.height, color=GREEN)
        frame.draw_string(
            box.x,
            max(0, box.y - 18),
            f"person {result.detection.confidence:.2f}",
            color=GREEN,
            scale=scale,
        )

    if result.track is not None and result.gimbal.valid:
        predicted_x, predicted_y = camera_model.world_to_pixel(
            result.track.azimuth_deg,
            result.track.elevation_deg,
            result.gimbal,
        )
        if 0 <= predicted_x < camera_model.width and 0 <= predicted_y < camera_model.height:
            frame.draw_rect(
                int(predicted_x) - 5,
                int(predicted_y) - 5,
                10,
                10,
                color=YELLOW,
            )

    mode_color = RED if result.mode.value == "FAULT" else WHITE
    frame.draw_string(
        4,
        4,
        f"Mode: {result.mode.value}",
        color=mode_color,
        scale=scale,
    )
    frame.draw_string(
        4,
        22,
        f"Gimbal: {result.gimbal.pan_deg:+.1f}, {result.gimbal.tilt_deg:+.1f} deg",
        color=WHITE,
        scale=scale,
    )
    if result.track is not None:
        frame.draw_string(
            4,
            40,
            (
                f"Target: {result.track.azimuth_deg:+.1f}, "
                f"{result.track.elevation_deg:+.1f} deg"
            ),
            color=WHITE,
            scale=scale,
        )
        frame.draw_string(
            4,
            58,
            (
                f"Rate: {result.track.azimuth_rate_deg_s:+.1f}, "
                f"{result.track.elevation_rate_deg_s:+.1f} deg/s"
            ),
            color=WHITE,
            scale=scale,
        )
    if result.status_message:
        frame.draw_string(
            4,
            camera_model.height - 20,
            result.status_message,
            color=mode_color,
            scale=scale,
        )
