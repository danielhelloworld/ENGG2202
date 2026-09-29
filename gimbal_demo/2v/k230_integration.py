"""Minimal K230-side integration pattern.

Create the UART with the pin mapping required by your K230 carrier board, then
call ``run_tracking`` with a detector callback. The callback returns
``(cx, cy, frame_width, frame_height)`` or ``None`` when no drone is detected.
No laser/output control is included.
"""

import time

from f32c_protocol import F32CGimbal
from tracking_controller import ImageTrackingController, TrackingConfig


def run_tracking(uart, detect_target, *, y_dead_zone_deg=120.0):
    """Run the motor-control half of a K230 detector/tracker pipeline.

    ``uart`` must be a configured 115200-8-N-1 ``machine.UART`` instance.
    ``detect_target`` is called repeatedly and returns target-center geometry.
    """
    bus = F32CGimbal(uart, frame_gap_s=0.002)
    controller = ImageTrackingController(
        TrackingConfig(y_dead_zone_width_deg=y_dead_zone_deg)
    )
    bus.start(speed_rpm=30, power_on_delay_s=1.5, require_feedback=True)
    try:
        while True:
            target = detect_target()
            if target is None:
                x_deg, y_deg, _ = controller.update(
                    target_cx=None,
                    target_cy=None,
                    frame_width=640,
                    frame_height=480,
                )
            else:
                cx, cy, width, height = target
                x_deg, y_deg, _ = controller.update(
                    target_cx=cx,
                    target_cy=cy,
                    frame_width=width,
                    frame_height=height,
                )
            bus.set_relative_angles(x_deg, y_deg)
            time.sleep(0.02)  # 50 Hz; manual recommends 100 Hz, max 200 Hz.
    finally:
        bus.stop()


# Typical board-specific bootstrap (adjust UART number/pinmux to your K230 board):
#
# from machine import UART
# uart = UART(UART.UART1, 115200, bits=8, parity=None, stop=1)
# run_tracking(uart, your_yolo_detector_center)

