"""Configuration included at the top of the generated standalone main.py."""
import os
import time
import logging
import numpy as np
import cv2

CONFIG = {
    # Leave empty to select an installed YOLO11n/s or YOLOv8n model.
    "model_path": "",
    "model_type": "YOLO11",  # Used only when model_path is explicitly set.
    "confidence": 0.50,
    "iou": 0.45,
    "max_objects": 12,
    "width": 640,
    "height": 480,
    "camera_fps": 60,  # Requested sensor rate; actual rate depends on sensor/mode.
    "loop_fps": 0,  # 0 removes the application display/composition FPS cap.
    "detector_mode": "inline",  # Compatibility default; "process" is experimental on board.
    "inference_fps": 10,  # Skip inference between scheduled frames; 0 means every opportunity.
    "camera_read_timeout_ms": 200,
    "camera_stall_seconds": 10,
    "diagnostic_trace": True,  # First three loop attempts + stalled Python stacks.
    "max_detection_age": 0.35,
    "max_detection_thermal_delta": 0.15,
    "alpha": 0.40,
    "thermal_flip_x": True,
    "thermal_flip_y": True,
    # Smaller INITIAL footprint; not measured lens FOV. ALIGN tunes each distance.
    "thermal_rect": [0.25, 0.25, 0.5, 0.5],
    "manual_adjustments": {},  # Per-distance scale/shift; exported on SAVE.
    # Installation metadata only; centimetres cannot be converted to pixels without optics/depth.
    "thermal_offset_m": [0.05, 0.05, 0.0],  # right, down, forward relative to RGB
    "calibration_distances_m": [0.5, 1.0, 2.0, 3.0, 5.0],
    "calibration_distance_m": 1.0,
    "calibration_profiles": {},  # Paste exported profiles here for MaixVision single-file reuse.
    "alignment_path": "/root/thermal_yolo_fusion/alignment.json",  # Survives MaixVision temp uploads.
    "calibration_ransac_px": 4.0,
    "calibration_max_error_px": 6.0,  # RGB model-input pixels, not screen pixels
    "max_thermal_age": 0.60,
    "max_pair_delta": 0.25,
    "hot_min_c": 25.0,  # Candidate must also exceed the frame median by hot_delta_c.
    "hot_delta_c": 3.0,
    "hot_min_area_px": 5,  # Original 160x120 thermal pixels, not enlarged display pixels.
    "hot_max_candidates": 12,
    "hot_track_gate_px": 14.0,
    "hot_track_misses": 2,
    "hot_track_timeout_s": 0.8,
    "uart_port": "/dev/ttyS2",
    "configure_uart_pins": True,
    "reset_pin": "A9",
    "reset_gpio": "GPIOA9",
    "reset_active": 0,
    "skip_frames": 10,
    "thermal_retry_seconds": 5.0,
    "rtsp_enabled": True,
    "rtsp_host": "0.0.0.0",
    "rtsp_port": 8554,
    "stream_fps": 30,  # JPEG encode/output ceiling, independent of screen refresh.
    "jpeg_quality": 70,
    "max_clients": 2,
}

LOG = logging.getLogger("thermal_yolo")
THERMAL_W, THERMAL_H = 160, 120
PIXELS = THERMAL_W * THERMAL_H
BODY_SIZE = PIXELS + 30
WIRE_SIZE = BODY_SIZE + 1
