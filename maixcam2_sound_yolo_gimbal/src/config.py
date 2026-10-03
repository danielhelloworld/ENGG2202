"""Edit these settings, then run build.py to refresh the standalone main.py."""

CONFIG = {
    "motor_enabled": False,       # First run observes only; never opens motor UART.
    "motor_calibrated": False,    # Confirm physical zero, polarity and safe limits.
    "motor_port": "/dev/ttyS3", "motor_pins": (("B2", "UART3_TX"), ("B3", "UART3_RX")),
    "motor_ids": (1, 2), "motor_signs": (1, 1),
    "motor_reference": "mechanical",  # Saved physical zero, rebuilt at every connection.
    "motor_raw_zero": (0.0, 0.0),     # Only used in explicit legacy raw mode.
    "limits": ((-150.0, 150.0), (-100.0, 100.0)),
    "motor_rpm": 5, "motor_acceleration": 30,
    "max_rate": (25.0, 15.0), "feedback_timeout": 0.15, "watchdog_s": 0.8,
    "feedback_retries": 2, "reference_stationary_deg": 0.2,
    "reference_mismatch_deg": 1.0,
    "reference_max_age_s": 0.5, "reference_jump_slack_deg": 2.0,
    "motion_guard": True, "motion_error_deg": 3.0,
    "motion_grace_s": 0.8, "motion_window_s": 1.5,
    "motion_progress_deg": 0.5, "motion_reverse_deg": 1.5,
    "mic_port": "/dev/ttyS1", "mic_baud": 2000000,
    "mic_rotation": 0, "mic_mirror": False,
    # Fixed-base calibration: (heatmap vx, vy, absolute logical yaw, pitch).
    # Collect real measurements. Empty table intentionally prevents sound motion.
    "sound_samples": [], "sound_calibrated": False,
    "sound_radius": 0.18, "sound_quality": 0.65, "sound_stable_s": 0.35,
    "sound_stable_deg": 5.0, "sound_fresh_s": 0.35,
    "model_kind": "YOLO11", "model_path": "/root/models/yolo11n.mud",
    "confidence": 0.5, "iou": 0.5, "class_ids": None,  # mainweb: all model classes
    "inference_fps": 10.0, "vision_max_age": 0.4,
    "lost_timeout": 2.0, "confirmation_frames": 3,
    "acquire_radius": 0.75, "ambiguity_margin": 0.10,
    "settle_tolerance": 2.0, "settle_s": 0.35, "turn_timeout": 15.0,
    "search_timeout": 6.0, "search_amplitude": 12.0, "search_rate": 4.0,
    # mainweb normalized dead zones, EMA and step caps. Gains below act on angles.
    "deadzone": (70.0 / 1920, 40.0 / 1080), "ema": (0.06, 0.10),
    "step_cap": (0.8, 1.0), "visual_gain": (0.7, 0.7),
    "camera_fov": (87.0, 49.0), "pixel_sign": (1, -1),
}


def validate_config(c):
    import math
    def finite(value):
        return isinstance(value, (int, float)) and math.isfinite(value)
    for key in ("motor_raw_zero", "camera_fov", "max_rate", "step_cap", "ema", "visual_gain", "deadzone"):
        if len(c[key]) != 2 or not all(finite(v) for v in c[key]):
            raise ValueError("Invalid " + key)
    for key in ("motor_signs", "pixel_sign"):
        if len(c[key]) != 2 or any(v not in (-1, 1) for v in c[key]):
            raise ValueError("Invalid " + key)
    if len(c["motor_ids"]) != 2 or len(set(c["motor_ids"])) != 2 or any(not isinstance(v, int) or not 1 <= v <= 255 for v in c["motor_ids"]):
        raise ValueError("Two distinct motor IDs required")
    if len(c["limits"]) != 2:
        raise ValueError("Two axis limits required")
    for lo, hi in c["limits"]:
        if not finite(lo) or not finite(hi) or not lo < hi:
            raise ValueError("Invalid axis limits")
    if c["limits"][1][0] < -100 or c["limits"][1][1] > 100:
        raise ValueError("Pitch cannot exceed -100..100 degrees")
    for key in ("feedback_timeout", "watchdog_s", "inference_fps", "vision_max_age", "lost_timeout",
                "sound_radius", "sound_stable_s", "sound_stable_deg", "sound_fresh_s", "settle_s",
                "settle_tolerance", "turn_timeout", "search_timeout", "search_amplitude", "search_rate",
                "reference_stationary_deg", "reference_mismatch_deg", "reference_max_age_s",
                "reference_jump_slack_deg", "motion_error_deg", "motion_grace_s",
                "motion_window_s", "motion_progress_deg", "motion_reverse_deg"):
        if not finite(c[key]) or c[key] <= 0:
            raise ValueError("Invalid " + key)
    if any(not 0 < v < 180 for v in c["camera_fov"]) or any(v <= 0 for v in c["max_rate"] + c["step_cap"]):
        raise ValueError("Invalid FOV/rate")
    if any(not 0 < v <= 1 for v in c["ema"]) or any(not 0 <= v < .5 for v in c["deadzone"]):
        raise ValueError("Invalid filter/deadzone")
    if not 1 <= c["motor_rpm"] <= 20 or not 1 <= c["motor_acceleration"] <= 100:
        raise ValueError("Conservative motor speed/acceleration range exceeded")
    if not isinstance(c["confirmation_frames"], int) or c["confirmation_frames"] < 1:
        raise ValueError("Invalid confirmation_frames")
    for sample in c["sound_samples"]:
        if len(sample) != 4 or not all(finite(v) for v in sample) or any(abs(v) > 1 for v in sample[:2]):
            raise ValueError("Invalid sound calibration row")
        if any(not lo <= v <= hi for v, (lo, hi) in zip(sample[2:], c["limits"])):
            raise ValueError("Sound sample outside motor limits")
    if c["sound_calibrated"] and len(c["sound_samples"]) < 3:
        raise ValueError("At least 3 measured calibration points required")
    if c["motor_enabled"] and (not c["motor_calibrated"] or not c["sound_calibrated"]):
        raise ValueError("Real motion requires motor and sound calibration")
    if c["motor_reference"] not in ("mechanical", "raw"):
        raise ValueError("Invalid motor_reference")
    if c["motor_enabled"] and c["motor_reference"] != "mechanical":
        raise ValueError("Real tracking requires mechanical reference for zero-loss protection")
    if not isinstance(c["feedback_retries"], int) or not 1 <= c["feedback_retries"] <= 3:
        raise ValueError("feedback_retries must be 1..3")
    if c["motor_reference"] == "mechanical" and any(lo <= -180 or hi >= 180 for lo, hi in c["limits"]):
        raise ValueError("This fixed-base tracker requires unique signed single-turn limits")
    if c["motor_port"] == c["mic_port"] or c["motor_port"] in ("/dev/ttyS0", "/dev/ttyS4"):
        raise ValueError("UART conflict: motor must not use mic/system/MaixVision port")
    if c["motor_port"] not in ("/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3"):
        raise ValueError("Unsupported motor UART; verify the adapter before adding a port")
    prefix = "UART" + c["motor_port"][-1]
    if len(c["motor_pins"]) != 2 or {f for _, f in c["motor_pins"]} != {prefix+"_TX", prefix+"_RX"}:
        raise ValueError("Motor UART port and pin functions disagree")
    for key in ("confidence", "iou", "sound_quality", "acquire_radius", "ambiguity_margin"):
        if not finite(c[key]) or not 0 <= c[key] <= 1:
            raise ValueError("Invalid " + key)
