"""YOLO app settings: these override the shared motor defaults above."""
CONFIG.update({
    "app_version": "manual-half-soft-v6",
    "joystick_signs": (-1, -1),  # Both input axes reversed; physical zero unchanged.
    "auto_home_on_arm": False,  # ARM holds current pose; HOME is a separate action.
    "mechanical_zero_confirmed": True,  # Trust the existing saved zero; never redefine it.
    "startup_auto_home": True, "startup_home_rate": (108., 90.), "home_rate": (108., 90.),
    "recovery_margin_deg": 2., "recovery_hard_margin_deg": 5.,
    "motion_violation_grace_s": .3,
    "model_kind": "YOLO11", "model_path": "/root/models/yolo11n.mud",
    "dual_buff": True, "camera_fps": 60, "camera_rotate": True,
    "confidence": .5, "iou": .5, "class_ids": None,
    "inference_fps": 30., "preview_fps": 8., "vision_max_age": .45, "lost_timeout": 2.,
    "vision_disable_age_s": 2.,
    "feedback_timeout": .04, "feedback_retries": 3,
    "feedback_loss_grace_s": .5, "reference_max_age_s": .25,
    "limit_feedback_margin_deg": .8, "limit_hard_margin_deg": 3.,
    "reference_jump_slack_deg": 4.,
    "control_pause_age_s": .5, "watchdog_s": 1.2,
    "rotate_180": True, "jpeg_quality": 70,
    "deadzone": (.018, .02), "ema": (.28, .32),
    "tracking_near_kp": (2.8, 3.2), "tracking_far_kp": (6., 6.), "tracking_blend_deg": (8., 5.),
    "camera_fov": (87., 49.), "visual_gain": (.7, .7), "step_cap": (15., 12.),
    "tracking_speed_multiplier": 5.,  # Scale the requested correction, not only its ceiling.
    "motor_rpm": 40,  # Half v5 manual ceiling; actual speed is load dependent.
    "max_rate": (240., 180.), "manual_rate": (240., 180.), "tracking_rate": (150., 120.),
    "command_accel_deg_s2": (900., 750.), "frame_history": 12, "max_tracks": 64,
    "tracking_accel_deg_s2": (900., 750.), "home_accel_deg_s2": (540., 450.),
    "manual_position_lead_deg": (12., 9.), "tracking_position_lead_deg": (10., 8.),
    "home_position_lead_deg": (8., 6.),  # Outside manual adjustment keeps the shared 3 deg cap.
    "manual_jerk_deg_s3": (6000., 5000.), "manual_response_s": .10,
    "adaptive_response": True, "adaptive_accel_min": (500., 425.),
    "adaptive_scale_bounds": (.8, 1.15), "adaptive_gain_slew_s": .3,
    "adaptive_lag_deg": .6, "adaptive_overshoot_deg": .2,
    "adaptive_min_progress_deg_s": .5, "adaptive_correction_kp": 1.5,
    "adaptive_correction_cap_deg_s": 3., "telemetry_period_s": .1, "telemetry_samples": 600,
    "prediction_enabled": True, "prediction_lead_s": .12, "prediction_latency_cap_s": .12,
    "prediction_velocity_alpha": .35, "prediction_min_samples": 3,
    "prediction_max_gap_s": .25, "prediction_jump_gate": .18,
    "prediction_max_velocity": .8, "prediction_min_velocity": .025,
    "prediction_max_shift": .06, "prediction_feedforward": .35,
    "prediction_coast_s": .15, "prediction_coast_rate": (4., 3.), "pose_max_age_s": .2,
    "motion_guard": True, "motion_error_deg": 2.,
    "motion_grace_s": .6, "motion_window_s": 1.2,
    "motion_progress_deg": .2, "motion_reverse_deg": 2., "motion_reverse_persist_s": .4,
})


def validate_visual_config(c):
    validate_config(c)
    if not .05 <= c["feedback_loss_grace_s"] <= 1.:
        raise ValueError("Feedback loss grace must be 50..1000ms")
    if not 0 < c["feedback_timeout"] <= c["feedback_loss_grace_s"]/2:
        raise ValueError("Packet wait must allow several retries before feedback expiry")
    if not c["vision_max_age"] < c["vision_disable_age_s"] <= 2:
        raise ValueError("Vision hold must precede disable")
    if not 0 <= c["limit_feedback_margin_deg"] <= 1.5:
        raise ValueError("Invalid limit feedback margin")
    if not c["limit_feedback_margin_deg"] < c["limit_hard_margin_deg"] <= 3:
        raise ValueError("Invalid hard-limit feedback margin")
    if not .1 <= c["motion_violation_grace_s"] <= .6 or not .1 <= c["motion_reverse_persist_s"] <= .8:
        raise ValueError("Invalid motion anomaly debounce")
    if not .2 <= c["control_pause_age_s"] < c["watchdog_s"] <= 1.5:
        raise ValueError("Control hold must precede the hard watchdog")
    if type(c["adaptive_response"]) is not bool or not .5 <= c["adaptive_scale_bounds"][0] <= 1 <= c["adaptive_scale_bounds"][1] <= 1.25:
        raise ValueError("Invalid bounded adaptive response")
    if not .05 <= c["adaptive_gain_slew_s"] <= 1 or type(c["telemetry_samples"]) is not int or not 20 <= c["telemetry_samples"] <= 1200 or not .05 <= c["telemetry_period_s"] <= 1:
        raise ValueError("Invalid tuning/telemetry cadence")
    if type(c["auto_home_on_arm"]) is not bool or c["motion_guard"] is not True:
        raise ValueError("Auto-home must be boolean; motion protection must remain enabled")
    if type(c["startup_auto_home"]) is not bool or c["auto_home_on_arm"]:
        raise ValueError("ARM and HOME must remain separate; auto_home_on_arm=False")
    if len(c["startup_home_rate"]) != 2 or any(not math.isfinite(v) or not 0 < v <= c["motor_rpm"]*6 for v in c["startup_home_rate"]):
        raise ValueError("Recovery home rate must stay within the motor speed ceiling")
    if type(c["dual_buff"]) is not bool or type(c["camera_rotate"]) is not bool or c["camera_fps"] not in (30, 60):
        raise ValueError("Invalid camera/buffer settings")
    if not 0 <= c["recovery_margin_deg"] <= 2. or not c["recovery_margin_deg"] < c["recovery_hard_margin_deg"] <= 5.:
        raise ValueError("Invalid recovery feedback margin")
    if c["model_kind"] not in ("YOLO11", "YOLOv8") or not c["model_path"].endswith(".mud"):
        raise ValueError("Use a MaixCAM2 YOLO11/YOLOv8 .mud model")
    for key in ("inference_fps", "preview_fps", "vision_max_age", "lost_timeout", "pose_max_age_s",
                "prediction_lead_s", "prediction_latency_cap_s", "prediction_velocity_alpha",
                "prediction_max_gap_s", "prediction_jump_gate", "prediction_max_velocity",
                "prediction_min_velocity", "prediction_max_shift", "prediction_coast_s"):
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError("Invalid " + key)
    for key in ("deadzone", "ema", "camera_fov", "visual_gain", "step_cap", "tracking_rate", "manual_rate", "prediction_coast_rate", "tracking_near_kp", "tracking_far_kp", "tracking_blend_deg", "adaptive_accel_min", "manual_jerk_deg_s3", "tracking_accel_deg_s2", "home_accel_deg_s2", "manual_position_lead_deg", "tracking_position_lead_deg", "home_position_lead_deg"):
        if len(c[key]) != 2 or any(not math.isfinite(v) or v <= 0 for v in c[key]):
            raise ValueError("Invalid " + key)
    if any(lo > hi for lo,hi in zip(c["adaptive_accel_min"], c["command_accel_deg_s2"])):
        raise ValueError("Adaptive acceleration exceeds its fixed ceiling")
    if not math.isfinite(c["manual_response_s"]) or not .05 <= c["manual_response_s"] <= .2:
        raise ValueError("Manual response must be 50..200ms")
    if any(v > 40000 for v in c["manual_jerk_deg_s3"]):
        raise ValueError("Manual jerk exceeds its fixed ceiling")
    if not math.isfinite(c["tracking_speed_multiplier"]) or not 1 <= c["tracking_speed_multiplier"] <= 5:
        raise ValueError("Tracking correction multiplier must be 1..5")
    for key, hard in (("manual_position_lead_deg",(12.,9.)), ("tracking_position_lead_deg",(10.,8.)),
                      ("home_position_lead_deg",(8.,6.))):
        if any(v > hi for v,hi in zip(c[key],hard)):
            raise ValueError("Invalid bounded position horizon: "+key)
    for key in ("tracking_accel_deg_s2","home_accel_deg_s2"):
        if any(v > hi for v,hi in zip(c[key],c["command_accel_deg_s2"])):
            raise ValueError("Profile acceleration exceeds command ceiling: "+key)
    for key in ("adaptive_lag_deg", "adaptive_overshoot_deg", "adaptive_min_progress_deg_s", "adaptive_correction_kp", "adaptive_correction_cap_deg_s"):
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError("Invalid " + key)
    if any(v >= .5 for v in c["deadzone"]) or any(v > 1 for v in c["ema"]):
        raise ValueError("Invalid deadzone/EMA")
    if any(v > rate for v, rate in zip(c["tracking_rate"], c["max_rate"])):
        raise ValueError("Tracking cannot exceed manual rate caps")
    if any(v > cap for v,cap in zip(c["manual_rate"], c["max_rate"])):
        raise ValueError("Manual rate exceeds motor rate ceiling")
    if (type(c["prediction_enabled"]) is not bool or type(c["prediction_min_samples"]) is not int
            or not 3 <= c["prediction_min_samples"] <= 10 or c["prediction_velocity_alpha"] > 1
            or c["prediction_coast_s"] > .2 or c["prediction_lead_s"] > .2
            or c["prediction_max_shift"] > .1 or not 0 <= c["prediction_feedforward"] <= 1):
        raise ValueError("Invalid bounded prediction settings")

