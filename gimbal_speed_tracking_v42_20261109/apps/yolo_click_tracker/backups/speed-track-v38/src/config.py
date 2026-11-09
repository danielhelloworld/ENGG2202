"""YOLO app settings: these override the shared motor defaults above."""
CONFIG.update({
    "app_version": "speed-track-v38", "continuous_yaw": True,
    "overhead_tracking": True, "overhead_turn_timeout_s": 4.,
    "native_coast_max_deg": (4.,2.5),
    "tracking_motor_control": "speed",  # F32C mode 0; manual/HOME keep position control.
    "speed_mode_signs": (-1, -1),  # Raw positive RPM is opposite encoder angle on both installed axes.
    "speed_tracking_kp": (2.4, 2.6), "speed_tracking_kd": (.15, .15),
    "speed_tracking_margin_deg": 1.,
    "native_tracking": True, "native_tracking_rate": None,  # No separate low tracking cap; use motor_rpm.
    "native_tracking_accel": (3,3), "native_aim_gain": (1.,1.),
    "native_target_period_s": .02, "native_target_deadband_deg": .2,
    "native_target_window_deg": (45.,30.),
    "native_center_deadzone": (.008,.01), "native_center_exit_ratio": 1.25,
    "native_velocity_damping_s": .035, "native_damping_max_deg": 3.,
    "motor_loop_period_s": .005, "position_period_s": .02,
    "joystick_signs": (-1, -1),  # Both input axes reversed; physical zero unchanged.
    "auto_home_on_arm": False,  # ARM holds current pose; HOME is a separate action.
    "mechanical_zero_confirmed": True,  # Trust the existing saved zero; never redefine it.
    "startup_auto_home": True, "startup_home_rate": (150., 120.), "home_rate": (150., 120.),
    "hold_return_rate": (180., 150.),
    "home_jerk_deg_s3": (2500., 2000.), "home_response_s": .10,
    "home_tolerance_deg": 1., "home_settle_s": .25,
    "recovery_margin_deg": 2., "recovery_hard_margin_deg": 5.,
    "motion_violation_grace_s": .3,
    "model_kind": "YOLO11", "model_path": "/root/models/yolo11n.mud",
    "dual_buff": True, "camera_fps": 60, "camera_buffers": 3, "camera_rotate": True,
    "confidence": .25, "tracker_confidence": .5, "new_track_confidence": .5,
    "iou": .5, "class_ids": None,
    "inference_fps": 60., "preview_fps": 30., "vision_max_age": .45, "lost_timeout": 2.,
    "web_enabled": True, "web_preview_fps": 4.,
    "software_autotune_profile": "/root/gimbal_autotune/software_profile.json",
    "vision_disable_age_s": 2.,
    "feedback_timeout": .04, "feedback_retries": 3,
    "feedback_loss_grace_s": .5, "reference_max_age_s": .25,
    "limit_feedback_margin_deg": .8, "limit_hard_margin_deg": 3.,
    "reference_jump_slack_deg": 4.,
    "control_pause_age_s": .5, "watchdog_s": 1.2,
    "rotate_180": True, "jpeg_quality": 70,
    "deadzone": (.018, .02), "ema": (.5, .5),
    "tracking_near_kp": (1.2, 1.4), "tracking_far_kp": (2.4, 2.6), "tracking_blend_deg": (10., 8.),
    "tracking_settle_ratio": 1.6, "tracking_stop_horizon_s": .35,
    "camera_fov": (87., 49.), "visual_gain": (.7, .7), "step_cap": (7.5, 6.),
    "tracking_speed_multiplier": 1.,
    "motor_rpm": 40,  # Half v5 manual ceiling; actual speed is load dependent.
    "max_rate": (240., 180.), "manual_rate": (240., 180.), "tracking_rate": (75., 60.),
    "command_accel_deg_s2": (600., 500.), "frame_history": 32, "max_tracks": 64,
    "tracking_accel_deg_s2": (180., 150.), "home_accel_deg_s2": (450., 350.),
    "tracking_jerk_deg_s3": (1200., 1000.),
    "tracking_reverse_deg": 12., "tracking_reverse_persist_s": 1.2,
    "tracking_error_deg": 6.,
    "tracking_feedback_phase_s": .08, "tracking_feedback_margin_deg": 12.,
    "manual_position_lead_deg": (12., 9.), "tracking_position_lead_deg": (6., 5.),
    "home_position_lead_deg": (8., 6.),  # Outside manual adjustment keeps the shared 3 deg cap.
    "manual_jerk_deg_s3": (3000., 2500.), "manual_response_s": .12,
    "manual_accel_deg_s2": (400., 350.), "tracking_response_s": .10,
    "brake_accel_deg_s2": (400., 350.), "brake_jerk_deg_s3": (3000., 2500.),
    "brake_response_s": .08, "brake_timeout_s": 1.6,
    "hold_block_timeout_s": 8., "jump_confirm_window_s": .3,
    "adaptive_response": True, "adaptive_accel_min": (300., 250.),
    "adaptive_scale_bounds": (.8, 1.15), "adaptive_gain_slew_s": .3,
    "adaptive_lag_deg": .6, "adaptive_overshoot_deg": .2,
    "adaptive_min_progress_deg_s": .5, "adaptive_correction_kp": 1.5,
    "adaptive_correction_cap_deg_s": 3., "telemetry_period_s": .1, "telemetry_samples": 600,
    "prediction_enabled": True, "prediction_lead_s": .04, "prediction_latency_cap_s": .06,
    "prediction_velocity_alpha": .35, "prediction_min_samples": 3,
    "prediction_max_gap_s": .25, "prediction_jump_gate": .18,
    "prediction_max_velocity": .8, "prediction_min_velocity": .025,
    "prediction_max_shift": .02, "prediction_feedforward": 0.,
    "prediction_coast_s": .15, "prediction_coast_rate": (4., 3.), "pose_max_age_s": .2,
    "motion_guard": True, "motion_error_deg": 2.,
    "motion_grace_s": .6, "motion_window_s": 1.2,
    "motion_progress_deg": .2, "motion_reverse_deg": 2., "motion_reverse_persist_s": .4,
})


def native_tracking_rates(c):
    return ((float(c['motor_rpm']*6),)*2 if c['native_tracking_rate'] is None
            else tuple(c['native_tracking_rate']))


def decoded_position(c, axis, units, near):
    canonical=signed_mechanical(units/10.)*c['motor_signs'][axis]
    if axis==0 and c.get('continuous_yaw',False):
        return round(near+signed_mechanical(canonical-near),7)
    return round(canonical,7)


def validate_visual_config(c):
    validate_config(c)
    if c['tracking_motor_control'] not in ('speed', 'position'):
        raise ValueError('Invalid tracking motor control')
    if len(c['speed_mode_signs']) != 2 or any(type(v) is not int or v not in (-1, 1) for v in c['speed_mode_signs']):
        raise ValueError('Invalid speed_mode_signs')
    for key in ('speed_tracking_kp', 'speed_tracking_kd'):
        if len(c[key]) != 2 or any(not math.isfinite(v) or not 0 <= v <= 10 for v in c[key]):
            raise ValueError('Invalid ' + key)
    if not math.isfinite(c['speed_tracking_margin_deg']) or not .5 <= c['speed_tracking_margin_deg'] <= 5:
        raise ValueError('Invalid speed tracking limit margin')
    if type(c['continuous_yaw']) is not bool:raise ValueError('continuous_yaw must be boolean')
    if (type(c['overhead_tracking']) is not bool or not 2<=c['overhead_turn_timeout_s']<=5.
            or not .1<=c['confidence']<=c['tracker_confidence']<=c['new_track_confidence']<=.9
            or len(c['native_coast_max_deg'])!=2 or any(not 0<v<=5 for v in c['native_coast_max_deg'])):
        raise ValueError('Invalid overhead/recovery settings')
    if (type(c['native_tracking']) is not bool or len(c['native_tracking_accel'])!=2
            or any(type(v) is not int or not 1<=v<=5 for v in c['native_tracking_accel'])
            or len(native_tracking_rates(c))!=2 or any(not math.isfinite(v) or not 0<v<=c['motor_rpm']*6 for v in native_tracking_rates(c))
            or len(c['native_aim_gain'])!=2 or any(not math.isfinite(v) or not .5<=v<=1. for v in c['native_aim_gain'])
            or len(c['native_target_window_deg'])!=2 or any(not math.isfinite(v) or not 0<v<90 for v in c['native_target_window_deg'])
            or not .02<=c['native_target_period_s']<=.1 or not .1<=c['native_target_deadband_deg']<=.5):
        raise ValueError('Invalid native tracking profile')
    if (len(c['native_center_deadzone'])!=2 or any(not math.isfinite(v) or not .003<=v<=.02 for v in c['native_center_deadzone'])
            or not 1<=c['native_center_exit_ratio']<=1.5):
        raise ValueError('Invalid native center tolerance')
    if not 0<=c['native_velocity_damping_s']<=.08 or not 0<c['native_damping_max_deg']<=5:
        raise ValueError('Invalid native velocity damping')
    if (len(c['hold_return_rate'])!=2 or any(not math.isfinite(v) or not 0<v<=cap for v,cap in zip(c['hold_return_rate'],c['manual_rate']))
            or len(c['home_jerk_deg_s3'])!=2 or any(not math.isfinite(v) or not 0<v<=5000 for v in c['home_jerk_deg_s3'])
            or not math.isfinite(c['home_response_s']) or not .05<=c['home_response_s']<=.2):
        raise ValueError('Invalid fast return profile')
    if not .005 <= c["motor_loop_period_s"] <= .02:
        raise ValueError("Motor loop period must be 5..20ms")
    for key in ("manual_accel_deg_s2","brake_accel_deg_s2","brake_jerk_deg_s3"):
        if len(c[key])!=2 or any(not math.isfinite(v) or not 0<v<=5000 for v in c[key]):
            raise ValueError("Invalid bounded start/stop profile: "+key)
    for key in ("manual_accel_deg_s2","brake_accel_deg_s2"):
        if any(v>cap for v,cap in zip(c[key],c["command_accel_deg_s2"])):
            raise ValueError("Start/stop acceleration exceeds command ceiling")
    if not math.isfinite(c["tracking_response_s"]) or not .05<=c["tracking_response_s"]<=.2:
        raise ValueError("Invalid tracking response time")
    if not .08<=c["brake_response_s"]<=.2 or not .3<=c["brake_timeout_s"]<=2.:
        raise ValueError("Invalid braking time bounds")
    if (not math.isfinite(c['hold_block_timeout_s']) or not 2.<=c['hold_block_timeout_s']<=8.
            or not math.isfinite(c['jump_confirm_window_s']) or not .1<=c['jump_confirm_window_s']<=.4):
        raise ValueError('Invalid bounded position hold settings')
    if type(c["web_enabled"]) is not bool or not 0 < c["web_preview_fps"] <= c["preview_fps"]:
        raise ValueError("Invalid optional web preview settings")
    if type(c["camera_buffers"]) is not int or not 1 <= c["camera_buffers"] <= 3:
        raise ValueError("Camera buffers must be 1..3")
    if not 1 <= c["tracking_settle_ratio"] <= 2 or not .2 <= c["tracking_stop_horizon_s"] <= .6:
        raise ValueError("Invalid tracking settling/braking settings")
    if (len(c["tracking_jerk_deg_s3"]) != 2 or any(not math.isfinite(v) or not 0 < v <= 10000 for v in c["tracking_jerk_deg_s3"])
            or not 2 <= c["tracking_reverse_deg"] <= 12 or not .4 <= c["tracking_reverse_persist_s"] <= 1.2
            or not 2 <= c["tracking_error_deg"] <= 6
            or not 0 <= c["tracking_feedback_phase_s"] <= .08 or not 0 <= c["tracking_feedback_margin_deg"] <= 12):
        raise ValueError("Invalid bounded tracking transient settings")
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
    for key, hard in (("manual_position_lead_deg",(12.,9.)), ("tracking_position_lead_deg",(16.,12.)),
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

