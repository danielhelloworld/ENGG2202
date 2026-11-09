# Generated standalone YOLO click tracker. Edit CONFIG near the top.
# Default startup automatically enables and homes from the current pose.

# Source: motor_joystick\src\config.py
"""Motor-only settings. Build main.py after editing maintained sources."""
import math

CONFIG = {
    "motor_enabled": True,  # Open UART, initially DISABLED; ARM is a screen action.
    "motor_port": "/dev/ttyS3", "motor_pins": (("B2", "UART3_TX"), ("B3", "UART3_RX")),
    "motor_ids": (1, 2), "motor_signs": (1, 1),
    "joystick_signs": (-1, 1),  # Camera upside down: reverse LEFT/RIGHT input only.
    "motor_reference": "mechanical_only", "motor_raw_zero": (0.0, 0.0),
    "mechanical_zero_confirmed": False,  # Or tap ZERO OK after checking saved physical zero.
    "limits": ((-150.0, 150.0), (-100.0, 30.0)),
    "motor_rpm": 5, "motor_acceleration": 10,
    "position_mode": 4,  # Single-turn direct; software ramps the streamed target.
    "command_accel_deg_s2": (80.0, 60.0), "position_period_s": 0.04,
    "max_rate": (24.0, 16.0), "home_rate": (10.0, 8.0), "manual_lead_deg": 3.0,
    "feedback_timeout": 0.12, "feedback_retries": 2, "watchdog_s": 0.8,
    "reference_stationary_deg": 0.2, "reference_mismatch_deg": 1.0,
    "reference_max_age_s": 0.5, "reference_jump_slack_deg": 2.0,
    "motion_guard": True, "motion_error_deg": 1.0,
    "motion_grace_s": 0.8, "motion_window_s": 1.5,
    "motion_progress_deg": 0.3, "motion_reverse_deg": 1.0,
    "home_tolerance_deg": 0.5, "home_settle_s": 0.6, "home_timeout_s": 45.0,
    "clear_tolerance_deg": 0.2, "joystick_deadzone": 0.15, "max_touch_hold_s": 8.0,
}


def clip(value, lo, hi):
    return min(hi, max(lo, value))


def validate_config(c):
    if c["motor_enabled"] is not True or c["motor_reference"] not in ("mechanical", "mechanical_only"):
        raise ValueError("This demo requires disabled-start hardware with mechanical reference")
    if c["motor_port"] != "/dev/ttyS3" or tuple(c["motor_pins"]) != (("B2", "UART3_TX"), ("B3", "UART3_RX")):
        raise ValueError("Motor demo is isolated on UART3 B2/B3")
    if len(c["motor_ids"]) != 2 or len(set(c["motor_ids"])) != 2 or any(type(v) is not int or not 1 <= v <= 255 for v in c["motor_ids"]):
        raise ValueError("Two distinct motor IDs required")
    if len(c["motor_signs"]) != 2 or any(v not in (-1, 1) for v in c["motor_signs"]):
        raise ValueError("Invalid motor_signs")
    if len(c["joystick_signs"]) != 2 or any(v not in (-1, 1) for v in c["joystick_signs"]):
        raise ValueError("Invalid joystick_signs")
    if c["position_mode"] not in (2, 4):
        raise ValueError("Single-turn position mode must be 2 or 4")
    if not 0.02 <= c["position_period_s"] <= 0.1:
        raise ValueError("Position period must be 0.02..0.1 seconds")
    if len(c["command_accel_deg_s2"]) != 2 or any(not math.isfinite(v) or not 0 < v <= 2000 for v in c["command_accel_deg_s2"]):
        raise ValueError("Invalid software command acceleration")
    if len(c["limits"]) != 2:
        raise ValueError("Two limits required")
    for axis, (lo, hi) in enumerate(c["limits"]):
        if axis == 0 and c.get("continuous_yaw", False) and (lo, hi) == (float("-inf"), float("inf")):
            continue
        if not all(math.isfinite(v) for v in (lo, hi)) or not -180 < lo < 0 < hi < 180:
            raise ValueError("Limits must include zero and stay inside signed single-turn range")
    if c["limits"][1][0] < -100 or c["limits"][1][1] > 30:
        raise ValueError("Pitch must stay within -100..30")
    if not 1 <= c["motor_rpm"] <= 80 or not 1 <= c["motor_acceleration"] <= 30:
        raise ValueError("Demo RPM/acceleration cap exceeded")
    for key in ("max_rate", "home_rate"):
        if len(c[key]) != 2 or any(not math.isfinite(v) or not 0 < v <= c["motor_rpm"] * 6 for v in c[key]):
            raise ValueError("Invalid " + key)
    for key in ("feedback_timeout", "watchdog_s", "reference_stationary_deg", "reference_mismatch_deg",
                "reference_max_age_s", "reference_jump_slack_deg", "motion_error_deg", "motion_grace_s",
                "motion_window_s", "motion_progress_deg", "motion_reverse_deg", "home_tolerance_deg",
                "home_settle_s", "home_timeout_s", "clear_tolerance_deg", "manual_lead_deg", "max_touch_hold_s"):
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError("Invalid " + key)
    if not 0 <= c["joystick_deadzone"] < 1 or not 1 <= c["feedback_retries"] <= 3:
        raise ValueError("Invalid joystick/feedback setting")


# Source: yolo_click_tracker\src\config.py
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



# Source: motor_joystick\vendor\protocol.py
"""Frame encoding and feedback parsing for WHEELTEC F32C motors."""

from typing import List, Tuple


FRAME_HEADER = 0x7A
FRAME_TAIL = 0x7B

FUNCTION_MODE = 0x00
FUNCTION_SPEED = 0x01
FUNCTION_MULTI_TURN_POSITION = 0x02
FUNCTION_DISABLE = 0x05
FUNCTION_ENABLE = 0x06
FUNCTION_ACCELERATION = 0x07
FUNCTION_SAVE = 0x08
FUNCTION_MECHANICAL_ZERO = 0x0A
FUNCTION_FEEDBACK = 0x0E

MODE_SPEED = 0x0000
MODE_MULTI_TURN_PLANNED = 0x0001
MODE_MULTI_TURN_DIRECT = 0x0003

FEEDBACK_SPEED = 0x00
FEEDBACK_TOTAL_ANGLE = 0x01
FEEDBACK_MECHANICAL_ANGLE = 0x02
FEEDBACK_ACCELERATION = 0x03
FEEDBACK_BUS_VOLTAGE = 0x04


def bcc_xor(data: bytes) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum


def _frame(motor_id: int, function: int, payload: bytes = b"") -> bytes:
    body = bytes((FRAME_HEADER, motor_id, function)) + payload
    return body + bytes((bcc_xor(body), FRAME_TAIL))


def enable(motor_id: int) -> bytes:
    return _frame(motor_id, FUNCTION_ENABLE)


def disable(motor_id: int) -> bytes:
    return _frame(motor_id, FUNCTION_DISABLE)


def set_mechanical_zero_frame(motor_id: int) -> bytes:
    return _frame(motor_id, FUNCTION_MECHANICAL_ZERO)


def save_parameters_frame(motor_id: int) -> bytes:
    return _frame(motor_id, FUNCTION_SAVE)


def select_mode(motor_id: int, mode: int) -> bytes:
    return _frame(motor_id, FUNCTION_MODE, int(mode).to_bytes(2, "big"))


def set_speed(motor_id: int, rpm: int) -> bytes:
    rpm = max(-1000, min(1000, int(rpm)))
    return _frame(
        motor_id,
        FUNCTION_SPEED,
        rpm.to_bytes(2, "big", signed=True),
    )


def set_acceleration(motor_id: int, rpm_per_s: int) -> bytes:
    value = max(0, min(65535, int(rpm_per_s)))
    return _frame(motor_id, FUNCTION_ACCELERATION, value.to_bytes(2, "big"))


def set_multi_turn_position(motor_id: int, angle_deg: float) -> bytes:
    scaled_angle = int(round(angle_deg * 10.0))
    scaled_angle = max(-(2 ** 31), min(2 ** 31 - 1, scaled_angle))
    return _frame(
        motor_id,
        FUNCTION_MULTI_TURN_POSITION,
        scaled_angle.to_bytes(4, "big", signed=True),
    )


def request_feedback(motor_id: int, feedback_type: int) -> bytes:
    return _frame(motor_id, FUNCTION_FEEDBACK, bytes((feedback_type,)))


class FeedbackParser:
    """Incrementally parse fixed-length F32C feedback frames from a byte stream."""

    RESPONSE_LENGTH = 9

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> List[Tuple[int, int, int]]:
        self.buffer.extend(data)
        decoded: List[Tuple[int, int, int]] = []

        while self.buffer:
            try:
                header_index = self.buffer.index(FRAME_HEADER)
            except ValueError:
                self.buffer.clear()
                break

            if header_index > 0:
                del self.buffer[:header_index]
            if len(self.buffer) < self.RESPONSE_LENGTH:
                break

            candidate = bytes(self.buffer[: self.RESPONSE_LENGTH])
            if candidate[-1] != FRAME_TAIL:
                del self.buffer[0]
                continue
            if bcc_xor(candidate[:7]) != candidate[7]:
                del self.buffer[0]
                continue

            motor_id = candidate[1]
            feedback_type = candidate[2]
            raw_value = int.from_bytes(candidate[3:7], "big", signed=True)
            decoded.append((motor_id, feedback_type, raw_value))
            del self.buffer[: self.RESPONSE_LENGTH]

        return decoded


# Source: motor_joystick\vendor\motor_base.py
"""One UART owner, feedback before enable, bounded mailbox, latched watchdog."""
import threading
import time


def signed_mechanical(angle):
    return (angle + 180.0) % 360.0 - 180.0


class MotionGuard:
    """Position-progress heuristic, not torque or collision sensing."""
    def __init__(self, c):
        self.c = c
        self.windows = [None, None]

    def reset(self):
        self.windows = [None, None]

    def update(self, now, actual, commanded):
        for axis, (position, goal) in enumerate(zip(actual, commanded)):
            error = goal - position
            if abs(error) < self.c["motion_error_deg"]:
                self.windows[axis] = None
                continue
            direction = 1 if error > 0 else -1
            window = self.windows[axis]
            if window is None or window[2] != direction:
                self.windows[axis] = (now, position, direction)
                continue
            elapsed, progress = now-window[0], (position-window[1])*direction
            if elapsed < self.c["motion_grace_s"]:
                continue
            if progress <= -self.c["motion_reverse_deg"]:
                raise RuntimeError("Axis %d moving opposite commanded direction" % axis)
            if elapsed >= self.c["motion_grace_s"] + self.c["motion_window_s"]:
                if progress < self.c["motion_progress_deg"]:
                    raise RuntimeError("Axis %d motion stalled (position heuristic)" % axis)
                self.windows[axis] = (now, position, direction)


class MotorService:
    def __init__(self, c, transport_factory=None):
        self.c = c
        self.factory = transport_factory
        self.serial = None
        self.parser = FeedbackParser()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.command_target = None
        self.command_time = 0.0
        self.angles = (0.0, 0.0)
        self.feedback_time = 0.0
        self.ready = False
        self.fault = None
        self.sent_target = None
        self.last_send = None
        self.reference_zero = tuple(c["motor_raw_zero"])
        self.mechanical = (None, None)
        self.armed = False
        self.arm_requested = False
        self.guard = MotionGuard(c)
        self.reference_valid = False
        self.reference_fault = None
        self.captured_reference = None
        self.raw_feedback = (None, None)
        self.total_samples = [None, None]

    def start(self):
        validate_config(self.c)
        if self.thread is not None:
            raise RuntimeError("MotorService already started")
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, target):
        if len(target) != 2 or any(not math.isfinite(v) or not lo <= v <= hi for v, (lo, hi) in zip(target, self.c["limits"])):
            raise ValueError("Motor target outside limits")
        with self.lock:
            if self.fault is not None:
                raise RuntimeError(self.fault)
            if self.c["motor_enabled"] and not self.reference_valid:
                raise RuntimeError("ZERO_REFERENCE_LOST: no valid reference")
            self.command_target = tuple(target)
            self.command_time = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {"angles": self.angles, "ready": self.ready, "fault": self.fault,
                    "age": time.monotonic() - self.feedback_time,
                    "simulated": not self.c["motor_enabled"], "armed": self.armed,
                    "mechanical_angles": self.mechanical, "reference_zero": self.reference_zero,
                    "reference_valid": self.reference_valid, "reference_fault": self.reference_fault,
                    "raw_feedback": self.raw_feedback}

    def request_arm(self, armed):
        with self.lock:
            if armed and (self.fault or not self.ready or not self.reference_valid):
                raise RuntimeError(self.fault or "Motor not ready")
            self.arm_requested = bool(armed)

    def _open(self):
        if self.factory is not None:
            return self.factory()
        from maix import pinmap, err
        from maix.peripheral import uart
        for pin, function in self.c["motor_pins"]:
            if function not in [str(v) for v in pinmap.get_pin_functions(pin)]:
                raise RuntimeError("Unavailable motor pin function: " + pin + " " + function)
            err.check_raise(pinmap.set_pin_function(pin, function), "motor pinmap " + pin)
        return uart.UART(port=self.c["motor_port"], baudrate=115200)

    def _write(self, data):
        time.sleep(.002)
        if self.stop_event.is_set() and data[2] != FUNCTION_DISABLE:
            raise RuntimeError("Motor stop requested")
        # Final transmission gate: no enable or position frame with a lost/stale zero.
        if data[2] in (FUNCTION_ENABLE, FUNCTION_MULTI_TURN_POSITION):
            self._require_reference(fresh=True)
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete motor UART write")

    def _read(self):
        try:
            return self.serial.read(512, timeout=2)
        except TypeError:
            return self.serial.read(512, 2)

    def _feedback_once(self, motor_id, kind):
        # Drain queued bytes before sending a new query. Do not reuse old feedback.
        for _ in range(8):
            if not self._read():
                break
        else:
            raise RuntimeError("Motor UART input never drained")
        self.parser = FeedbackParser()
        self._write(request_feedback(motor_id, kind))
        deadline = time.monotonic() + self.c["feedback_timeout"]
        received = bytearray()
        while time.monotonic() < deadline and not self.stop_event.is_set():
            data = self._read()
            received.extend(data or b"")
            for mid, response_kind, raw in self.parser.feed(bytes(data or b"")):
                if mid == motor_id and response_kind == kind:
                    # Type 0 is integer RPM; angle types use 0.1-degree units.
                    return raw if kind == FEEDBACK_SPEED else raw / 10.0
            time.sleep(.001)
        raise RuntimeError("No valid feedback from motor %d type=%d RX=%s" % (motor_id, kind, bytes(received[-64:]).hex()))

    def _feedback(self, motor_id, kind=FEEDBACK_TOTAL_ANGLE):
        attempts = 1 if self.armed else self.c["feedback_retries"]
        for attempt in range(attempts):
            try:
                return self._feedback_once(motor_id, kind)
            except RuntimeError:
                if self.stop_event.is_set() or attempt + 1 >= attempts:
                    raise

    def _latch_fault(self, reason, reference_lost=False):
        with self.lock:
            self.fault = self.fault or reason
            self.ready = self.armed = self.arm_requested = self.reference_valid = False
            self.command_target = self.sent_target = None
            if reference_lost:
                self.reference_fault = self.reference_fault or reason
        self._disable_all()  # Attempt each axis even if the first UART write fails.

    def _lose_reference(self, reason):
        message = "ZERO_REFERENCE_LOST: " + reason
        self._latch_fault(message, reference_lost=True)
        raise RuntimeError(message)

    def _require_reference(self, fresh=False):
        if self.fault or not self.reference_valid:
            self._lose_reference("reference unavailable; restart and verify physical zero")
        if (not isinstance(self.reference_zero, (tuple, list)) or len(self.reference_zero) != 2
                or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in self.reference_zero)):
            self._lose_reference("reference missing or nonfinite")
        if self.c["motor_enabled"] and tuple(self.reference_zero) != self.captured_reference:
            self._lose_reference("captured zero reference changed unexpectedly")
        if fresh and time.monotonic()-self.feedback_time > self.c["reference_max_age_s"]:
            self._lose_reference("reference feedback expired")

    def _check_total_continuity(self, axis, angle):
        now = time.monotonic()
        previous = self.total_samples[axis]
        if previous is not None:
            dt = max(0.0, now-previous[0])
            # Use physical RPM cap, not modulo 360, so whole-turn resets are caught.
            bound = self.c["motor_rpm"] * 6.0 * dt + self.c["reference_jump_slack_deg"]
            if abs(angle-previous[1]) > bound:
                self._lose_reference("total angle discontinuity/reference mismatch on axis %d" % axis)
        self.total_samples[axis] = (now, angle)

    def _capture_reference(self):
        self.reference_valid = False
        zeros = []
        for mid in self.c["motor_ids"]:
            first = self._feedback(mid)
            mechanical = signed_mechanical(self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE))
            last = self._feedback(mid)
            if abs(last-first) > self.c["reference_stationary_deg"]:
                raise RuntimeError("Motor moved while reading fixed reference; support load and retry")
            zeros.append(first-mechanical)
        with self.lock:
            self.reference_zero = tuple(zeros)
            self.captured_reference = tuple(zeros)
            self.reference_valid = True
        self.total_samples = [None, None]

    def _measure(self):
        try:
            self._require_reference(fresh=self.armed)
            return self._read_measurement()
        except Exception as exc:
            if self.stop_event.is_set() or self.reference_fault:
                raise
            self._lose_reference(str(exc))

    def _read_measurement(self):
        raw, mechanical = [], []
        for axis, (mid, zero) in enumerate(zip(self.c["motor_ids"], self.reference_zero)):
            first = self._feedback(mid)
            self._check_total_continuity(axis, first)
            if self.c["motor_reference"] == "mechanical":
                mech = signed_mechanical(self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE))
                last = self._feedback(mid)
                self._check_total_continuity(axis, last)
                expected = (first+last)/2-zero
                if abs(signed_mechanical(expected-mech)) > self.c["reference_mismatch_deg"]:
                    raise RuntimeError("Mechanical/total reference mismatch; reconnect after motor restart")
                mechanical.append(mech)
                raw.append(last)
            else:
                raw.append(first)
                mechanical.append(None)
        # Normalize arithmetic roundoff (e.g. 100.00000000000001 at +100).
        # Feedback resolution remains 0.1 degree; this does not expand limits.
        angles = tuple(round((r-z)*s, 7) for r, z, s in zip(raw, self.reference_zero, self.c["motor_signs"]))
        with self.lock:
            self.angles, self.feedback_time, self.mechanical = angles, min(sample[0] for sample in self.total_samples), tuple(mechanical)
            self.raw_feedback = tuple(raw)
        if any(not lo <= v <= hi for v, (lo, hi) in zip(angles, self.c["limits"])):
            raise RuntimeError("Measured position outside calibrated limits")
        return raw, angles

    def _read_diagnostics(self):
        # Deliberately does not rebuild zero, mark it valid, or refresh control age.
        totals, mechanical = list(self.raw_feedback), list(self.mechanical)
        for axis, mid in enumerate(self.c["motor_ids"]):
            try:
                totals[axis] = self._feedback(mid)
                mechanical[axis] = signed_mechanical(self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE))
            except Exception:
                pass
        with self.lock:
            self.raw_feedback, self.mechanical = tuple(totals), tuple(mechanical)

    def _disable_all(self):
        if self.serial is None:
            return []
        errors = []
        for mid in self.c["motor_ids"]:
            try:
                self._write(disable(mid))
            except Exception as exc:
                print("MOTOR_DISABLE_FAILED", mid, str(exc))
                errors.append("motor %d: %s" % (mid, exc))
        return errors

    def _initialize(self):
        for mid in self.c["motor_ids"]:
            self._write(disable(mid))
        if self.c["motor_reference"] == "mechanical":
            self._capture_reference()
        else:
            self.reference_valid = True
        _, angles = self._measure()
        return angles  # Connect disabled and continue reading feedback.

    def _arm_current(self):
        raw, angles = self._measure()  # Both axes must answer BEFORE any enable.
        for mid, angle in zip(self.c["motor_ids"], raw):
            self._write(select_mode(mid, MODE_MULTI_TURN_PLANNED))
            self._write(set_speed(mid, self.c["motor_rpm"]))
            self._write(set_acceleration(mid, self.c["motor_acceleration"]))
            self._write(set_multi_turn_position(mid, angle))
        for mid in self.c["motor_ids"]:
            self._write(enable(mid))
        return angles

    def calibrate_mechanical_zero(self, motor_id, confirm=False):
        """Explicit maintenance API. Never called by normal tracking startup."""
        if not confirm:
            raise ValueError("Confirm supported, stationary physical zero before opening UART")
        if self.thread is not None or motor_id not in self.c["motor_ids"]:
            raise RuntimeError("Calibration requires a separate stopped service and a valid axis")
        validate_config(self.c)
        try:
            self.serial = self._open()
            for mid in self.c["motor_ids"]:
                self._write(disable(mid))
            totals = []
            for _ in range(3):
                totals.append(self._feedback(motor_id))
                if self._feedback(motor_id, FEEDBACK_SPEED) != 0:
                    raise RuntimeError("Motor moving; mechanical zero not changed")
                time.sleep(.1)
            if max(totals)-min(totals) > .2:
                raise RuntimeError("Motor not stationary; mechanical zero not changed")
            self._write(set_mechanical_zero_frame(motor_id))
            time.sleep(.2)
            for _ in range(3):
                if abs(signed_mechanical(self._feedback(motor_id, FEEDBACK_MECHANICAL_ANGLE))) > .5:
                    raise RuntimeError("Mechanical zero readback failed; save not sent")
                if abs(self._feedback(motor_id)-totals[-1]) > .2:
                    raise RuntimeError("Motor moved during calibration; save not sent")
                time.sleep(.1)
            self._write(save_parameters_frame(motor_id))
            print("ZERO_SAVE_SENT: live zero verified; power-cycle persistence still requires hardware check")
        finally:
            self._disable_all()
            if self.serial is not None:
                self.serial.close()
                self.serial = None

    def _run(self):
        try:
            if self.c["motor_enabled"]:
                self.serial = self._open()
                self.sent_target = self._initialize()
            else:
                self.sent_target = tuple(clip(0.0, *lim) for lim in self.c["limits"])
                self.angles = self.sent_target
                self.reference_valid = True
                self.armed = self.arm_requested = True  # Simulation only.
            with self.lock:
                self.ready = True
                self.feedback_time = self.command_time = time.monotonic()
                self.command_target = self.sent_target
            self.last_send = time.monotonic()
            while not self.stop_event.wait(.02):
                if self.c["motor_enabled"]:
                    self._measure()
                now = time.monotonic()
                with self.lock:
                    target, submitted, arm_requested = self.command_target, self.command_time, self.arm_requested
                if arm_requested and not self.armed:
                    current = self._arm_current()
                    with self.lock:
                        self.armed = True
                        self.command_target = target = self.sent_target = current
                        self.command_time = submitted = time.monotonic()
                    self.guard.reset()
                    now = time.monotonic()
                elif self.armed and not arm_requested:
                    errors = self._disable_all()
                    if errors:
                        raise RuntimeError("Disable request failed: " + "; ".join(errors))
                    with self.lock:
                        self.armed = False
                        self.command_target = self.sent_target = self.angles
                    self.guard.reset()
                if not self.armed:
                    continue
                if now - submitted > self.c["watchdog_s"]:
                    raise RuntimeError("Control heartbeat expired; restart required")
                if self.c["motor_enabled"] and self.c["motion_guard"]:
                    self.guard.update(now, self.angles, self.sent_target)
                dt = min(.10, max(0.0, now - self.last_send))
                next_target = tuple(old + clip(new-old, -rate*dt, rate*dt)
                                    for old, new, rate in zip(self.sent_target, target, self.c["max_rate"]))
                if self.c["motor_enabled"]:
                    for mid, angle, sign, zero in zip(self.c["motor_ids"], next_target, self.c["motor_signs"], self.reference_zero):
                        self._write(set_multi_turn_position(mid, zero + angle / sign))
                else:
                    with self.lock:
                        self.angles, self.feedback_time = next_target, now
                self.sent_target, self.last_send = next_target, now
        except Exception as exc:
            if self.stop_event.is_set():
                return  # Intentional shutdown still runs the disable/close finally.
            self._latch_fault(str(exc))
            print("MOTOR_FAULT", str(exc))
            # Keep feedback available after a fault; recovery never re-arms.
            disable_retries = 2
            while self.serial is not None and not self.stop_event.wait(.1):
                if disable_retries:
                    self._disable_all()
                    disable_retries -= 1
                self._read_diagnostics()
        finally:
            self._disable_all()
            if self.serial is not None:
                try:
                    self.serial.close()
                except Exception as exc:
                    print("MOTOR_CLOSE_FAILED", str(exc))
            with self.lock:
                self.ready = self.armed = self.arm_requested = self.reference_valid = False

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                print("MOTOR_THREAD_STUCK: physical power isolation required")


# Source: motor_joystick\src\service.py
"""Single UART owner, joystick mailbox, verified home and cumulative reset."""

class UserDisarm(Exception):
    pass


class JoystickMotorService(MotorService):
    def __init__(self, c, transport_factory=None):
        super().__init__(c, transport_factory)
        self.joystick = (0.0, 0.0)
        self.home_requested = False
        self.abort_requested = False
        self.clear_allowed = False
        self.phase = "CONNECTING"
        self.zero_confirmed = bool(c["mechanical_zero_confirmed"])
        self.manual_target = None
        self.was_moving = False

    def input(self, x, y):
        if any(not math.isfinite(v) or abs(v) > 1 for v in (x, y)):
            raise ValueError("Joystick vector must be finite and bounded")
        with self.lock:
            self.joystick = (float(x), float(y))
            self.command_time = time.monotonic()

    def confirm_reference(self):
        with self.lock:
            if self.fault or not self.ready or self.armed or self.arm_requested or not self.reference_valid:
                raise RuntimeError("Confirm saved physical zero only while ready and disabled")
            self.zero_confirmed = True
        print("USER_CONFIRMED_SAVED_MECHANICAL_ZERO (no calibration command sent)")

    def request_arm(self, armed):
        if armed:
            with self.lock:
                if not self.zero_confirmed:
                    raise RuntimeError("Check saved physical zero, then tap ZERO OK")
                if self.home_requested or self.phase not in ("DISABLED", "HOME_DONE_DISABLED"):
                    raise RuntimeError("ARM only from disabled idle; wait for HOME to finish")
                if self.fault or not self.ready or not self.reference_valid:
                    raise RuntimeError(self.fault or "Valid dual-axis feedback required")
                self.abort_requested = False
                self.arm_requested = True
        else:
            with self.lock:
                self.abort_requested = True
                self.arm_requested = False
                self.home_requested = False
                self.joystick = (0.0, 0.0)

    def request_home(self):
        with self.lock:
            if self.fault or not self.armed or not self.arm_requested or not self.reference_valid or not self.zero_confirmed:
                raise RuntimeError("Confirm physical zero and ARM before HOME")
            if self.home_requested or self.phase not in ("ARMED", "MANUAL", "HOME_HOLD", "BRAKING"):
                raise RuntimeError("Home already active or motor not ready")
            self.home_requested = True
            self.joystick = (0.0, 0.0)

    def snapshot(self):
        value = super().snapshot()
        with self.lock:
            value.update(phase=self.phase, zero_confirmed=self.zero_confirmed,
                         target=self.sent_target, home_requested=self.home_requested)
        return value

    def _write(self, data):
        time.sleep(0.002)
        if self.stop_event.is_set() and data[2] != FUNCTION_DISABLE:
            raise UserDisarm("Motor stop requested")
        if data[2] in (FUNCTION_ENABLE, FUNCTION_MULTI_TURN_POSITION, 0x09):
            self._heartbeat_check()
            if data[2] == 0x09 and (not self.clear_allowed or self.armed):
                raise RuntimeError("Cumulative reset outside verified disabled-home transaction")
        if data[2] in (FUNCTION_ENABLE, FUNCTION_MULTI_TURN_POSITION):
            self._require_reference(fresh=True)
            if not self.arm_requested:
                raise UserDisarm("No active ARM request")
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete motor UART write")

    def _heartbeat_check(self):
        if self.abort_requested or self.stop_event.is_set():
            raise UserDisarm("User requested DISARM")
        if time.monotonic() - self.command_time > self.c["watchdog_s"]:
            raise RuntimeError("Control heartbeat expired; verify state before rearming")

    def _disable_now(self):
        errors = self._disable_all()
        self.armed = self.arm_requested = False
        self.home_requested = False
        self.joystick = (0.0, 0.0)
        self.manual_target = self.sent_target = self.angles
        self.guard.reset()
        if errors:
            raise RuntimeError("Dual-axis disable request failed: " + "; ".join(errors))

    def _advance(self, goal, rate):
        self._heartbeat_check()
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_send))
        target = tuple(old + clip(new-old, -speed*dt, speed*dt)
                       for old, new, speed in zip(self.sent_target, goal, rate))
        for mid, q, sign, zero, limits in zip(self.c["motor_ids"], target, self.c["motor_signs"], self.reference_zero, self.c["limits"]):
            if not limits[0] <= q <= limits[1]:
                raise RuntimeError("Target outside mechanical limits")
            self._write(set_multi_turn_position(mid, zero + q / sign))
        self.sent_target, self.last_send = target, now
        self.guard.update(now, self.angles, target)

    def _at_home(self):
        tol = self.c["home_tolerance_deg"]
        return (all(abs(v) <= tol for v in self.angles)
                and all(v is not None and abs(v) <= tol for v in self.mechanical))

    def _release_hold(self):
        self.manual_target = self.angles

    def _stationary_home(self):
        """After disable, independently check physical home and zero speed 3 times."""
        readings = []
        for _ in range(3):
            self._heartbeat_check()
            self._measure()
            if not self._at_home():
                raise RuntimeError("Axis left physical home after disable; cumulative reset blocked")
            for mid in self.c["motor_ids"]:
                if self._feedback(mid, FEEDBACK_SPEED) != 0:
                    raise RuntimeError("Axis moving; cumulative reset blocked")
            readings.append(self.raw_feedback)
            if self.stop_event.wait(0.08):
                raise UserDisarm("Stopped")
        for axis in range(2):
            values = [sample[axis] for sample in readings]
            if max(values) - min(values) > self.c["reference_stationary_deg"]:
                raise RuntimeError("Axis not stationary; cumulative reset blocked")

    def _home_and_clear(self):
        started = time.monotonic()
        settled = None
        previous = None
        self.phase = "RETURNING"
        self.guard.reset()
        while True:
            self._heartbeat_check()
            self._measure()
            now = time.monotonic()
            if now - started > self.c["home_timeout_s"]:
                raise RuntimeError("Home timeout; cumulative reset NOT sent")
            self._advance((0.0, 0.0), self.c["home_rate"])
            stopped = self._at_home()
            if stopped:
                for mid in self.c["motor_ids"]:
                    stopped = (self._feedback(mid, FEEDBACK_SPEED) == 0) and stopped
            if previous is not None:
                stopped = stopped and all(abs(a-b) <= self.c["reference_stationary_deg"]
                                          for a, b in zip(previous, self.raw_feedback))
            previous = self.raw_feedback
            if stopped:
                self.phase = "SETTLING"
                if settled is None:
                    settled = now
                if now - settled >= self.c["home_settle_s"]:
                    break
            else:
                settled = None
                self.phase = "RETURNING"
            if self.stop_event.wait(0.02):
                raise UserDisarm("Stopped")
        self._disable_now()
        self.phase = "CHECK_DISABLED"
        self._stationary_home()
        self._heartbeat_check()
        self._require_reference(fresh=True)
        self.phase = "CLEARING"
        self.reference_valid = False  # No position or enable until both axes verified.
        self.clear_allowed = True
        try:
            for mid in self.c["motor_ids"]:
                self._heartbeat_check()
                if abs(signed_mechanical(self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE))) > self.c["home_tolerance_deg"]:
                    raise RuntimeError("Physical home lost before cumulative reset")
                if self._feedback(mid, FEEDBACK_SPEED) != 0:
                    raise RuntimeError("Motor moving before cumulative reset")
                self._write(_frame(mid, 0x09))
                if self.stop_event.wait(0.05):
                    raise UserDisarm("Stopped during reset")
                total = self._feedback(mid)
                mech = signed_mechanical(self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE))
                if abs(total) > self.c["clear_tolerance_deg"] or abs(mech) > self.c["home_tolerance_deg"]:
                    raise RuntimeError("Cumulative reset readback failed; restart required")
            self.phase = "VERIFYING"
            self._heartbeat_check()
            self._capture_reference()  # Sole intentional reset of continuity history.
            self._measure()
            self._heartbeat_check()
            if not self._at_home() or any(abs(v) > self.c["clear_tolerance_deg"] for v in self.raw_feedback):
                raise RuntimeError("Post-reset zero verification failed")
            self.sent_target = self.manual_target = self.angles
            self.phase = "HOME_DONE_DISABLED"
            print("HOME_DONE: physical home checked, cumulative zero read back; remains DISABLED")
        except Exception:
            self.reference_valid = False
            raise
        finally:
            self.clear_allowed = False

    def _recheck_requested(self):
        return False

    def _perform_recheck(self):
        raise RuntimeError("Recheck unsupported by this service")

    def _manual_phase(self, moving):
        return "MANUAL" if moving else "ARMED"

    def _control_loop(self):
        while not self.stop_event.wait(self.c.get("motor_loop_period_s", .02)):
            if self._recheck_requested():
                return
            try:
                self._measure()
                if self.arm_requested and not self.armed:
                    self.manual_target = self.sent_target = self._arm_current()
                    self.armed = True
                    self.was_moving = False
                    self.last_send = time.monotonic()
                    self.guard.reset()
                    self.phase = "ARMED"
                if self.armed and not self.arm_requested:
                    self._disable_now()
                    self.phase = "DISABLED"
                if not self.armed:
                    continue
                self._heartbeat_check()
                if self.home_requested:
                    self._home_and_clear()
                    continue
                now = time.monotonic()
                dt = min(0.1, max(0.0, now - self.last_send))
                with self.lock:
                    vector = self.joystick
                moving = any(abs(v) > 0 for v in vector)
                if not moving and self.was_moving:
                    self._release_hold()  # Release holds current measured pose.
                if moving:
                    self.manual_target = tuple(clip(clip(old+v*rate*dt, actual-self.c["manual_lead_deg"], actual+self.c["manual_lead_deg"]), *lim)
                                               for old, actual, v, rate, lim in zip(self.manual_target, self.angles, vector, self.c["max_rate"], self.c["limits"]))
                self.was_moving = moving
                self.phase = self._manual_phase(moving)
                self._advance(self.manual_target, self.c["max_rate"])
            except UserDisarm:
                if not self.reference_valid:
                    raise RuntimeError("Reset interrupted; zero invalid; restart required")
                self._disable_now()
                self.phase = "DISABLED"

    def _record_fault(self, exc):
        if not self.stop_event.is_set():
            self.phase = "FAULT"
            self._latch_fault(str(exc), reference_lost=not self.reference_valid)
            print("MOTOR_FAULT", str(exc))

    def _run(self):
        try:
            try:
                self.serial = self._open()
                self.sent_target = self.manual_target = self._initialize()
                self.ready = True
                self.phase = "DISABLED"
                self.last_send = self.command_time = time.monotonic()
            except Exception as exc:
                self._record_fault(exc)
            while not self.stop_event.is_set():
                try:
                    if self._recheck_requested():
                        self._perform_recheck()
                    if self.fault:
                        if self.stop_event.wait(0.1):
                            break
                        self._disable_all()
                        if self.serial is not None:
                            self._read_diagnostics()
                    else:
                        self._control_loop()
                except Exception as exc:
                    self._record_fault(exc)
        finally:
            self._disable_all()
            if self.serial is not None:
                try:
                    self.serial.close()
                except Exception as exc:
                    print("MOTOR_CLOSE_FAILED", str(exc))
            self.ready = self.armed = self.arm_requested = self.reference_valid = False


# Source: motor_joystick\src\mechanical_only.py
"""Mechanical feedback ONLY: no total-angle queries, targets or cumulative reset.

Single-turn direct mode=4 by default, position opcode=0x03, uint16 tenths.
Encoding checked against gimbal-zero-worktree/gimbal_demo/3v/f32c_protocol.py.
The inherited legacy service remains for PC regressions, not this board entry.
"""


def set_single_turn_position(mid, angle):
    if not math.isfinite(angle):
        raise ValueError("Nonfinite mechanical target")
    value = int(round((angle % 360.0) * 10)) % 3600
    return _frame(mid, 0x03, value.to_bytes(2, "big"))


class MechanicalOnlyService(JoystickMotorService):
    def __init__(self, c, transport_factory=None):
        if c["motor_reference"] != "mechanical_only":
            raise ValueError("Current board entry requires mechanical_only mode")
        super().__init__(c, transport_factory)
        self.mechanical_samples = [None, None]
        self.mechanical_feedback_times = [None, None]
        self.last_angle_jump = None
        self.stream_target = None
        self.stream_velocity = (0.0, 0.0)
        self.position_units = [None, None]
        self.last_position_send = 0.0
        self.hold_pending = False
        self.recheck_pending = False
        self.recheck_status = "IDLE"
        self.home_holding = False

    def request_recheck(self):
        """Mailbox only: the existing UART owner performs all checks."""
        with self.lock:
            if self.stop_event.is_set() or self.armed or self.arm_requested:
                raise RuntimeError("DISARM before rechecking")
            if self.recheck_pending or self.phase not in ("FAULT", "DISABLED", "HOME_DONE_DISABLED"):
                raise RuntimeError("Wait for idle before rechecking")
            self.recheck_pending = True
            self.recheck_status = "PENDING"
            self.phase = "RECHECK_PENDING"
            self.ready = self.zero_confirmed = False
            self.joystick = (0.0, 0.0)
            self.abort_requested = False

    def _recheck_requested(self):
        return self.recheck_pending

    def _recheck_stationary_required(self):
        return True  # Standalone demo retains its existing stationary check.

    def _perform_recheck(self):
        with self.lock:
            self.recheck_pending = False
            self.recheck_status = "CHECKING"
            self.phase = "RECHECKING"
        try:
            if self.serial is None:
                self.serial = self._open()
            self._disable_now()
            self.reference_valid = False
            self.mechanical_samples = [None, None]
            samples = []
            stationary_required = self._recheck_stationary_required()
            for _ in range(3 if stationary_required else 1):
                if self.abort_requested or self.stop_event.is_set():
                    raise UserDisarm("Recheck cancelled")
                # Bypass the old reference gate only for fresh read-only checks.
                # Existing fault remains latched until the complete check passes.
                self._read_measurement()
                if stationary_required:
                    for mid in self.c["motor_ids"]:
                        speed = self._feedback(mid, FEEDBACK_SPEED)
                        if speed != 0:
                            raise RuntimeError("Recheck: ID=%d still moving, speed=%g RPM" % (mid, speed))
                samples.append(self.mechanical)
                if stationary_required and self.stop_event.wait(0.08):
                    raise UserDisarm("Recheck stopped")
            if stationary_required and any(max(s[a] for s in samples)-min(s[a] for s in samples) > self.c["reference_stationary_deg"] for a in range(2)):
                raise RuntimeError("Recheck: mechanical position not stationary")
            with self.lock:
                if self.abort_requested or self.stop_event.is_set():
                    raise UserDisarm("Recheck cancelled")
                self.fault = self.reference_fault = self.last_angle_jump = None
                self.reference_valid = self.ready = True
                self.armed = self.arm_requested = self.home_requested = False
                self.zero_confirmed = False
                self.manual_target = self.sent_target = self.angles
                self.was_moving = False
                self.last_send = self.command_time = time.monotonic()
                self.phase = "DISABLED"
                self.recheck_status = "PASSED"
            print("MOTOR_RECHECK_PASSED: remains DISABLED; ZERO OK and manual ARM required")
        except Exception as exc:
            self.recheck_status = "FAILED"
            self.phase = "FAULT"
            self.ready = self.reference_valid = False
            # Replace the historical fault with this attempt's concrete reason.
            self.fault = self.reference_fault = str(exc)
            raise

    def _manual_phase(self, moving):
        if moving:
            self.home_holding = False
        return "HOME_HOLD" if self.home_holding else super()._manual_phase(moving)

    def input(self, x, y):
        super().input(x*self.c["joystick_signs"][0], y*self.c["joystick_signs"][1])

    def snapshot(self):
        result = super().snapshot()
        result.update(angle_source="mechanical_only", cumulative_angle_used=False)
        result.update(limits=self.c["limits"], joystick_signs=self.c["joystick_signs"],
                      position_mode=self.c["position_mode"], max_rate=self.c["max_rate"],
                      home_rate=self.c["home_rate"])
        with self.lock:
            now = time.monotonic()
            result["control_feedback_seen"] = self.feedback_time > 0
            if not result["control_feedback_seen"]:
                result["age"] = None
            result["mechanical_feedback_age"] = tuple(
                None if stamp is None else max(0.0, now-stamp)
                for stamp in self.mechanical_feedback_times)
            result["last_angle_jump"] = self.last_angle_jump
            result["recheck_status"] = self.recheck_status
            result["can_recheck"] = (not self.armed and not self.arm_requested and
                self.phase in ("FAULT", "DISABLED", "HOME_DONE_DISABLED") and not self.stop_event.is_set())
        return result

    def _mechanical_feedback(self, mid):
        value = self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE)
        if not math.isfinite(value) or not 0 <= value <= 360:
            raise RuntimeError("Invalid mechanical feedback from motor %d: %r" % (mid, value))
        angle = round(signed_mechanical(value), 7)
        # Diagnostic feedback remains visible even when rejected by control limits.
        # It must never refresh the accepted control feedback timestamp.
        axis = self.c["motor_ids"].index(mid)
        with self.lock:
            values = list(self.mechanical)
            values[axis] = angle
            self.mechanical = tuple(values)
            self.mechanical_feedback_times[axis] = time.monotonic()
        return angle

    def _require_reference(self, fresh=False):
        if self.fault or not self.reference_valid:
            self._lose_reference("mechanical feedback/reference unavailable; RECHECK while disabled")
        if fresh and time.monotonic()-self.feedback_time > self.c["reference_max_age_s"]:
            self._lose_reference("mechanical feedback expired")

    def _capture_reference(self):
        self.reference_valid = False
        for mid in self.c["motor_ids"]:
            first = self._mechanical_feedback(mid)
            last = self._mechanical_feedback(mid)
            if abs(last-first) > self.c["reference_stationary_deg"]:
                raise RuntimeError("Axis moved during mechanical-only initialization")
        self.mechanical_samples = [None, None]
        self.reference_valid = True

    def _initialize(self):
        errors = self._disable_all()
        if errors:
            raise RuntimeError("Initial disable request failed: " + "; ".join(errors))
        self._capture_reference()
        return self._measure()[1]

    def _read_measurement(self):
        mechanical, stamps = [], []
        for axis, mid in enumerate(self.c["motor_ids"]):
            angle = self._mechanical_feedback(mid)
            now = time.monotonic()
            previous = self.mechanical_samples[axis]
            # The configured motor RPM bounds commanded motion, not a person's
            # movement of a disabled axis. ARM still remeasures/preloads its pose.
            if previous is not None and (self.armed or self.arm_requested):
                dt = max(0, now-previous[0])
                bound = self.c["motor_rpm"] * 6 * dt + self.c["reference_jump_slack_deg"]
                # No modulo here: valid working limits never cross +/-180.
                delta = abs(angle-previous[1])
                if delta > bound:
                    with self.lock:
                        self.last_angle_jump = dict(axis=axis, motor_id=mid,
                            previous_deg=previous[1], current_deg=angle, delta_deg=delta,
                            elapsed_s=dt, allowed_deg=bound, armed=self.armed,
                            arm_requested=self.arm_requested, phase=self.phase)
                    raise RuntimeError("Unexpected mechanical angle jump: %s ID=%d %.1f->%.1f deg, delta=%.2f > %.2f deg in %.3fs; armed=%s arm_requested=%s" % (
                        ("YAW", "PITCH")[axis], mid, previous[1], angle, delta, bound,
                        dt, self.armed, self.arm_requested))
            mechanical.append(angle)
            stamps.append(now)
        angles = tuple(round(v*s, 7) for v, s in zip(mechanical, self.c["motor_signs"]))
        for axis, (value, (lo, hi)) in enumerate(zip(angles, self.c["limits"])):
            if not lo <= value <= hi:
                raise RuntimeError("Mechanical position outside calibrated limits: %s ID=%d %.1f deg, allowed %.1f..%.1f deg; move inside limits while disabled, then RECHECK" % (
                    ("YAW", "PITCH")[axis], self.c["motor_ids"][axis], value, lo, hi))
        with self.lock:
            self.mechanical = tuple(mechanical)
            self.angles = angles
            self.raw_feedback = (None, None)  # Explicitly unused, never fabricated totals.
            self.feedback_time = min(stamps)
            self.mechanical_samples = list(zip(stamps, mechanical))
        return tuple(mechanical), angles

    def _read_diagnostics(self):
        values = list(self.mechanical)
        for axis, mid in enumerate(self.c["motor_ids"]):
            try:
                values[axis] = self._mechanical_feedback(mid)
            except Exception:
                pass
        with self.lock:
            self.mechanical = tuple(values)
            self.raw_feedback = (None, None)
        # Diagnostic data does not rebuild reference or permit automatic re-arm.

    def _write(self, data):
        opcode = data[2]
        if opcode in (0x02, 0x08, 0x09, 0x0A) or (opcode == 0x0E and data[3] == FEEDBACK_TOTAL_ANGLE):
            raise RuntimeError("Cumulative/calibration command forbidden in mechanical-only mode")
        if opcode == FUNCTION_MODE and int.from_bytes(data[3:5], "big") != self.c["position_mode"]:
            raise RuntimeError("Mechanical-only mode requires configured single-turn mode")
        if opcode != 0x03:
            return super()._write(data)
        time.sleep(0.002)
        self._heartbeat_check()
        self._require_reference(fresh=True)
        if not self.arm_requested:
            raise UserDisarm("No active ARM request")
        if len(data) != 7 or data[1] not in self.c["motor_ids"]:
            raise RuntimeError("Invalid single-turn target frame")
        axis = self.c["motor_ids"].index(data[1])
        units = int.from_bytes(data[3:5], "big")
        q = signed_mechanical(units / 10.0) * self.c["motor_signs"][axis]
        lo, hi = self.c["limits"][axis]
        # The two endpoints are close, inside one convex allowed interval.
        # Therefore shortest-path single-turn motion cannot take the rear route.
        if units >= 3600 or not lo <= q <= hi or abs(q-self.angles[axis]) > self.c["manual_lead_deg"] + 0.051:
            raise RuntimeError("Unsafe single-turn step; no target sent")
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete motor UART write")

    def _arm_current(self):
        mechanical, angles = self._measure()
        for mid, angle in zip(self.c["motor_ids"], mechanical):
            self._write(select_mode(mid, self.c["position_mode"]))
            self._write(set_speed(mid, self.c["motor_rpm"]))
            self._write(set_acceleration(mid, self.c["motor_acceleration"]))
            self._write(set_single_turn_position(mid, angle))
        for mid in self.c["motor_ids"]:
            self._write(enable(mid))
        self.stream_target = angles
        self.stream_velocity = (0.0, 0.0)
        self.position_units = [int.from_bytes(set_single_turn_position(mid, angle)[3:5], "big")
                               for mid, angle in zip(self.c["motor_ids"], mechanical)]
        self.last_position_send = time.monotonic()
        self.hold_pending = False
        return angles

    def _release_hold(self):
        super()._release_hold()
        self.hold_pending = True  # One immediate hold; never chase feedback noise.

    def _disable_now(self):
        super()._disable_now()
        self.home_holding = False
        self.stream_target = None
        self.stream_velocity = (0.0, 0.0)
        self.hold_pending = False

    def _advance(self, goal, rate):
        self._heartbeat_check()
        self._require_reference(fresh=True)
        now = time.monotonic()
        dt = min(0.1, max(0.0, now-self.last_send))
        force_hold = self.hold_pending
        old_target = self.stream_target if self.stream_target is not None else self.sent_target
        target, velocity = [], []
        for old, new, old_v, actual, speed, accel, limits in zip(
                old_target, goal, self.stream_velocity, self.angles, rate,
                self.c["command_accel_deg_s2"], self.c["limits"]):
            error = new-old
            desired_v = math.copysign(min(speed, math.sqrt(2*accel*abs(error))), error) if error else 0.0
            v = clip(desired_v, old_v-accel*dt, old_v+accel*dt)
            step = v*dt
            candidate = old+step
            if step*error >= 0 and abs(step) >= abs(error):
                candidate, v = new, 0.0
            if force_hold:
                candidate, v = new, 0.0  # Release takes priority over a smoothing tail.
            q = clip(clip(candidate, actual-self.c["manual_lead_deg"], actual+self.c["manual_lead_deg"]), *limits)
            if q != candidate:
                v = 0.0
            target.append(q)
            velocity.append(v)
        self.stream_target, self.stream_velocity = tuple(target), tuple(velocity)
        self.last_send = now  # Planner timing is independent of packet timing.
        if force_hold or now-self.last_position_send >= self.c["position_period_s"]:
            sent = list(self.sent_target)
            for axis, (mid, q, sign) in enumerate(zip(self.c["motor_ids"], target, self.c["motor_signs"])):
                frame = set_single_turn_position(mid, q / sign)
                units = int.from_bytes(frame[3:5], "big")
                if units != self.position_units[axis]:
                    self._write(frame)  # Final reference/lead/limit gate always applies.
                    self.last_position_send = now
                    self.position_units[axis] = units
                    sent[axis] = decoded_position(self.c, axis, units, self.angles[axis])
            self.sent_target = tuple(sent)
            self.hold_pending = False
        self.guard.update(now, self.angles, self.sent_target)

    def _stationary_home(self):
        samples = []
        for _ in range(3):
            self._heartbeat_check()
            self._measure()
            if not self._at_home():
                raise RuntimeError("Axis left mechanical home after disable: yaw=%.1f pitch=%.1f deg" % self.angles)
            for mid in self.c["motor_ids"]:
                speed = self._feedback(mid, FEEDBACK_SPEED)
                if speed != 0:
                    raise RuntimeError("Axis moving after disable: ID=%d speed=%g RPM; HOME not verified, support/balance the load" % (mid, speed))
            samples.append(self.mechanical)
            if self.stop_event.wait(0.08):
                raise UserDisarm("Stopped")
        if any(max(s[a] for s in samples)-min(s[a] for s in samples) > self.c["reference_stationary_deg"] for a in range(2)):
            raise RuntimeError("Mechanical home not stationary")

    def _home_and_clear(self):
        """Worker hook retained by name; current implementation NEVER clears."""
        started = time.monotonic()
        settled = None
        previous = None
        self.guard.reset()
        self.phase = "RETURNING"
        while True:
            self._heartbeat_check()
            self._measure()
            now = time.monotonic()
            if now-started > self.c["home_timeout_s"]:
                raise RuntimeError("Mechanical home timeout")
            self._advance((0.0, 0.0), self.c["home_rate"])
            stopped = self._at_home()
            if stopped:
                for mid in self.c["motor_ids"]:
                    stopped = (self._feedback(mid, FEEDBACK_SPEED) == 0) and stopped
            if previous is not None:
                stopped = stopped and all(abs(a-b) <= self.c["reference_stationary_deg"] for a, b in zip(previous, self.mechanical))
            previous = self.mechanical
            if stopped:
                self.phase = "SETTLING"
                settled = now if settled is None else settled
                if now-settled >= self.c["home_settle_s"]:
                    break
            else:
                self.phase, settled = "RETURNING", None
            if self.stop_event.wait(0.02):
                raise UserDisarm("Stopped")
        self._heartbeat_check()
        self.manual_target = (0.0, 0.0)
        self.home_requested = self.was_moving = False
        self.home_holding = True
        self.phase = "HOME_HOLD"
        print("HOME_DONE: mechanical-only, remains ARMED holding zero; cumulative angle untouched")


# Source: motor_joystick\src\web.py
"""Browser control; UART remains owned exclusively by MechanicalOnlyService."""
import json
import secrets
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WEB_CONFIG = {"host": "0.0.0.0", "port": 8080, "lease_s": 0.6}


class ControlError(Exception):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


class BrowserControl:
    def __init__(self, motor, clock=time.monotonic, lease_s=0.6):
        if not 0.2 <= lease_s < motor.c["watchdog_s"]:
            raise ValueError("Browser lease must be shorter than motor watchdog")
        self.motor, self.clock, self.lease_s = motor, clock, lease_s
        self.lock = threading.RLock()
        self.token = None
        self.deadline = 0
        self.seq = 0
        self.hold_start = None
        self.blocked = False
        self.tuning_error = None
        self.notice = "请连接控制，确认已有机械零点后 ZERO OK → ARM"

    def _stop(self, reason):
        self._disable_request(reason)
        self.token = None
        self.hold_start = None
        self.blocked = False
        self.notice = reason

    def _disable_request(self, reason):
        self.motor.request_arm(False)

    def _reject_command(self, reason, status=409):
        self._stop(reason)
        raise ControlError(reason, status)

    def poll(self):
        with self.lock:
            if self.token and self.clock() >= self.deadline:
                if getattr(self.motor, "autotune_active", False):
                    self.tuning_background(self.token)
                else:
                    self._stop("控制连接超时，已请求失能；请重新连接并手动 ARM")

    def tuning_background(self, token):
        with self.lock:
            if not token or token != self.token:
                raise ControlError("控制连接已失效", 403)
            if not getattr(self.motor, "autotune_active", False):
                return self.stop(token)
            self.motor.pi_autotune.detached = True
            self.motor.input(0, 0)
            self.token = None
            self.notice = "板端继续 PI 自动整定，完成后保持使能"
            return self.state()

    def state(self):
        with self.lock:
            self.poll()
            value = self.motor.snapshot()
            value.update(connected=bool(self.token), notice=self.notice, hold_blocked=self.blocked,
                         tuning_error=self.tuning_error)
            return value

    def recheck(self, token):
        with self.lock:
            self.poll()
            if self.token and token != self.token:
                raise ControlError("另一网页正在控制，请先在该网页 DISARM", 403)
            try:
                self.motor.request_recheck()
            except RuntimeError as exc:
                raise ControlError(str(exc))
            self.token = None  # Revoke old queued ARM/movement commands.
            self.hold_start, self.blocked = None, False
            self.notice = "重新检查中，通过后连接控制 → ZERO OK → 手动使能"
            return self.state()

    def connect(self):
        with self.lock:
            self.poll()
            if self.token:
                raise ControlError("已有网页控制，请先断开或等待超时")
            state = self.motor.snapshot()
            tune_takeover = bool(state.get("autotune_takeover"))
            takeover = bool(state.get("startup_takeover")) or tune_takeover
            if state["fault"] or not state["ready"] or ((state["armed"] or self.motor.arm_requested) and not takeover):
                raise ControlError(state["fault"] or "请等待电机就绪且失能")
            if takeover:
                if tune_takeover:
                    self.motor.pi_autotune.detached = False
                elif not self.motor.claim_startup_hold():
                    raise ControlError("启动保持状态已改变，请刷新状态")
            else:
                self.motor.request_arm(False)
            self.motor.input(0, 0)
            self.tuning_error = None
            self.token = secrets.token_urlsafe(24)
            self.deadline, self.seq = self.clock() + self.lease_s, 0
            self.notice = "控制已连接，继续保持机械原点" if takeover else "控制已连接，当前失能；检查零点后 ZERO OK → ARM"
            return {"token": self.token, "state": self.state()}

    def stop(self, token):
        with self.lock:
            if token != self.token:
                raise ControlError("控制连接已失效", 403)
            self._stop("已请求双轴失能；再次操作请重新连接")
            return self.state()

    def command(self, body):
        with self.lock:
            self.poll()
            if not self.token or body.get("token") != self.token:
                raise ControlError("控制连接已失效，请重新连接", 403)
            seq, x, y, action = (body.get(k) for k in ("seq", "x", "y", "action"))
            if type(seq) is not int or seq != self.seq + 1:
                raise ControlError("控制序号无效")
            if (any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1 for v in (x, y))
                    or math.hypot(x, y) > 1.001 or action not in ("", "zero", "arm", "home", "pi", "record_start", "record_stop", "pi_autotune", "pi_autotune_cancel", "pi_autotune_save")):
                self._stop("无效控制输入，已请求失能")
                raise ControlError("无效控制输入", 400)
            self.seq = seq
            now = self.clock()
            if x == 0 and y == 0:
                self.hold_start, self.blocked = None, False
            else:
                self.hold_start = now if self.hold_start is None else self.hold_start
                if now-self.hold_start >= self.motor.c["max_touch_hold_s"]:
                    self.blocked = True
            if self.blocked:
                x = y = 0
            # ARM/HOME must start with released controls. Never queue movement with ARM.
            if action and (x or y):
                self._reject_command("按钮操作需先松开摇杆／方向键", 400)
            tuning_action = action in ("pi", "record_start", "record_stop", "pi_autotune", "pi_autotune_cancel", "pi_autotune_save")
            if tuning_action:
                self.tuning_error = None
            try:
                self.motor.input(x, y)
                if action == "zero":
                    self.motor.confirm_reference()
                    self.notice = "已确认现有机械零点（没有重新标定）"
                elif action == "arm":
                    self.motor.request_arm(True)
                    self.notice = "已请求在当前实测位置使能"
                elif action == "home":
                    self.motor.request_home()
                    self.notice = "正在返回机械原点；完成后保持使能"
                elif action == "pi":
                    self.motor.request_speed_pi(body.get("axis"), body.get("kp"), body.get("ki"))
                    self.notice = "已提交临时 PI 请求；等待串口线程静止检查"
                elif action == "record_start":
                    self.motor.start_tuning_capture(body.get("note", ""))
                    self.notice = "记录已开始；此操作不发运动指令"
                elif action == "record_stop":
                    self.motor.stop_tuning_capture()
                    self.notice = "记录已结束，可以导出实测数据"
                elif action == "pi_autotune":
                    self.motor.request_pi_autotune(body.get("axis"), body.get("kp"), body.get("ki"))
                    self.notice = "已提交 PI 自动整定，基线取自填写的 Kp/Ki"
                elif action == "pi_autotune_cancel":
                    self.motor.cancel_pi_autotune()
                    self.notice = "正在回到测试起点并恢复已知 PI 基线"
                elif action == "pi_autotune_save":
                    self.notice = self.motor.pi_autotune.save()
            except (RuntimeError, ValueError) as exc:
                if tuning_action:
                    self.tuning_error = str(exc)
                    self.deadline = now+self.lease_s
                    return self.state()  # Invalid tuning input must not drop holding torque.
                self._reject_command(str(exc))
            self.deadline = now + self.lease_s
            return self.state()


def make_http_server(control, address):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(0.3)

        def log_message(self, *args):
            pass

        def reply(self, status, body, content_type="application/json; charset=utf-8", filename=None, headers=None):
            data = body if isinstance(body, bytes) else body.encode("utf-8") if isinstance(body, str) else json.dumps(body, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("X-Content-Type-Options", "nosniff")
            if filename:
                self.send_header("Content-Disposition", 'attachment; filename="%s"' % filename)
            self.send_header("Cache-Control", "no-store, max-age=0")
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError, socket.timeout):
                pass

        def do_GET(self):
            if self.path == "/":
                self.reply(200, WEB_HTML, "text/html; charset=utf-8")
            elif self.path == "/api/telemetry":
                self.reply(200, control.telemetry())
            elif self.path == "/api/autotune":
                with control.lock:
                    report = control.software_autotune.export()
                self.reply(200, report)
            elif self.path == "/api/frame":
                try:
                    frame = control.media()
                except (RuntimeError, ValueError, OSError):
                    frame = None
                if frame is None:
                    self.reply(503, {"error": "No fresh video"})
                else:
                    self.reply(200, frame["jpeg"], "image/jpeg", headers={"X-Frame-Id": str(frame["id"])})
            elif self.path == "/api/state":
                self.reply(200, control.state())
            else:
                self.reply(404, {"error": "Not found"})

        def do_POST(self):
            try:
                # Same-origin JSON only; no CORS, file serving, shell or raw serial API.
                if self.headers.get("Origin") != "http://" + self.headers.get("Host", ""):
                    raise ControlError("Origin rejected", 403)
                if self.headers.get_content_type() != "application/json":
                    raise ControlError("JSON required", 415)
                if self.headers.get("Transfer-Encoding"):
                    raise ControlError("Chunked requests unsupported", 400)
                size = int(self.headers.get("Content-Length", "0"))
                if not 1 <= size <= 1024:
                    raise ControlError("Invalid body size", 400)
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ControlError("JSON object required", 400)
                if self.path == "/api/connect":
                    value = control.connect()
                elif self.path == "/api/control":
                    value = control.command(body)
                elif self.path == "/api/background":
                    value = control.background(body.get("token"))
                elif self.path == "/api/stop":
                    value = control.stop(body.get("token"))
                elif self.path == "/api/recheck":
                    value = control.recheck(body.get("token"))
                elif self.path == "/api/tuning/background":
                    value = control.tuning_background(body.get("token"))
                else:
                    raise ControlError("Not found", 404)
                self.reply(200, value)
            except ControlError as exc:
                self.reply(exc.status, {"error": str(exc)})
            except (ValueError, TypeError, socket.timeout):
                self.reply(400, {"error": "Invalid JSON request"})

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server




# Source: motor_joystick\src\autotune_core.py
"""Bounded, observation-driven response experiments. No hardware imports."""
import copy
import math


class AutoTuneExperiment:
    """One axis, matched +/- steps, baseline repeat and winner verification.

    apply(profile) runs only after centering has settled. set_offset(degrees)
    changes either a visual setpoint or a guarded local mechanical target.
    All callbacks execute in the caller's control loop, never an extra UART thread.
    """
    def __init__(self, apply, set_offset, amplitude=1.5, segment_s=1.5, tolerance=.3, amplitude_limit=4.):
        # Larger physical trials require an explicit caller limit and a callback
        # that ramps each waypoint through its own per-axis working limits.
        if (not math.isfinite(amplitude_limit) or not .5 <= amplitude_limit <= 15
                or not .5 <= amplitude <= amplitude_limit or not 1 <= segment_s <= 3 or not .1 <= tolerance <= 3):
            raise ValueError("Invalid bounded experiment settings")
        self.apply = apply
        self.set_offset = set_offset
        self.amplitude = amplitude
        self.segment_s = segment_s
        self.tolerance = tolerance
        self.active = False
        self.state = "IDLE"
        self.results = []
        self.best = None
        self.reason = None
        self.restore_pending = False

    def start(self, baseline, candidates, now):
        if self.active:
            raise RuntimeError("Auto tune already active")
        if not math.isfinite(now) or not 1 <= len(candidates) <= 8:
            raise ValueError("Invalid bounded candidate search")
        self.baseline = copy.deepcopy(baseline)
        self.candidates = [copy.deepcopy(baseline)] + copy.deepcopy(candidates)
        self.index = 0
        self.kind = "search"
        self.best = copy.deepcopy(baseline)
        self.best_score = None
        self.results = []
        self.reason = None
        self.restore_pending = False
        self.started = self.last_stamp = now
        self.active = True
        self._center(now)

    def _center(self, now):
        self.state = "CENTERING"
        self.phase_started = now
        self.stable_since = None
        self.set_offset(0.)

    def cancel(self, now, reason="user cancelled"):
        if self.active:
            self.reason = reason
            self.state = "CANCELLING"
            self.phase_started = now
            self.stable_since = None
            self.restore_pending = True
            try:
                self.set_offset(0.)
            except Exception as exc:
                self.fail("Return-to-center failed: " + str(exc))

    def fail(self, reason):
        # A feedback/transport failure cannot establish that rollback succeeded.
        self.active = False
        self.state = "FAILED"
        self.reason = reason
        self.restore_pending = True

    def observe(self, now, error_deg, velocity_deg_s=0., valid=True, gyro_rms=None):
        if not self.active:
            return
        if (not valid or not all(math.isfinite(v) for v in (now, error_deg, velocity_deg_s))
                or now <= self.last_stamp or now-self.last_stamp > .25):
            self.fail("Missing, stale or invalid observation; restore pending")
            return
        dt = now-self.last_stamp
        self.last_stamp = now
        if now-self.started > 120 and self.state != "CANCELLING":
            self.cancel(now, "experiment timeout")
        if self.state in ("CENTERING", "CANCELLING"):
            stable = abs(error_deg) <= self.tolerance and abs(velocity_deg_s) <= 1.
            self.stable_since = (now if self.stable_since is None else self.stable_since) if stable else None
            if self.stable_since is not None and now-self.stable_since >= .4:
                try:
                    if self.state == "CANCELLING":
                        self.apply(copy.deepcopy(self.baseline))
                        self.restore_pending = False
                        self.state, self.active = "CANCELLED", False
                        return
                    if self.kind == "finish":
                        self.apply(copy.deepcopy(self.best))
                        self.state, self.active = "COMPLETE", False
                        return
                    self.profile = copy.deepcopy(self.candidates[self.index] if self.kind == "search" else
                                                 self.baseline if self.kind == "baseline_repeat" else self.best)
                    self.apply(self.profile)
                    self.segment = 0
                    self.area = self.duration = self.peak = self.tail_area = self.tail_duration = 0.
                    self.gyro_area = self.gyro_duration = 0.
                    self.position_min = self.position_max = 0.
                    self._step(now)
                except Exception as exc:
                    self.fail("Profile/target application failed: " + str(exc))
                return
            if now-self.phase_started > 8:
                if self.state == "CANCELLING":
                    self.fail("Centering for rollback timed out")
                else:
                    self.cancel(now, "centering did not settle")
            return
        if self.state != "MEASURING":
            return
        self.area += error_deg*error_deg*dt
        inferred_position = (self.amplitude, 0., -self.amplitude, 0.)[self.segment]-error_deg
        self.position_min = min(self.position_min, inferred_position)
        self.position_max = max(self.position_max, inferred_position)
        self.duration += dt
        self.peak = max(self.peak, abs(error_deg))
        if now-self.phase_started >= self.segment_s-.5:
            self.tail_area += error_deg*error_deg*dt
            self.tail_duration += dt
            if gyro_rms is not None:
                if not math.isfinite(gyro_rms) or gyro_rms < 0:
                    self.fail("Invalid gyro observation")
                    return
                self.gyro_area += gyro_rms*gyro_rms*dt
                self.gyro_duration += dt
        if now-self.phase_started < self.segment_s:
            return
        self.segment += 1
        if self.segment < 4:
            try:
                self._step(now)
            except Exception as exc:
                self.fail("Step command failed: " + str(exc))
            return
        rms = math.sqrt(self.area/self.duration)
        tail = math.sqrt(self.tail_area/self.tail_duration)
        record = dict(kind=self.kind, profile=copy.deepcopy(self.profile), rmse_deg=rms,
                      peak_error_deg=self.peak, tail_rmse_deg=tail,
                      gyro_rms=math.sqrt(self.gyro_area/self.gyro_duration) if self.gyro_duration else None,
                      score=rms+.5*tail+.1*self.peak, duration_s=self.duration)
        self.results.append(record)
        if self.position_max-self.position_min < .3:
            self.cancel(now, "Insufficient measured response for calibration")
            return
        self._next(now, record)

    def _step(self, now):
        # Always return to center between +/- steps, preserving the local path.
        offset = (self.amplitude, 0., -self.amplitude, 0.)[self.segment]
        self.set_offset(offset)
        self.state = "MEASURING"
        self.phase_started = now

    def _next(self, now, record):
        score = record["score"]
        if self.kind == "search":
            if self.index == 0:
                self.initial_score = score
            if self.best_score is None or score < self.best_score:
                self.best_score, self.best = score, copy.deepcopy(self.profile)
            self.index += 1
            if self.index >= len(self.candidates):
                self.kind = "baseline_repeat"
        elif self.kind == "baseline_repeat":
            self.repeat_score = score
            if abs(score-self.initial_score) > max(.1, .35*self.initial_score):
                self.cancel(now, "Baseline drift; trials are not comparable")
                return
            self.kind = "verify"
        else:
            accepted = score < self.repeat_score*.95
            self.best = copy.deepcopy(self.best if accepted else self.baseline)
            self.reason = "Verified improvement" if accepted else "Baseline retained after verification"
            self.kind = "finish"
            self._center(now)
            return
        self._center(now)

    def snapshot(self):
        return dict(active=self.active, state=self.state, kind=getattr(self, "kind", None),
                    candidate=getattr(self, "index", 0), reason=self.reason,
                    restore_pending=self.restore_pending, best=copy.deepcopy(self.best),
                    trials=copy.deepcopy(self.results))


# Source: yolo_click_tracker\src\vision.py
"""Built-in NPU YOLO, same-ID tracking, and timestamped double buffering."""
from collections import deque


class NativeVision:
    def __init__(self, c):
        from maix import nn, tracker
        self.c, self.api = c, tracker
        if c["model_kind"] not in ("YOLO11", "YOLOv8"):
            raise ValueError("Choose YOLO11 or YOLOv8 to match the .mud model")
        self.model = getattr(nn, c["model_kind"])(model=c["model_path"], dual_buff=c["dual_buff"])
        self.labels = self.model.labels
        self.drone_label_declared=any('drone' in str(label).lower() or 'uav' in str(label).lower() for label in self.labels)
        self.trackers = {}
        self.preview_target = None
        self.inference_ms = None
        self.tracking_ms = None

    def detect(self, img):
        started = time.monotonic()
        objects = self.model.detect(img, conf_th=self.c["confidence"], iou_th=self.c["iou"])
        self.inference_ms = (time.monotonic()-started)*1000
        grouped = {}
        for obj in objects:
            cid = int(obj.class_id)
            if self.c["class_ids"] is not None and cid not in self.c["class_ids"]:
                continue
            grouped.setdefault(cid, []).append(self.api.Object(obj.x, obj.y, obj.w, obj.h, cid, obj.score))
        for cid in grouped:
            if cid not in self.trackers:
                # Lost buffer is frames; actual control timeout below is wall-clock 2s.
                self.trackers[cid] = self.api.ByteTracker(max(1, int(self.c["inference_fps"] * self.c["lost_timeout"])), self.c['tracker_confidence'], self.c['new_track_confidence'], .8, 60)
        output = []
        width, height = img.width(), img.height()
        for cid, tracker_instance in self.trackers.items():
            for track in tracker_instance.update(grouped.get(cid, [])):
                if track.lost:
                    continue
                history = track.history
                if not history:
                    continue
                obj = history[-1]
                key = (cid, int(track.id))
                output.append({"key": key, "box": (obj.x, obj.y, obj.w, obj.h),
                               "cx": (obj.x + obj.w/2) / width, "cy": (obj.y + obj.h/2) / height,
                               "score": float(track.score), "label": self.labels[cid],
                               "trail": [(o.x + o.w/2, o.y + o.h/2) for o in history[-40:]] if key == self.preview_target else []})
        self.tracking_ms = (time.monotonic()-started)*1000-self.inference_ms
        return output


class DetectionPipeline:
    """dual_buff results belong to the previous submitted image, not this one."""
    def __init__(self, vision, c, clock=time.monotonic):
        self.vision, self.c, self.clock = vision, c, clock
        self.pending = None
        self.completed = deque(maxlen=30)
        self.api_ms = self.latency_ms = None

    def push(self, img, stamp, pose):
        submitted = (img, stamp, pose)
        started = self.clock()
        tracks = self.vision.detect(img)
        finished = self.clock()
        self.api_ms = (finished-started)*1000
        if self.c["dual_buff"]:
            result = self.pending
            self.pending = submitted
        else:
            result = submitted
        if result is None:
            return None  # Warm-up is not a valid detection or control heartbeat.
        frame, captured, capture_pose = result
        self.completed.append(finished)
        self.latency_ms = max(0., finished-captured)*1000
        return frame, tracks, captured, capture_pose

    def stats(self):
        fps = 0.
        if len(self.completed) > 1 and self.completed[-1] > self.completed[0]:
            fps = (len(self.completed)-1)/(self.completed[-1]-self.completed[0])
        return dict(inference_fps_actual=round(fps, 1), detect_api_ms=self.api_ms,
                    model_detect_ms=getattr(self.vision, "inference_ms", None),
                    detector_confidence=self.c['confidence'],new_track_confidence=self.c['new_track_confidence'],
                    drone_label_declared=getattr(self.vision,'drone_label_declared',None),
                    tracking_postprocess_ms=getattr(self.vision, "tracking_ms", None),
                    capture_to_result_ms=self.latency_ms, dual_buff=self.c["dual_buff"],
                    model_path=self.c["model_path"], model_kind=self.c["model_kind"])

    def close(self):
        self.pending = None


def configure_camera_rotation(cam, c):
    """Use sensor/ISP mirroring if supported; otherwise restore and rotate in CPU."""
    if not c["rotate_180"] or not c["camera_rotate"]:
        return False
    try:
        horizontal, vertical = cam.hmirror(), cam.vflip()
        if horizontal not in (0, 1) or vertical not in (0, 1):
            raise RuntimeError("Camera flip state is unavailable")
    except Exception as exc:
        print("CAMERA_ROTATION CPU fallback:", str(exc), flush=True)
        return False
    try:
        cam.hmirror(1-horizontal)
        cam.vflip(1-vertical)
        if (cam.hmirror(), cam.vflip()) != (1-horizontal, 1-vertical):
            raise RuntimeError("Camera flip setting not acknowledged")
    except Exception as exc:
        # Never combine a half-applied camera flip with another 180-degree rotation.
        try:
            cam.hmirror(horizontal)
            cam.vflip(vertical)
            if (cam.hmirror(), cam.vflip()) != (horizontal, vertical):
                raise RuntimeError("Camera flip restoration failed")
        except Exception as restore:
            raise RuntimeError("Camera orientation is uncertain; motor startup blocked") from restore
        print("CAMERA_ROTATION CPU fallback:", str(exc), flush=True)
        return False
    print("CAMERA_ROTATION camera hmirror+vflip (no CPU image rotation)", flush=True)
    return True




# Source: yolo_click_tracker\src\performance.py
"""Bounded timing samples; display work never queues behind old frames."""
class StageTimings:
    def __init__(self, capacity=120):
        self.capacity = capacity
        self.lock = threading.Lock()
        self.samples = {}

    def add(self, name, milliseconds):
        if not math.isfinite(milliseconds) or milliseconds < 0:
            return
        with self.lock:
            self.samples.setdefault(name, deque(maxlen=self.capacity)).append(milliseconds)

    def snapshot(self):
        with self.lock:
            copied = {name: list(values) for name, values in self.samples.items()}
        return {name: dict(samples=len(values), last_ms=round(values[-1], 2),
                           p50_ms=round(sorted(values)[math.ceil(.5*len(values))-1], 2),
                           p95_ms=round(sorted(values)[math.ceil(.95*len(values))-1], 2))
                for name, values in copied.items() if values}


class LatestPreview:
    """One in-flight render plus one replaceable pending frame, never a FIFO.

    The renderer owns its image until finished. Local clicks use only the frame
    which successfully reached Display.show, not the newer inference mailbox.
    """
    def __init__(self, render, fps, timings, clock=time.monotonic):
        self.render, self.period, self.timings, self.clock = render, 1/fps, timings, clock
        self.condition = threading.Condition()
        self.pending = self.displayed = None
        self.stopping = False
        self.error = None
        self.replaced = self.rendered = 0
        self.completed = deque(maxlen=60)
        self.thread = threading.Thread(target=self._run, name="local-preview", daemon=True)

    def offer(self, frame, image, selected, prediction):
        with self.condition:
            if self.stopping:
                return
            if self.pending is not None:
                self.replaced += 1
            self.pending = (frame, image, selected, prediction)
            self.condition.notify()

    def take(self):
        with self.condition:
            job, self.pending = self.pending, None
            return job

    def _run(self):
        deadline = 0.
        previous = None
        try:
            while True:
                with self.condition:
                    while not self.stopping:
                        wait = deadline-self.clock()
                        if (self.pending is not None or previous is not None) and wait <= 0:
                            break
                        self.condition.wait(max(.001, wait) if self.pending is not None or previous is not None else None)
                    if self.stopping:
                        return
                    fresh = self.pending is not None
                    job, self.pending = self.pending or previous, None
                started = self.clock()
                self.timings.add("capture_to_preview_start", max(0., started-job[0]["stamp"])*1000)
                self.render(*job)
                finished = self.clock()
                with self.condition:
                    self.displayed = job[0]
                    if fresh:
                        self.rendered += 1
                        self.completed.append(finished)
                self.timings.add("local_preview", (finished-started)*1000)
                self.timings.add("capture_to_display_submit", max(0., finished-job[0]["stamp"])*1000)
                # Start-to-start cadence, rather than adding another period to rendering time.
                previous = job
                deadline = started+(self.period if fresh else max(.2, self.period))
        except Exception as exc:
            self.error = exc

    def stats(self):
        with self.condition:
            completed = list(self.completed)
        fps = ((len(completed)-1)/(completed[-1]-completed[0])
               if len(completed)>1 and completed[-1]>completed[0] else 0.)
        return dict(local_preview_fps_actual=round(fps, 1), preview_replaced_frames=self.replaced,
                    preview_rendered_frames=self.rendered)

    def close(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify_all()
        if self.thread.ident is not None:
            self.thread.join(2.)
            if self.thread.is_alive():
                raise RuntimeError("Display thread did not stop; camera resources retained")


# Source: yolo_click_tracker\src\response.py
"""Bounded software response adjustment from real position error; no torque API."""


class ManualRateRamp:
    """Critically damped velocity response, bounded acceleration and jerk.

    Substeps keep the ramp stable with varying UART/control loop intervals.
    Explicit hold, feedback loss and hard limit handling may reset immediately.
    """
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self, axis=None):
        if axis is None:
            self.acceleration = [0., 0.]
        else:
            self.acceleration[axis] = 0.

    def advance(self, axis, velocity, desired, accel, dt, cap):
        if dt <= 0:
            return velocity
        steps = max(1, int(math.ceil(dt/.005)))
        h, omega = dt/steps, 2./self.c["manual_response_s"]
        a, jerk_cap = self.acceleration[axis], self.c["manual_jerk_deg_s3"][axis]
        for _ in range(steps):
            jerk = clip(omega*omega*(desired-velocity)-2*omega*a, -jerk_cap, jerk_cap)
            a = clip(a+jerk*h, -accel, accel)
            proposed = velocity+a*h
            velocity = clip(proposed, -cap, cap)
            if velocity != proposed:
                a = 0.  # The speed ceiling takes priority over smoothing.
        self.acceleration[axis] = a
        return velocity


class ResponseTuner:
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self):
        self.previous = None
        self.velocity = self.error = self.error_square = (0., 0.)
        self.scale = (1., 1.)
        self.accel = tuple(clip(.85*a, lo, a) for a,lo in zip(self.c["command_accel_deg_s2"], self.c["adaptive_accel_min"]))
        self.correction = (0., 0.)
        self.status = ("IDLE", "IDLE")

    def update(self, now, actual, commanded, requested, enabled, healthy=True):
        if not enabled or not healthy or commanded is None:
            self.reset()
            return
        error = tuple(q-a for q,a in zip(commanded, actual))
        self.error = error
        previous, self.previous = self.previous, (now, tuple(actual))
        if previous is None:
            return
        dt = now-previous[0]
        if not .005 <= dt <= .25:
            self.velocity = self.correction = (0., 0.)
            return
        alpha = 1-math.exp(-dt/.12)
        velocity = tuple(v+alpha*((a-b)/dt-v) for v,a,b in zip(self.velocity, actual, previous[1]))
        scales, accels, corrections, states = [], [], [], []
        squares = []
        for axis,(e,request,speed,scale,accel,position,limits) in enumerate(zip(error,requested,velocity,
                self.scale,self.accel,actual,self.c["limits"])):
            lo,hi = self.c["adaptive_scale_bounds"]
            minimum,maximum = self.c["adaptive_accel_min"][axis], self.c["command_accel_deg_s2"][axis]
            target_scale, target_accel, correction, status = 1., .85*maximum, 0., "STABLE"
            direction = 1 if request >= 0 else -1
            progress, lag = speed*direction, e*direction
            near_limit = min(position-limits[0], limits[1]-position) < self.c["manual_lead_deg"]
            if not self.c["adaptive_response"]:
                target_accel, status = maximum, "FIXED"
            elif abs(request) <= .5 or near_limit:
                status = "HOLD" if abs(request) <= .5 else "LIMIT_APPROACH"
            elif lag < -self.c["adaptive_overshoot_deg"] or progress > abs(request)*1.3+1:
                target_scale, target_accel, status = lo, minimum, "DAMPING"
                correction = clip(e*self.c["adaptive_correction_kp"], -self.c["adaptive_correction_cap_deg_s"], self.c["adaptive_correction_cap_deg_s"])
            elif lag > self.c["adaptive_lag_deg"]:
                if progress >= self.c["adaptive_min_progress_deg_s"]:
                    target_scale, target_accel, status = hi, maximum, "LAG_COMPENSATING"
                    correction = clip(e*self.c["adaptive_correction_kp"], -self.c["adaptive_correction_cap_deg_s"], self.c["adaptive_correction_cap_deg_s"])
                else:
                    # Lack of progress is not permission to push harder into an obstruction.
                    target_scale, target_accel, status = min(scale, 1.), minimum, "NO_PROGRESS"
            scale += clip(target_scale-scale, -self.c["adaptive_gain_slew_s"]*dt, self.c["adaptive_gain_slew_s"]*dt)
            accel += clip(target_accel-accel, -maximum*dt, maximum*dt)
            scales.append(clip(scale, lo, hi))
            accels.append(clip(accel, minimum, maximum))
            corrections.append(correction)
            states.append(status)
            squares.append(self.error_square[axis]+alpha*(e*e-self.error_square[axis]))
        self.velocity, self.scale, self.accel = velocity, tuple(scales), tuple(accels)
        self.correction, self.status, self.error_square = tuple(corrections), tuple(states), tuple(squares)

    def snapshot(self):
        return dict(error_deg=self.error, error_rms_deg=tuple(math.sqrt(max(0.,v)) for v in self.error_square),
                    measured_velocity_deg_s=self.velocity, drive_scale=self.scale,
                    software_accel_deg_s2=self.accel, rate_correction_deg_s=self.correction, status=self.status,
                    torque_feedback_available=False, driver_pid_modified=False)


# Source: yolo_click_tracker\src\angle_move.py
"""One-shot mechanical-angle experiment. All trajectory updates run on the UART worker."""


class AngleMove:
    def __init__(self):
        self.active = False
        self.state = "IDLE"
        self.goal = self.origin = None
        self.accepted = self.started = self.first_write = self.first_motion = self.completed = None
        self.duration = self.progress = 0.
        self.settled = None
        self.speed = None
        self.reason = None
        self.profile = 'software_quintic'

    def begin(self, origin, goal, speed, accepted, now, c):
        self.__init__()
        self.active = True
        self.state = "MOVING"
        self.origin, self.goal, self.speed = tuple(origin), tuple(goal), speed
        self.accepted, self.started = accepted, now
        # Quintic position interpolation has zero velocity and acceleration at
        # both ends; these are its exact derivative maxima, with a small margin.
        self.duration = max([.1]+[max(1.875*abs(b-a)/min(speed,cap),
                                       (5.774*abs(b-a)/accel)**.5,
                                       (60.*abs(b-a)/jerk)**(1./3.))
                                   for a,b,cap,accel,jerk in zip(origin,goal,c['manual_rate'],
                                                                c['manual_accel_deg_s2'],c['manual_jerk_deg_s3'])])

    def waypoint(self, dt, actual, lead):
        progress=min(self.duration,self.progress+dt)
        u=progress/self.duration
        blend=u*u*u*(10.+u*(-15.+6.*u))
        target=tuple(a+(b-a)*blend for a,b in zip(self.origin,self.goal))
        if any(abs(q-a)>cap for q,a,cap in zip(target,actual,lead)):
            return None  # Pause trajectory time until feedback catches up.
        self.progress=progress
        velocity=tuple((b-a)*30.*u*u*(1.-u)*(1.-u)/self.duration for a,b in zip(self.origin,self.goal))
        return target,velocity

    def observe(self, now, actual, velocity, tolerance, allow_timeout=True):
        if self.first_motion is None and max(abs(a-b) for a,b in zip(actual,self.origin))>=.3:
            self.first_motion=now
        if self.progress>=self.duration:
            self.state='SETTLING'
            at_goal=(all(abs(a-b)<=tolerance for a,b in zip(actual,self.goal))
                     and max(abs(v) for v in velocity)<3.)
            self.settled=(now if self.settled is None else self.settled) if at_goal else None
            if self.settled is not None and now-self.settled>=.25:
                self.active=False;self.state='COMPLETE';self.completed=now
        if allow_timeout and self.active and now-self.started>self.duration*3.+5.:
            raise RuntimeError('ANGLE_MOVE_TIMEOUT: target did not settle')

    def cancel(self, reason):
        if self.active:
            self.active=False;self.state='CANCELLED';self.reason=reason

    def snapshot(self, pending=None):
        latency=lambda t:None if t is None or self.accepted is None else (t-self.accepted)*1000.
        return dict(active=self.active or pending is not None,state='PENDING' if pending else self.state,profile=self.profile,
                    target_deg=pending[:2] if pending else self.goal,speed_deg_s=pending[2] if pending else self.speed,
                    planned_duration_s=self.duration,progress_s=self.progress,
                    queue_ms=latency(self.started),first_write_ms=latency(self.first_write),
                    first_motion_ms=latency(self.first_motion),completed_ms=latency(self.completed),reason=self.reason)


# Source: yolo_click_tracker\src\native_tracking.py
"""Latest absolute visual aim, retargeted inside F32C's single-turn planner."""


def velocity_damped_goal(goal, measured_velocity, target_velocity, c):
    correction=clip((measured_velocity-target_velocity)*c['native_velocity_damping_s'],
                    -c['native_damping_max_deg'],c['native_damping_max_deg'])
    return goal-correction,correction


class NativeTrackingStream:
    def __init__(self, driver, c):
        self.driver,self.c=driver,c
        self.engaged=[False,False]
        self.message=None
        self.last_send=0.
        self.observation_stamp=None
        self.offered_at=None
        self.gap_held=[False,False]
        self.centered=[False,False]
        self.setup_count=self.gap_count=0
        self.accepted_aim=self.aim_candidate=None
        self.aim_rejections=0
        self.previous_pose=[None,None]
        self.velocity=[0.,0.]
        self.damping=[0.,0.]

    def offer(self, goal, stamp, active, velocity=(0.,0.)):
        if not math.isfinite(stamp):raise ValueError('Invalid native observation timestamp')
        if stamp!=self.observation_stamp:self.offered_at=time.monotonic()
        self.observation_stamp=stamp
        if goal is None:
            self.message=None;return
        if (len(goal)!=2 or len(active)!=2 or len(velocity)!=2 or any(not math.isfinite(v) for v in velocity) or not math.isfinite(stamp)
                or any(not math.isfinite(q) for q in goal)):
            raise ValueError('Invalid native visual aim')
        goal=tuple(goal)
        if self.accepted_aim is not None and stamp>self.accepted_aim[1]:
            old,old_stamp=self.accepted_aim
            dt=min(.25,stamp-old_stamp)
            limit=self.c['motor_rpm']*6*dt+4.
            if any(abs(q-p)>limit for q,p in zip(goal,old)):
                candidate=self.aim_candidate
                if candidate is not None and candidate[1]==stamp:return
                self.aim_candidate=(goal,stamp)
                if not (candidate is not None and 0<stamp-candidate[1]<=.15
                        and all(abs(q-p)<=self.c['motor_rpm']*6*(stamp-candidate[1])+4. for q,p in zip(goal,candidate[0]))):
                    self.aim_rejections+=1
                    return  # Do not chase a single-frame world-aim discontinuity.
        self.accepted_aim=(goal,stamp);self.aim_candidate=None
        self.message=(goal,stamp,tuple(bool(v) for v in active),self.offered_at,tuple(velocity))

    def sample_velocity(self):
        d=self.driver
        for axis in (0,1):
            stamp=d.mechanical_feedback_times[axis]
            previous=self.previous_pose[axis]
            if stamp is None or (previous is not None and stamp<=previous[1]):continue
            if previous is not None:
                dt=stamp-previous[1]
                if .005<=dt<=.2:
                    rate=clip((d.angles[axis]-previous[0])/dt,-self.c['motor_rpm']*6*1.5,self.c['motor_rpm']*6*1.5)
                    self.velocity[axis]+=dt/(.03+dt)*(rate-self.velocity[axis])
                else:self.velocity[axis]=0.
            self.previous_pose[axis]=(d.angles[axis],stamp)

    def wanted(self):
        d=self.driver
        tracking=self.message is not None and d.response_context=='TRACK'
        gap=(d.response_context in ('LOST','COAST','HOLD') or (d.response_context=='TRACK' and self.message is None)) and any(self.engaged)
        return (self.c['native_tracking'] and (tracking or gap) and self.observation_stamp is not None
                and 0<=time.monotonic()-self.observation_stamp<=self.c['vision_max_age']
                and d.armed and not d.home_requested and not d.recovery_active
                and d.outside_hold_origin is None and not d.angle_move.active and d.angle_pending is None)

    def hold_gap(self):
        d=self.driver
        for axis in (0,1):
            if self.engaged[axis] and not self.gap_held[axis]:
                self.position(axis,d.angles[axis])
                self.gap_held[axis]=True
                self.gap_count+=1
        d.stream_target=d.manual_target=d.sent_target
        d.stream_velocity=(0.,0.);d.last_send=time.monotonic()
        d.hold_pending=False;d.braking=d.soft_hold=False
        if d.response_context in ('HOLD','LOST','COAST') and d.hold_target is None:d._latch_hold(d.sent_target)

    def position(self, axis, q):
        d=self.driver;mid=self.c['motor_ids'][axis]
        packet=set_single_turn_position(mid,q/self.c['motor_signs'][axis])
        d._write(packet)
        units=int.from_bytes(packet[3:5],'big');d.position_units[axis]=units
        target=list(d.sent_target);target[axis]=decoded_position(self.c,axis,units,d.angles[axis])
        d.sent_target=tuple(target);d.last_position_send=time.monotonic()

    def advance(self):
        d=self.driver;d._heartbeat_check();d._require_reference(fresh=True)
        self.sample_velocity()
        with d.lock:message=self.message
        if not self.wanted():
            self.restore();return
        if message is None or d.response_context in ('LOST','COAST','HOLD'):
            self.hold_gap()
            if d._fixed_hold_active():d._advance_fixed_hold()
            return
        for axis,mid in enumerate(self.c['motor_ids']):
            if not self.engaged[axis]:
                self.position(axis,d.angles[axis])
                self.engaged[axis]=True
                d._write(select_mode(mid,2))
                d._write(set_acceleration(mid,self.c['native_tracking_accel'][axis]))
                rpm=int(math.ceil(native_tracking_rates(self.c)[axis]/6.))
                d._write(set_speed(mid,rpm));d.programmed_rpm[axis]=rpm
                self.setup_count+=1
        now=time.monotonic()
        if not self.wanted():
            self.restore();return
        if self.message is None or d.response_context in ('LOST','COAST','HOLD'):
            self.hold_gap();return
        # Mode/speed setup can span several visual frames. Use the newest aim
        # after it completes rather than sending the pre-setup observation.
        with d.lock:message=self.message
        if message is None or not 0<=time.monotonic()-message[1]<=self.c['vision_max_age']:
            self.restore();return
        goal,stamp,active,offered_at,target_velocity=message
        self.gap_held=[False,False]
        if now-self.last_send>=self.c['native_target_period_s']:
            for axis,(q,actual,(lo,hi),moving) in enumerate(zip(goal,d.angles,self.c['limits'],active)):
                if axis==1 and self.c.get('overhead_tracking',False):lo=max(lo,-90.)
                relative_velocity=self.velocity[axis]-target_velocity[axis]
                braking=self.c['native_velocity_damping_s']>0 and abs(relative_velocity)>3.
                if not moving and self.centered[axis] and not braking:continue
                q,self.damping[axis]=velocity_damped_goal(q,self.velocity[axis],target_velocity[axis],self.c)
                lead=self.c['native_target_window_deg'][axis]
                q=clip(clip(q,lo,hi),actual-lead,actual+lead)
                if abs(q-d.sent_target[axis])>=self.c['native_target_deadband_deg']:
                    self.position(axis,q)
                    d.timings.add('native_goal_age_at_send',(time.monotonic()-stamp)*1000)
                    d.timings.add('native_offer_to_write',(time.monotonic()-offered_at)*1000)
                self.centered[axis]=not moving and not braking
            self.last_send=now
        d.stream_target=d.manual_target=d.sent_target
        d.stream_velocity=tuple(clip(v*r,-cap,cap) for v,r,cap in
                                 zip(d.joystick,self.c['max_rate'],native_tracking_rates(self.c)))
        d.last_send=now;d.hold_target=None;d.hold_pending=False;d.braking=d.soft_hold=False

    def restore(self):
        d=self.driver
        if not any(self.engaged):return True
        if d.armed and (not d.reference_valid or any(d.feedback_issues)
                        or time.monotonic()-d.feedback_time>=self.c['reference_max_age_s']):
            return False
        if d.armed and max(abs(v) for v in d.response.velocity)>8.:
            self.hold_gap()
            return False  # Brake in T mode before switching a moving axis to direct mode.
        for axis,mid in enumerate(self.c['motor_ids']):
            if not self.engaged[axis]:continue
            q=d.angles[axis]
            if d.armed:self.position(axis,q)
            d._write(select_mode(mid,self.c['position_mode']))
            d._write(set_acceleration(mid,self.c['motor_acceleration']))
            if d.armed:self.position(axis,q)
            self.engaged[axis]=False
            self.gap_held[axis]=False
            self.centered[axis]=False
        d.stream_target=d.manual_target=d.sent_target;d.stream_velocity=(0.,0.)
        self.accepted_aim=self.aim_candidate=None
        self.previous_pose=[None,None];self.velocity=[0.,0.];self.damping=[0.,0.]
        d.manual_ramp.reset();d.tracking_ramp.reset();d.hold_pending=False
        if d.armed and not any(d.joystick):d._latch_hold(d.sent_target)
        return True


def speed_rpm_to_logical_rate(c, axis, rpm):
    """Decode signed speed using speed-mode polarity and the encoder-axis sign."""
    return rpm * 6. * c['motor_signs'][axis] * c['speed_mode_signs'][axis]


def logical_rate_to_speed_rpm(c, axis, rate):
    """Truncate toward zero; the speed-mode sign is independent of visual input."""
    return int(rate / speed_rpm_to_logical_rate(c, axis, 1))


class SpeedTrackingStream(NativeTrackingStream):
    """Visual bearing error -> signed RPM, with zero-speed handoff to position hold."""
    def __init__(self, driver, c):
        super().__init__(driver, c)
        self.command_velocity = [0., 0.]
        self.command_rpm = [0, 0]
        self.control_time = None

    def stop(self):
        # Zero speed must remain writable when feedback or the UI heartbeat is stale.
        for axis, mid in enumerate(self.c['motor_ids']):
            if self.engaged[axis]:
                self.driver._write(set_speed(mid, 0))
        self.command_velocity = [0., 0.]
        self.command_rpm = [0, 0]
        self.driver.stream_velocity = (0., 0.)
        self.control_time = None

    def hold_gap(self):
        self.stop()
        self.gap_count += 1
        self.restore()

    def advance(self):
        d = self.driver
        d._heartbeat_check(); d._require_reference(fresh=True)
        self.sample_velocity()
        if not self.wanted():
            self.restore(); return
        with d.lock: message = self.message
        if message is None or d.response_context != 'TRACK':
            self.hold_gap(); return
        for axis, mid in enumerate(self.c['motor_ids']):
            if not self.engaged[axis]:
                d._write(set_speed(mid, 0))  # Clear the old position-mode speed before changing modes.
                self.engaged[axis] = True
                d.position_intent[axis] = None  # A speed command has no position corridor.
                d._write(select_mode(mid, MODE_SPEED))
                d._write(set_acceleration(mid, self.c['motor_acceleration']))
                self.setup_count += 1
        now = time.monotonic()
        # Re-read the latest mailbox after UART setup, which may span visual frames.
        with d.lock: message = self.message
        if not self.wanted() or message is None or d.response_context != 'TRACK':
            self.hold_gap(); return
        if now - self.last_send < self.c['native_target_period_s']: return
        goal, stamp, active, offered_at, target_velocity = message
        if not 0 <= now - stamp <= self.c['vision_max_age']:
            self.hold_gap(); return
        dt = .02 if self.control_time is None else clip(now-self.control_time, .001, .1)
        self.control_time = now
        rates = []
        targets = []
        for axis, (q, actual, moving, ff, cap, accel) in enumerate(zip(
                goal, d.angles, active, target_velocity, native_tracking_rates(self.c),
                self.c['tracking_accel_deg_s2'])):
            accel = min(accel, self.c['motor_acceleration']*6.)
            lo, hi = self.c['limits'][axis]
            if axis == 1 and self.c.get('overhead_tracking', False): lo = max(lo, -90.)
            q = clip(q, lo, hi)
            error = q-actual
            # Centered stationary detections request zero; a moving aim retains feedforward.
            requested = (self.c['speed_tracking_kp'][axis]*error + ff
                         - self.c['speed_tracking_kd'][axis]*(self.velocity[axis]-ff))
            if not moving and abs(ff) < 1.: requested = 0.
            requested = clip(requested, -cap, cap)
            v = self.command_velocity[axis] + clip(requested-self.command_velocity[axis], -accel*dt, accel*dt)
            # Reserve feedback age and braking distance inside the pitch envelope.
            distance = (hi-actual if v > 0 else actual-lo)-self.c['speed_tracking_margin_deg']
            reaction = self.c['reference_max_age_s'] + self.c['native_target_period_s']
            safe = max(0., math.sqrt((accel*reaction)**2+2*accel*max(0.,distance))-accel*reaction)
            v = clip(v, -safe, safe)
            self.command_velocity[axis] = v
            # Truncate toward zero: integer RPM must never round beyond a safety bound.
            rpm = logical_rate_to_speed_rpm(self.c, axis, v)
            d._write(set_speed(self.c['motor_ids'][axis], rpm))
            self.command_rpm[axis] = rpm
            d.programmed_rpm[axis] = rpm
            rates.append(speed_rpm_to_logical_rate(self.c, axis, rpm)); targets.append(q)
        self.last_send = d.last_send = now
        d.sent_target = d.stream_target = d.manual_target = tuple(targets)
        d.stream_velocity = tuple(rates)
        d.hold_target = None; d.hold_pending = False
        d.braking = d.soft_hold = False
        d.timings.add('native_goal_age_at_send', (time.monotonic()-stamp)*1000)
        d.timings.add('native_offer_to_write', (time.monotonic()-offered_at)*1000)

    def restore(self):
        d = self.driver
        if not any(self.engaged): return True
        self.stop()
        if d.armed:
            if (not d.reference_valid or any(d.feedback_issues)
                    or time.monotonic()-d.feedback_time >= self.c['reference_max_age_s']):
                return False
            self.sample_velocity()
            if max(abs(v) for v in self.velocity) > 8.: return False
        # Switch only after stopping. Preload a local position reference in speed mode;
        # this packet sets the next mode's target and does not command speed.
        for axis, mid in enumerate(self.c['motor_ids']):
            if not self.engaged[axis]: continue
            if d.armed: self.position(axis, d.angles[axis])
            d._write(select_mode(mid, self.c['position_mode']))
            if d.armed: self.position(axis, d.angles[axis])
            self.engaged[axis] = False
            d._write(set_acceleration(mid, self.c['motor_acceleration']))
            d.programmed_rpm[axis] = None
        d.stream_target = d.manual_target = d.sent_target = tuple(d.angles)
        d.stream_velocity = (0., 0.)
        self.accepted_aim = self.aim_candidate = None
        self.previous_pose = [None, None]; self.velocity = [0., 0.]
        self.gap_held = [False, False]; self.centered = [False, False]
        d.manual_ramp.reset(); d.tracking_ramp.reset(); d.hold_pending = False
        if d.armed:
            d._sync_driver_speed()
            if not any(d.joystick): d._latch_hold(d.sent_target)
        return True


# Source: yolo_click_tracker\src\tracking_motor.py
"""Separate ARM/hold and HOME, with bounded recovery and motion diagnostics."""
from collections import deque


class FeedbackUnavailable(RuntimeError):
    pass


class MotionProtectionFault(RuntimeError):
    pass


class TrackingReplan(RuntimeError):
    pass


class TrackingMotionGuard(MotionGuard):
    def __init__(self, c):
        super().__init__(c)
        self.last_event = None
        self.request_windows = [None, None]
        self.paused_at = None
        self.reverse_since = [None, None]
        self.warning = None
        self.tracking_samples = deque(maxlen=256)
        self.tracking_error_since = [None,None]
        self.tracking_reverse = [None,None]

    def reset(self):
        super().reset()
        self.request_windows = [None, None]
        self.paused_at = None
        self.reverse_since = [None, None]
        self.warning = None
        self.tracking_samples.clear()
        self.tracking_error_since = [None,None]
        self.tracking_reverse = [None,None]

    def update_tracking(self, now, actual, commanded, planned):
        """Moving targets need physical-motion evidence, not point-error direction."""
        self.warning = None
        self.tracking_samples.append((now,tuple(actual)))
        duration = self.c["motion_grace_s"]+self.c["motion_window_s"]
        while len(self.tracking_samples)>1 and now-self.tracking_samples[1][0] >= duration:
            self.tracking_samples.popleft()
        for axis,(position,goal,rate) in enumerate(zip(actual,commanded,planned)):
            error = goal-position
            if abs(error) < self.c["tracking_error_deg"]:
                self.tracking_error_since[axis] = None
            elif self.tracking_error_since[axis] is None:
                self.tracking_error_since[axis] = now
            positions = [row[1][axis] for row in self.tracking_samples]
            since = self.tracking_error_since[axis]
            if (since is not None and now-since >= duration and now-self.tracking_samples[0][0] >= duration
                    and max(positions)-min(positions) < self.c["motion_progress_deg"]):
                reason = "SUSPECTED_BLOCK: tracking axis %d has no physical movement for %.1fs" % (axis,duration)
                self.last_event = dict(reason=reason,time=now,actual=tuple(actual),commanded=tuple(commanded))
                raise RuntimeError(reason)
            # Overshoot and planned braking are not evidence of wrong wiring.
            direction = 1 if rate > 0 else -1
            if abs(rate) < 2 or error*direction < self.c["tracking_error_deg"]:
                self.tracking_reverse[axis] = None
                continue
            window = self.tracking_reverse[axis]
            if window is None or window[2] != direction or (position-window[1])*direction >= self.c["motion_progress_deg"]:
                self.tracking_reverse[axis] = (now,position,direction,None)
                continue
            progress = (position-window[1])*direction
            if now-window[0] >= self.c["motion_grace_s"] and progress <= -self.c["tracking_reverse_deg"]:
                first = now if window[3] is None else window[3]
                self.tracking_reverse[axis] = (*window[:3],first)
                self.warning = "Tracking axis %d reverse transient; observing" % axis
                if now-first >= self.c["tracking_reverse_persist_s"]:
                    reason = "SUSPECTED_BLOCK: tracking axis %d persistently moving opposite planned motion" % axis
                    self.last_event = dict(reason=reason,time=now,actual=tuple(actual),commanded=tuple(commanded))
                    raise RuntimeError(reason)
            elif window[3] is not None:
                self.tracking_reverse[axis] = (*window[:3],None)

    def pause(self, now):
        if self.paused_at is None:
            self.paused_at = now

    def resume(self, now):
        if self.paused_at is not None:
            duration = max(0., now-self.paused_at)
            for windows in (self.windows, self.request_windows):
                for axis,window in enumerate(windows):
                    if window is not None:
                        windows[axis] = (window[0]+duration, *window[1:])
            self.reverse_since = [None if t is None else t+duration for t in self.reverse_since]
            self.tracking_samples = deque(((t+duration,p) for t,p in self.tracking_samples),maxlen=256)
            self.tracking_error_since = [None if t is None else t+duration for t in self.tracking_error_since]
            self.tracking_reverse = [None if w is None else (w[0]+duration,w[1],w[2],None if w[3] is None else w[3]+duration) for w in self.tracking_reverse]
            self.paused_at = None

    def check_request(self, now, actual, commanded, requested, homing=False):
        for axis,(position,request,limits) in enumerate(zip(actual,requested,self.c["limits"])):
            # A residual below the position-error guard is not evidence of a
            # blockage, even while the home planner still requests motion.
            at_goal = abs(position) <= max(self.c["home_tolerance_deg"], self.c["motion_error_deg"]) if homing else (
                (request < 0 and position <= limits[0]+self.c["home_tolerance_deg"]) or
                (request > 0 and position >= limits[1]-self.c["home_tolerance_deg"]))
            if abs(request) < 1. or at_goal:
                self.request_windows[axis] = None
                continue
            direction = 1 if request > 0 else -1
            window = self.request_windows[axis]
            if window is None or window[2] != direction or (position-window[1])*direction >= self.c["motion_progress_deg"]:
                self.request_windows[axis] = (now, position, direction)
            elif now-window[0] >= self.c["motion_grace_s"]+self.c["motion_window_s"]:
                reason = "Axis %d requested movement has no physical progress" % axis
                self.last_event = dict(reason=reason,time=now,actual=tuple(actual),commanded=commanded)
                raise RuntimeError("SUSPECTED_BLOCK: "+reason)

    def update(self, now, actual, commanded):
        self.warning = None
        try:
            for axis,(position,goal) in enumerate(zip(actual,commanded)):
                error = goal-position
                if abs(error) < self.c["motion_error_deg"]:
                    self.windows[axis] = self.reverse_since[axis] = None
                    continue
                direction = 1 if error > 0 else -1
                window = self.windows[axis]
                if window is None or window[2] != direction:
                    self.windows[axis] = (now,position,direction)
                    self.reverse_since[axis] = None
                    continue
                elapsed,progress = now-window[0],(position-window[1])*direction
                if elapsed < self.c["motion_grace_s"]:
                    continue
                if progress <= -self.c["motion_reverse_deg"]:
                    if self.reverse_since[axis] is None:
                        self.reverse_since[axis] = now
                    if now-self.reverse_since[axis] >= self.c["motion_reverse_persist_s"]:
                        raise RuntimeError("Axis %d moving opposite commanded direction" % axis)
                    self.warning = "Axis %d brief reverse movement; observing" % axis
                else:
                    self.reverse_since[axis] = None
                if elapsed >= self.c["motion_grace_s"]+self.c["motion_window_s"]:
                    if progress < self.c["motion_progress_deg"]:
                        raise RuntimeError("Axis %d motion stalled (position heuristic)" % axis)
                    self.windows[axis] = (now,position,direction)
        except RuntimeError as exc:
            self.last_event = dict(reason=str(exc), time=now,
                                   actual=tuple(actual), commanded=tuple(commanded))
            raise RuntimeError("SUSPECTED_BLOCK: %s; actual=%s target=%s; DISARM requested" % (
                str(exc), tuple(round(v, 2) for v in actual), tuple(round(v, 2) for v in commanded))) from exc


class VisualMotorService(MechanicalOnlyService):
    def __init__(self, c, transport_factory=None):
        c=dict(c)
        self.configured_limits=c['limits']
        if c.get('continuous_yaw',False):
            c['limits']=((float('-inf'),float('inf')),tuple(c['limits'][1]))
        super().__init__(c, transport_factory)
        self.timings = StageTimings()
        self.last_measure_started = None
        self.guard = TrackingMotionGuard(c)
        self.arm_homing = False
        self.recovery_active = False
        self.recovery_origin = None
        self.startup_completed = False
        self.startup_initialized = self.startup_cancelled = False
        self.outside_hold_origin = None
        self.recovery_rate = self.c["startup_home_rate"]
        self.feedback_cache = [None, None]
        self.feedback_issues = [None, None]
        self.feedback_rejects = [0, 0]
        self.feedback_missing_since = [None, None]
        self.feedback_query_started = [None, None]
        self.last_feedback_fault = None
        self.recheck_previous_fault = None
        self.recheck_started = self.recheck_finished = None
        self.strict_feedback = False
        self.response = ResponseTuner(c)
        self.manual_ramp = ManualRateRamp(c)
        self.home_ramp = ManualRateRamp(dict(c,manual_jerk_deg_s3=c['home_jerk_deg_s3'],
                                            manual_response_s=c['home_response_s']))
        self.tracking_ramp = ManualRateRamp(dict(c,manual_jerk_deg_s3=c["tracking_jerk_deg_s3"],
                                                manual_response_s=c["tracking_response_s"]))
        self.brake_ramp = ManualRateRamp(dict(c,manual_jerk_deg_s3=c["brake_jerk_deg_s3"],
                                             manual_response_s=c["brake_response_s"]))
        self.braking = False
        self.soft_hold = False
        self.brake_started = None
        self.release_source = "MANUAL"
        self.angle_move = AngleMove()
        self.angle_pending = None
        self.angle_cancel = False
        self.inertia_test_active = False
        self.native_pitch_requested = self.native_pitch_engaged = False
        self.native_pitch_sent = self.native_restore_pending = False
        self.native_pitch_accel = 1
        self.native_tracker = (SpeedTrackingStream if c['tracking_motor_control']=='speed' else NativeTrackingStream)(self,c)
        self.hold_target = None
        self.hold_error_since = [None,None]
        self.hold_error_best = [None,None]
        self.hold_jump_candidate = [None,None]
        self.position_intent = [None, None]
        self.programmed_rpm = [None, None]
        self.position_write_time = [None, None]
        self.position_write_interval = [None, None]
        self.position_write_step = [None, None]
        self.position_write_count = [0, 0]
        self.telemetry_lock = threading.Lock()
        self.telemetry_history = deque(maxlen=c["telemetry_samples"])
        self.last_telemetry = 0.
        self.response_context = "MANUAL"
        self.motion_anomalies = {}
        self.motion_warning = None
        self.motion_paused = False
        self.last_disable_reason = None
        self.disable_history = deque(maxlen=8)

    def set_response_context(self, context):
        if context not in ("MANUAL", "TRACK", "COAST", "LOST", "HOLD"):
            raise ValueError("Invalid response context")
        if context != self.response_context:
            preserve = context in ("HOLD","LOST","COAST") and self.response_context in ("MANUAL","TRACK") and self.armed
            if not preserve:
                self.manual_ramp.reset()
                self.tracking_ramp.reset()
            self.guard.tracking_samples.clear()
            self.guard.tracking_error_since = [None,None]
            self.guard.tracking_reverse = [None,None]
        self.response_context = context
        if context == 'TRACK':
            self.hold_target = None
        if context in ("MANUAL","TRACK"):
            self.release_source = context

    def input(self, x, y):
        super().input(x,y)
        if x or y:
            self.hold_target = None
            if self.angle_move.active or self.angle_pending:
                self.angle_cancel = True
            if self.braking:
                ramp = self.tracking_ramp if self.response_context == "TRACK" else self.manual_ramp
                ramp.acceleration = list(self.brake_ramp.acceleration)
            self.braking = self.soft_hold = False

    def capture_pose(self):
        with self.lock:
            if (self.reference_valid and not any(self.feedback_issues)
                    and time.monotonic()-self.feedback_time <= self.c["pose_max_age_s"]):
                return self.angles
            return None

    def _fixed_hold_active(self):
        native_hold=(self.response_context in ('HOLD','LOST','COAST') and all(not engaged or held for engaged,held in
                     zip(self.native_tracker.engaged,self.native_tracker.gap_held)))
        return (self.hold_target is not None and self.armed and not self.recovery_active and not self.home_requested
                and self.outside_hold_origin is None and not self.angle_move.active and self.angle_pending is None
                and (not any(self.native_tracker.engaged) or native_hold) and not self.braking and not any(self.joystick) and self.response_context!='TRACK')

    def offer_tracking_goal(self, goal, stamp, active, velocity=(0.,0.)):
        with self.lock:self.native_tracker.offer(goal,stamp,active,velocity)

    def _heartbeat_check(self):
        if self.inertia_test_active:
            if self.abort_requested or self.stop_event.is_set():
                raise UserDisarm('Inertia experiment cancelled')
            return  # One-shot board trajectory does not depend on PC/vision heartbeat.
        return super()._heartbeat_check()

    def _latch_hold(self, target):
        self.hold_target=tuple(round(float(q),7)+0. for q in target)
        self.stream_target=self.manual_target=self.hold_target
        self.stream_velocity=(0.,0.)
        self.hold_error_since=[None,None];self.hold_error_best=[None,None]
        self.soft_hold=True

    def _advance_fixed_hold(self):
        self._heartbeat_check();self._require_reference(fresh=True);self._sync_driver_speed()
        now=time.monotonic();self.last_send=now
        # F32C's existing position loop holds this already-written reference.
        # Do not replace it with the hand-displaced encoder angle or a sliding
        # feedback-relative waypoint; that would move the equilibrium itself.
        self.stream_target=self.manual_target=self.hold_target
        self.stream_velocity=(0.,0.)
        for axis,(actual,goal) in enumerate(zip(self.angles,self.hold_target)):
            error=abs(goal-actual)
            if error<self.c['motion_error_deg']:
                self.hold_error_since[axis]=self.hold_error_best[axis]=None
                continue
            best=self.hold_error_best[axis]
            if best is None or error<best-self.c['motion_progress_deg']:
                self.hold_error_since[axis]=now;self.hold_error_best[axis]=error
            elif now-self.hold_error_since[axis]>=self.c['hold_block_timeout_s']:
                raise RuntimeError('SUSPECTED_BLOCK: fixed hold axis %d has no return progress for %.1fs' %
                                   (axis,self.c['hold_block_timeout_s']))

    def _position_lead_deg(self, axis):
        if self.inertia_test_active:
            lo,hi=self.c['limits'][axis]
            return hi-lo  # Test trajectory is not throttled by normal feedback lead.
        if self.native_tracker.engaged[axis]:
            return self.c['native_target_window_deg'][axis]
        if self.angle_move.active or self.angle_pending:
            return self.c["manual_position_lead_deg"][axis]
        if self.recovery_active:
            return self.c["home_position_lead_deg"][axis]
        if self.outside_hold_origin is not None:
            return self.c["manual_lead_deg"]
        if self.braking:
            return self.c["manual_position_lead_deg"][axis] if self.release_source=="MANUAL" else self.c["tracking_position_lead_deg"][axis]
        if self.response_context == "MANUAL" and self.joystick[axis]:
            return self.c["manual_position_lead_deg"][axis]
        if self.response_context == "TRACK":
            return self.c["tracking_position_lead_deg"][axis]
        return self.c["manual_lead_deg"]

    def _effective_accel(self):
        if self.recovery_active:
            return tuple(self.c['home_accel_deg_s2'])
        ceiling = (self.c["home_accel_deg_s2"] if self.recovery_active else
                   self.c["manual_accel_deg_s2"] if self.response_context == "MANUAL" else
                   self.c["tracking_accel_deg_s2"])
        if self.response_context in ("MANUAL", "TRACK") and not self.recovery_active:
            return tuple(ceiling)  # Fixed trajectory; feedback must not modulate it each packet.
        return tuple(a*min(1.,tuned/base) for a,tuned,base in
                     zip(ceiling,self.response.accel,self.c["command_accel_deg_s2"]))

    def _driver_rates(self):
        if self.inertia_test_active:
            speed=self.angle_pending[2] if self.angle_pending else self.angle_move.speed
            return (self.c['manual_rate'][0],speed)
        if self.angle_move.active or self.angle_pending:
            return self.c["manual_rate"]
        if self.recovery_active:
            return self.recovery_rate
        if self.native_tracker.wanted():
            return native_tracking_rates(self.c)
        if self._fixed_hold_active():
            return self.c['hold_return_rate']
        if self.outside_hold_origin is not None:
            return tuple(min(v,cap) for v,cap in zip(self.c["manual_rate"],(48.,36.)))
        if self.braking:
            return self.c["manual_rate"] if self.release_source=="MANUAL" else self.c["tracking_rate"]
        if self.response_context == "MANUAL":
            return self.c["manual_rate"]
        if self.response_context == "COAST":
            return self.c["prediction_coast_rate"]
        return self.c["tracking_rate"]

    def _sync_driver_speed(self):
        if isinstance(self.native_tracker, SpeedTrackingStream) and any(self.native_tracker.engaged):
            return  # In speed mode this opcode commands motion, not a position speed cap.
        for axis,(mid,rate) in enumerate(zip(self.c["motor_ids"],self._driver_rates())):
            rpm = min(self.c["motor_rpm"],max(1,int(math.ceil(rate/6.))))
            if self.programmed_rpm[axis] != rpm:
                self._write(set_speed(mid,rpm))
                self.programmed_rpm[axis] = rpm

    def _response_snapshot(self):
        result = self.response.snapshot()
        result["diagnostic_drive_scale"] = result["drive_scale"]
        result["diagnostic_rate_correction_deg_s"] = result["rate_correction_deg_s"]
        result.update(drive_scale=(1.,1.),rate_correction_deg_s=(0.,0.),adaptive_applied=False)
        result["software_accel_deg_s2"] = self._effective_accel()
        return result

    def request_arm(self, armed, reason="手动 DISARM／停止请求"):
        if not armed and not self.ready and self.phase == "CONNECTING":
            self.startup_cancelled = True
        super().request_arm(armed)
        if armed:
            self.last_disable_reason = None
        else:
            self.last_disable_reason = dict(reason=reason,time=time.monotonic(),phase=self.phase,
                actual_deg=self.angles,commanded_deg=self.sent_target,context=self.response_context)
            self.disable_history.append(dict(self.last_disable_reason))
            print("MOTOR_DISABLE_REQUEST "+json.dumps(self.last_disable_reason),flush=True)

    def _read(self, size=512, timeout=0):
        # Draining is nonblocking. A reply is nine bytes, so do not wait for
        # an impossible 512-byte batch before returning each valid response.
        started = time.monotonic()
        try:
            try:
                return self.serial.read(size, timeout=timeout)
            except TypeError:
                return self.serial.read(size, timeout)
        finally:
            self.timings.add("uart_read", (time.monotonic()-started)*1000)

    def _feedback_once(self, motor_id, kind):
        for _ in range(8):
            if not self._read():
                break
        else:
            raise RuntimeError("Motor UART input never drained")
        self.parser = FeedbackParser()
        if kind == FEEDBACK_MECHANICAL_ANGLE:
            self.feedback_query_started[self.c["motor_ids"].index(motor_id)] = time.monotonic()
        self._write(request_feedback(motor_id, kind))
        deadline = time.monotonic()+self.c["feedback_timeout"]
        received = bytearray()
        while not self.stop_event.is_set():
            # Read queued bytes BEFORE checking the deadline. NPU/Python scheduling
            # may delay this thread while the correct UART reply is already ready.
            data = self._read(9, 2)
            received.extend(data or b"")
            for mid,response_kind,raw in self.parser.feed(bytes(data or b"")):
                if mid == motor_id and response_kind == kind:
                    return raw if kind == FEEDBACK_SPEED else raw/10.
            if time.monotonic() >= deadline:
                break
            time.sleep(.001)
        raise FeedbackUnavailable("No valid feedback from motor %d type=%d RX=%s" % (motor_id,kind,bytes(received[-64:]).hex()))

    def _require_reference(self, fresh=False):
        if self.fault or not self.reference_valid:
            self._lose_reference("mechanical reference unavailable; RECHECK while disabled")
        if fresh and time.monotonic()-self.feedback_time >= self.c["reference_max_age_s"]:
            try:
                while True:
                    self._heartbeat_check()
                    self._read_measurement()  # Data age alone is not a failed query.
                    if not any(self.feedback_issues):
                        break  # A motion write must not proceed with a cached reply.
            except UserDisarm:
                raise
            except MotionProtectionFault as exc:
                self._latch_fault(str(exc))
                raise
            except Exception as exc:
                self._lose_reference(str(exc))
        for axis,since in enumerate(self.feedback_missing_since):
            if since is not None and time.monotonic()-since >= self.c["feedback_loss_grace_s"]:
                self._lose_reference("ID=%d continuous mechanical query failures for >=%.0fms" % (
                    self.c["motor_ids"][axis], self.c["feedback_loss_grace_s"]*1000))

    def _feedback(self, motor_id, kind=FEEDBACK_TOTAL_ANGLE):
        try:
            return super()._feedback(motor_id, kind)
        except FeedbackUnavailable:
            if kind == FEEDBACK_SPEED and self.armed:
                return math.nan  # Unknown speed must postpone HOME settling, never mean zero.
            raise

    def _unavailable_angle(self, axis, reason):
        self.feedback_issues[axis] = reason
        self.feedback_rejects[axis] += 1
        if self.feedback_missing_since[axis] is None:
            self.feedback_missing_since[axis] = self.feedback_query_started[axis] or time.monotonic()
        cached = self.feedback_cache[axis]
        if not self.strict_feedback and cached is not None and time.monotonic()-self.feedback_missing_since[axis] < self.c["feedback_loss_grace_s"]:
            return cached[0]  # Its ORIGINAL timestamp is retained; this is not a fresh sample.
        raise FeedbackUnavailable("ID=%d continuous mechanical query failures for %.0fms: %s" % (
            self.c["motor_ids"][axis], self.c["feedback_loss_grace_s"]*1000, reason))

    def _mechanical_feedback(self, mid):
        axis = self.c["motor_ids"].index(mid)
        try:
            raw = self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE)
        except FeedbackUnavailable as exc:
            return self._unavailable_angle(axis, str(exc))
        if not math.isfinite(raw) or not 0 <= raw <= 360:
            return self._unavailable_angle(axis, "Invalid mechanical feedback: %r" % raw)
        value, now = round(signed_mechanical(raw), 7), time.monotonic()
        previous = self.mechanical_samples[axis]
        pending_home = self.arm_requested and not self.armed and self.home_requested
        if previous is not None and ((axis==0 and self.c.get('continuous_yaw',False)) or self.recovery_active or pending_home or self.outside_hold_origin is not None):
            value = previous[1]+signed_mechanical(value-previous[1])
        if previous is not None and (self.armed or self.arm_requested) and not self.inertia_test_active:
            dt = max(0., now-previous[0])
            if self.feedback_missing_since[axis] is not None:
                # A rejected offset must not become believable just by waiting.
                dt = min(dt,max(0.,self.feedback_missing_since[axis]-previous[0]))
            # Position packets and feedback are sampled at different instants.
            # Credit at most one position-packet period for this phase offset,
            # only when a successfully written target provides an independent
            # bounded movement corridor. Never refresh a rejected sample's time.
            phase_slack = self.c["position_period_s"] if self.position_intent[axis] is not None else 0.
            bound = max(self.c["motor_rpm"]*6*(dt+phase_slack)+self.c["reference_jump_slack_deg"],
                        self.c["manual_lead_deg"]+self.c["recovery_margin_deg"])
            intent = self.position_intent[axis]
            if intent is not None:
                # A faster driver ceiling must not make an idle 20-degree
                # reference jump plausible. Trust only the last written target
                # corridor, plus the existing noise/recovery tolerances.
                travel = abs(signed_mechanical(intent/self.c["motor_signs"][axis]-previous[1]))
                allowance = self.c["reference_jump_slack_deg"]+(self.c["recovery_hard_margin_deg"] if self.recovery_active else 0.)
                if self.response_context in ("TRACK","LOST","COAST","HOLD") or self.angle_move.active:
                    allowance += min(self.c["tracking_feedback_margin_deg"],abs(self.stream_velocity[axis])*self.c["tracking_feedback_phase_s"])
                bound = min(bound,max(self.c["manual_lead_deg"]+self.c["recovery_margin_deg"],travel+allowance))
            if abs(value-previous[1]) > bound:
                # Fixed-position disturbance or near-zero HOME settling can
                # legitimately move faster than the outgoing position stream.
                # Accept no new target here. Reconfirm large encoder changes
                # with a second fresh consistent packet inside work limits.
                near_home = (self.recovery_active and self.sent_target is not None
                             and abs(self.sent_target[axis])<=self.c['home_tolerance_deg'])
                normal = self.c['limits'][axis][0]<=value*self.c['motor_signs'][axis]<=self.c['limits'][axis][1]
                candidate=self.hold_jump_candidate[axis]
                if normal and (self._fixed_hold_active() or near_home):
                    confirmed=(candidate is not None and 0<now-candidate[1]<=self.c['jump_confirm_window_s']
                               and abs(value-candidate[0])<=self.c['motor_rpm']*6*(now-candidate[1])+self.c['reference_jump_slack_deg'])
                    self.hold_jump_candidate[axis]=(value,now)
                    if confirmed:bound=abs(value-previous[1])
            if abs(value-previous[1]) > bound:
                self.last_angle_jump = dict(axis=axis, motor_id=mid, previous_deg=previous[1],
                                           current_deg=value, elapsed_s=dt, allowed_deg=bound)
                return self._unavailable_angle(axis, "Unexpected mechanical angle jump: %.1f->%.1f" % (previous[1],value))
        self.hold_jump_candidate[axis]=None
        self.feedback_cache[axis] = (value, now)
        self.feedback_issues[axis] = None
        self.feedback_missing_since[axis] = None
        with self.lock:
            values = list(self.mechanical)
            values[axis] = value
            self.mechanical = tuple(values)
            self.mechanical_feedback_times[axis] = now
        return value

    def _measure(self):
        started = time.monotonic()
        if self.last_measure_started is not None:
            self.timings.add("uart_cycle_interval", (started-self.last_measure_started)*1000)
        self.last_measure_started = started
        try:
            return self._measure_impl()
        finally:
            self.timings.add("uart_measure", (time.monotonic()-started)*1000)

    def _measure_impl(self):
        was_paused = self.motion_paused
        try:
            # Do not apply a 100ms age gate before giving a delayed UART thread
            # a chance to obtain the next real reply.
            if self.fault or not self.reference_valid:
                self._lose_reference("mechanical reference unavailable; RECHECK while disabled")
            result = self._read_measurement()
            self._require_reference(fresh=False)
        except MotionProtectionFault as exc:
            self._latch_fault(str(exc))
            raise
        except Exception as exc:
            if self.stop_event.is_set() or self.reference_fault:
                raise
            self._lose_reference(str(exc))
        degraded = any(self.feedback_issues)
        now = time.monotonic()
        self.motion_paused = bool(self.armed and (degraded or (not self.inertia_test_active and
                                  (self.motion_warning or now-self.command_time > self.c["control_pause_age_s"])) ))
        if self.motion_paused:
            if isinstance(self.native_tracker, SpeedTrackingStream): self.native_tracker.stop()
            self.manual_ramp.reset()
            self.tracking_ramp.reset()
            self.guard.pause(now)
        else:
            self.guard.resume(now)
        if was_paused and not self.motion_paused and self.armed:
            # Resume from the measured pose, without a queued movement or old ramp.
            self.stream_target = self.manual_target = self.hold_target if self._fixed_hold_active() else self.angles
            self.stream_velocity = (0., 0.)
            self.last_send = time.monotonic()
        requested = (tuple(-math.copysign(rate,q) if abs(q) > self.c["home_tolerance_deg"] else 0.
                           for q,rate in zip(self.angles,self.recovery_rate)) if self.recovery_active else
                     tuple(v*r for v,r in zip(self.joystick,self.c["max_rate"])))
        self.response.update(time.monotonic(), self.angles, self.sent_target, requested,
                             self.armed, healthy=not self.motion_paused)
        if self.armed and not self.motion_paused and not self.inertia_test_active:
            if self.response_context == "TRACK" and not self.recovery_active:
                self.guard.update_tracking(now,self.angles,self.sent_target,self.stream_velocity)
            else:
                self.guard.check_request(now, self.angles, self.sent_target, requested, self.recovery_active)
        self._record_telemetry(requested)
        return result

    def _record_telemetry(self, requested):
        now = time.monotonic()
        if now-self.last_telemetry < self.c["telemetry_period_s"]:
            return
        self.last_telemetry = now
        row = dict(t=now, phase=self.phase, control_context=self.response_context, armed=self.armed, actual_deg=self.angles,
                   commanded_deg=self.sent_target if self.armed else None, requested_rate_deg_s=requested,
                   planned_velocity_deg_s=self.stream_velocity, braking_active=self.braking,soft_hold=self.soft_hold,
                   angle_move=self.angle_move.snapshot(self.angle_pending),
                   inertia_test_active=self.inertia_test_active,
                   hold_target_deg=self.hold_target,hold_fixed_active=self._fixed_hold_active(),
                   manual_acceleration_deg_s2=tuple(self.manual_ramp.acceleration),
                   feedback_age_s=now-self.feedback_time, feedback_degraded=bool(any(self.feedback_issues)),
                   position_write_interval_s=tuple(self.position_write_interval),
                   position_write_step_deg=tuple(self.position_write_step),
                   position_write_count=tuple(self.position_write_count),
                   motion_warning=self.motion_warning, motion_paused=self.motion_paused,
                   response=self._response_snapshot())
        with self.telemetry_lock:
            self.telemetry_history.append(row)

    def telemetry(self):
        with self.telemetry_lock:
            rows = list(self.telemetry_history)
        return dict(schema=1, units="degrees, seconds", samples=rows,
                    notes="Only armed, fresh-feedback samples are used for software tuning. No torque/current feedback; driver PID unchanged.")

    def _initialize(self):
        angles = super()._initialize()  # Disabled reads accept positions outside work limits.
        self.startup_initialized = True
        if self.c["startup_auto_home"] and not self.startup_cancelled and not self.abort_requested:
            # User-authorized automatic startup uses the existing saved zero.
            # No zero-setting, parameter save or cumulative reset is performed.
            self.zero_confirmed = True
            self.arm_requested = True
            self.home_requested = True
            print("STARTUP_AUTO_HOME: automatic enable and bounded home from current pose")
        return angles

    def _capture_reference(self):
        self.reference_valid = False
        self.feedback_cache = [None, None]
        self.feedback_missing_since = [None, None]
        self.strict_feedback = True
        try:
            # Saved single-turn zero needs no stationary/multiturn alignment.
            # A gravity-loaded pitch may already be falling while disabled.
            for mid in self.c["motor_ids"]:
                self._mechanical_feedback(mid)
        finally:
            self.strict_feedback = False
        self.mechanical_samples = [None, None]
        self.reference_valid = True

    def _read_measurement(self):
        mechanical, stamps = [], []
        for axis, mid in enumerate(self.c["motor_ids"]):
            value = self._mechanical_feedback(mid)
            mechanical.append(value)
            stamps.append(self.feedback_cache[axis][1])
        angles = tuple(round(v*s, 7) for v,s in zip(mechanical, self.c["motor_signs"]))
        current_anomalies = {}
        def check_corridor(axis,q,lo,hi,soft,hard,label):
            deviation = max(lo-q,q-hi,0.)
            if deviation <= soft+1e-7:
                return
            reason = "%s axis %d outside %.1f..%.1f by %.2f deg" % (label,axis,lo,hi,deviation)
            if deviation > hard+1e-7:
                raise MotionProtectionFault("MOTION_HARD_LIMIT: "+reason)
            key = (label,axis)
            since = self.motion_anomalies.get(key,time.monotonic())
            current_anomalies[key] = since
            if time.monotonic()-since >= self.c["motion_violation_grace_s"]:
                raise MotionProtectionFault("MOTION_PATH_PERSISTENT: "+reason)
        if self.inertia_test_active:
            for axis,(q,(lo,hi)) in enumerate(zip(angles,self.c['limits'])):
                if not lo<=q<=hi:
                    raise MotionProtectionFault('INERTIA_ANGLE_LIMIT: axis %d %.1f outside %.1f..%.1f' % (axis,q,lo,hi))
        elif self.recovery_active and self.recovery_origin is not None:
            margin = self.c["recovery_margin_deg"]
            for axis, (q, start, (lo,hi)) in enumerate(zip(angles, self.recovery_origin,self.c['limits'])):
                # Saved-zero homing may settle across zero inside the ordinary
                # working range. An outside start still must not drift farther
                # outward; final position writes remain local and toward zero.
                low = start if start < -self.c['home_tolerance_deg'] else lo
                high = start if start > self.c['home_tolerance_deg'] else hi
                check_corridor(axis,q,low,high,margin,self.c["recovery_hard_margin_deg"],"Recovery")
        elif self.armed and self.outside_hold_origin is not None:
            for axis,(q,start,(lo,hi)) in enumerate(zip(angles,self.outside_hold_origin,self.c["limits"])):
                margin = self.c["limit_feedback_margin_deg"]+(self.c["manual_lead_deg"] if not lo <= start <= hi else 0.)
                hard = max(margin+.5,self.c["recovery_hard_margin_deg"] if not lo <= start <= hi else self.c["limit_hard_margin_deg"])
                check_corridor(axis,q,min(start,lo),max(start,hi),margin,hard,"Outside-range recovery")
            tol = self.c["limit_feedback_margin_deg"]
            if all(lo-tol <= q <= hi+tol for q,(lo,hi) in zip(angles,self.c["limits"])):
                self.outside_hold_origin = None
                self.manual_target = angles
        elif self.armed and not self.recovery_active:
            margin = self.c["limit_feedback_margin_deg"]
            for axis,(q,(lo,hi)) in enumerate(zip(angles,self.c["limits"])):
                check_corridor(axis,q,lo,hi,margin,self.c["limit_hard_margin_deg"],"Mechanical position outside normal work limits")
        self.motion_anomalies = current_anomalies
        self.motion_warning = "; ".join("%s axis %d: pausing to recheck" % key for key in current_anomalies) or None
        with self.lock:
            self.mechanical, self.angles = tuple(mechanical), angles
            self.raw_feedback = (None, None)
            self.feedback_time = min(stamps)
            self.mechanical_samples = list(zip(stamps, mechanical))
        return tuple(mechanical), angles

    def _write(self, data):
        started = time.monotonic()
        try:
            return self._write_impl(data)
        finally:
            self.timings.add("uart_transaction", (time.monotonic()-started)*1000)

    def _write_impl(self, data):
        speed_track = (isinstance(self.native_tracker, SpeedTrackingStream)
                       and data[1] in self.c['motor_ids']
                       and self.native_tracker.engaged[self.c['motor_ids'].index(data[1])])
        if speed_track and data[2] in (FUNCTION_MODE, FUNCTION_SPEED):
            if (len(data) != 7 or data[0] != FRAME_HEADER or data[-1] != FRAME_TAIL
                    or bcc_xor(data[:-2]) != data[-2]):
                raise RuntimeError('Invalid speed tracking frame')
            value = int.from_bytes(data[3:5], 'big', signed=data[2]==FUNCTION_SPEED)
            if data[2] == FUNCTION_MODE:
                if value not in (MODE_SPEED, self.c['position_mode']):
                    raise RuntimeError('Invalid speed tracking mode')
                if value == MODE_SPEED:
                    self._heartbeat_check(); self._require_reference(fresh=True)
                    if not self.armed or not self.arm_requested: raise UserDisarm('Speed mode requires ARM')
            elif value:
                self._heartbeat_check(); self._require_reference(fresh=True)
                if not self.armed or not self.arm_requested: raise UserDisarm('Speed requires ARM')
                if any(self.feedback_issues): raise FeedbackUnavailable('Speed requires fresh feedback')
                axis = self.c['motor_ids'].index(data[1])
                rate = speed_rpm_to_logical_rate(self.c, axis, value)
                lo, hi = self.c['limits'][axis]
                if axis == 1 and self.c.get('overhead_tracking', False): lo = max(lo, -90.)
                margin = self.c['speed_tracking_margin_deg']
                if (abs(rate) > native_tracking_rates(self.c)[axis]
                        or (rate > 0 and self.angles[axis] >= hi-margin)
                        or (rate < 0 and self.angles[axis] <= lo+margin)):
                    raise RuntimeError('Unsafe tracking speed')
            # Stop packets bypass stale feedback/heartbeat checks, but retain UART checks.
            return JoystickMotorService._write(self, data)
        native_test = data[1]==self.c['motor_ids'][1] and self.native_pitch_requested and self.inertia_test_active
        native_track = (data[1] in self.c['motor_ids'] and self.c['native_tracking']
                        and self.native_tracker.engaged[self.c['motor_ids'].index(data[1])])
        if (len(data)==7 and data[2]==FUNCTION_MODE and int.from_bytes(data[3:5],'big')==2 and (native_test or native_track)):
            if data[0]!=0x7A or data[-1]!=0x7B or bcc_xor(data[:-2])!=data[-2]:
                raise RuntimeError('Invalid native mode frame')
            return JoystickMotorService._write(self,data)  # UART worker's temporary single-turn T mode only.
        if data[2] in (3, FUNCTION_ENABLE) and any(self.feedback_issues):
            raise FeedbackUnavailable("Motion paused until fresh mechanical feedback returns")
        if data[2] == 3 and not self.recovery_active and self.outside_hold_origin is not None:
            origin = self.outside_hold_origin
            time.sleep(.002)
            self._heartbeat_check()
            self._require_reference(fresh=True)
            if not self.arm_requested:
                raise UserDisarm("Outside-range motion cancelled")
            if (len(data) != 7 or data[1] not in self.c["motor_ids"]
                    or data[0] != 0x7A or data[-1] != 0x7B or bcc_xor(data[:-2]) != data[-2]):
                raise RuntimeError("Invalid outside-range position transaction")
            axis = self.c["motor_ids"].index(data[1])
            units = int.from_bytes(data[3:5], "big")
            canonical = signed_mechanical(units/10.)*self.c["motor_signs"][axis]
            q = self.angles[axis]+signed_mechanical(canonical-self.angles[axis])
            actual = self.angles[axis]
            lo,hi = self.c["limits"][axis]
            local = abs(q-actual) <= self._position_lead_deg(axis)+.051
            inward = (lo <= q <= hi if lo <= actual <= hi else
                      hi-.051 <= q <= actual+.051 if actual > hi else
                      actual-.051 <= q <= lo+.051)
            margin = self.c["limit_feedback_margin_deg"]+(self.c["manual_lead_deg"] if not lo <= origin[axis] <= hi else 0.)
            if (units >= 3600 or not local or not inward or
                    not min(origin[axis],lo)-margin <= q <= max(origin[axis],hi)+margin):
                raise RuntimeError("Outside-range target must hold or make a local inward step")
            self._send_position(data,axis,q)
            return
        if data[2] != 3:
            return super()._write(data)
        time.sleep(.002)
        self._heartbeat_check()
        self._require_reference(fresh=True)
        self._heartbeat_check()  # A refresh may have overlapped a user DISARM.
        if not self.arm_requested:
            raise UserDisarm("No active ARM request")
        if len(data) != 7 or data[1] not in self.c["motor_ids"] or data[0] != 0x7A or data[-1] != 0x7B or bcc_xor(data[:-2]) != data[-2]:
            raise RuntimeError("Invalid recovery target frame")
        axis = self.c["motor_ids"].index(data[1])
        units = int.from_bytes(data[3:5], "big")
        actual = self.angles[axis]
        canonical = signed_mechanical(units/10.)*self.c["motor_signs"][axis]
        target = actual+signed_mechanical(canonical-actual)
        if not self.recovery_active:
            lo,hi = self.c["limits"][axis]
            if axis==0 and self.c.get('continuous_yaw',False):canonical=target
            if units >= 3600 or not lo <= canonical <= hi:
                raise RuntimeError("Unsafe single-turn step; no target sent")
            if abs(canonical-actual) > self._position_lead_deg(axis)+.051:
                if self.response_context in ("TRACK","LOST","COAST","HOLD") or self.angle_move.active:
                    raise TrackingReplan("Tracking feedback changed after planning; refresh local target")
                raise RuntimeError("Unsafe single-turn step; no target sent")
            self._send_position(data,axis,canonical)
            return
        if not self.arm_homing or not self.home_requested or self.recovery_origin is None:
            raise RuntimeError("Recovery target outside active homing transaction")
        start = self.recovery_origin[axis]
        margin = self.c["recovery_margin_deg"]
        if (units >= 3600 or not min(start, 0)-margin <= target <= max(start, 0)+margin
                or abs(target-actual) > self._position_lead_deg(axis)+.051
                or abs(target) > abs(actual)+.051 or target*actual < -.001):
            raise RuntimeError("Unsafe recovery step: only local motion toward mechanical zero allowed")
        self._send_position(data,axis,target)

    def _send_position(self, data, axis, target):
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete motor UART write")
        # Publish per-axis intent immediately: a refresh between the two axis
        # writes must already recognize motion toward the first sent target.
        now = time.monotonic()
        previous_time,previous_target = self.position_write_time[axis],self.position_intent[axis]
        self.position_write_interval[axis] = None if previous_time is None else now-previous_time
        self.position_write_step[axis] = None if previous_target is None else target-previous_target
        self.position_write_count[axis] += 1
        self.position_write_time[axis] = now
        self.position_intent[axis] = target

    def _arm_current(self):
        if not self.arm_requested or self.stop_event.is_set():
            raise UserDisarm("ARM homing cancelled")
        if self.home_requested:
            self._begin_home()
        while True:
            self._heartbeat_check()
            mechanical, angles = self._measure()
            if not any(self.feedback_issues):
                break  # Bounded by the per-axis loss deadline; no cached-pose enable.
        if not self.home_requested:
            self.outside_hold_origin = angles if any(not lo <= q <= hi for q,(lo,hi) in zip(angles,self.c["limits"])) else None
        for mid in self.c["motor_ids"]:
            self._write(select_mode(mid, self.c["position_mode"]))
        self._sync_driver_speed()
        for mid,angle in zip(self.c["motor_ids"], mechanical):
            self._write(set_acceleration(mid, self.c["motor_acceleration"]))
            self._write(set_single_turn_position(mid, angle))
        for mid in self.c["motor_ids"]:
            self._write(enable(mid))
        self.stream_target, self.stream_velocity = angles, (0., 0.)
        self.manual_ramp.reset()
        self.position_units = [int.from_bytes(set_single_turn_position(mid, angle)[3:5], "big")
                               for mid,angle in zip(self.c["motor_ids"], mechanical)]
        self.last_position_send = time.monotonic()
        self.hold_pending = False
        if not self.home_requested and self.outside_hold_origin is None:
            self._latch_hold(angles)
        print("ARM: current pose preloaded; HOME separately requested=%s" % self.home_requested)
        return angles

    def request_home(self):
        if not self.armed:
            raise RuntimeError("先 ARM 在当前位置使能，再点 HOME；HOME 不会自动使能")
        super().request_home()
        self._cancel_angle("HOME requested")
        self.braking = self.soft_hold = False

    def request_angle_move(self, yaw, pitch, speed, inertia_test=False, native_pitch=False, native_accel=3):
        if native_pitch and (type(native_accel) is not int or not 1<=native_accel<=5):
            raise ValueError('Native acceleration parameter must be integer 1..5')
        values=(yaw,pitch,speed)
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
            raise ValueError("角度与速度必须为有限数字")
        if self.c.get('continuous_yaw',False):
            yaw=round(self.angles[0]+signed_mechanical(yaw-self.angles[0]),1)
            values=(yaw,pitch,speed)  # Absolute heading takes the nearest equivalent turn.
        if not 1.<=speed<=min(self.c['manual_rate']):
            raise ValueError("测试速度须在 1～%g°/s 内" % min(self.c['manual_rate']))
        if any(not lo<=q<=hi for q,(lo,hi) in zip(values[:2],self.c['limits'])):
            raise ValueError("目标角度超出工作限位")
        with self.lock:
            if (not self.armed or not self.reference_valid or self.fault or self.home_requested or self.recovery_active
                    or self.outside_hold_origin is not None or any(self.feedback_issues)
                    or time.monotonic()-self.feedback_time>self.c['pose_max_age_s']
                    or any(not lo<=q<=hi for q,(lo,hi) in zip(self.angles,self.c['limits']))):
                raise RuntimeError("先使能、完成回零，并等待正常范围内的新鲜角度反馈")
            if self.angle_move.active or self.angle_pending or self.native_pitch_engaged or self.braking or any(self.joystick) or max(abs(v) for v in self.stream_velocity)>1.:
                raise RuntimeError("请等待当前运动停止，或先停止角度测试")
            self.angle_pending=(round(yaw,1),round(pitch,1),float(speed),time.monotonic())
            self.angle_cancel=False
            self.inertia_test_active=bool(inertia_test)
            self.native_pitch_requested=bool(native_pitch and inertia_test)
            self.native_pitch_accel=native_accel
            self.native_pitch_sent=self.native_restore_pending=False

    def cancel_angle_move(self):
        with self.lock:
            self.angle_cancel=True

    def _cancel_angle(self, reason):
        self.native_restore_pending=self.native_pitch_engaged
        self.inertia_test_active=self.native_pitch_engaged
        self.native_pitch_requested=False
        if self.angle_pending:
            self.angle_move.state='CANCELLED';self.angle_move.reason=reason
        self.angle_pending=None
        self.angle_move.cancel(reason)
        self.angle_cancel=False

    def _native_position(self, q):
        axis=1;mid=self.c['motor_ids'][axis]
        packet=set_single_turn_position(mid,q/self.c['motor_signs'][axis])
        self._write(packet)
        self.position_units[axis]=int.from_bytes(packet[3:5],'big')
        self.last_position_send=time.monotonic()
        target=list(self.sent_target);target[axis]=round(q,1);self.sent_target=tuple(target)

    def _restore_native_pitch(self, target=None):
        if not self.native_pitch_engaged:
            self.native_restore_pending=False
            return
        mid=self.c['motor_ids'][1]
        try:
            if self.armed:
                self._require_reference(fresh=True)
                q=self.angles[1] if target is None else target
                self._native_position(q)  # Preload before switching, without enable toggles.
            self._write(select_mode(mid,self.c['position_mode']))
            self._write(set_acceleration(mid,self.c['motor_acceleration']))
            if self.armed:self._native_position(q)
        finally:
            self.native_pitch_engaged=self.native_pitch_requested=False
            self.native_pitch_sent=self.native_restore_pending=False
            self.inertia_test_active=False

    def _advance_native_pitch(self, now):
        mid=self.c['motor_ids'][1];goal=self.angle_move.goal[1]
        if not self.native_pitch_sent:
            self._native_position(self.angles[1])
            self.native_pitch_engaged=True
            self._write(select_mode(mid,2))
            rpm=max(1,int(math.ceil(self.angle_move.speed/6.)))
            self._write(set_speed(mid,rpm));self.programmed_rpm[1]=rpm
            self._write(set_acceleration(mid,self.native_pitch_accel))
            self._native_position(goal)
            self.angle_move.first_write=time.monotonic()
            self.angle_move.profile='native_t'
            distance=abs(goal-self.angle_move.origin[1]);speed=rpm*6.;accel=360.*self.native_pitch_accel
            self.angle_move.duration=(2*(distance/accel)**.5 if distance<=speed*speed/accel else distance/speed+speed/accel)
            self.native_pitch_sent=True
        self.angle_move.progress=min(self.angle_move.duration,now-self.angle_move.started)
        self.stream_target=self.manual_target=self.sent_target;self.stream_velocity=(0.,0.)
        self.last_send=now
        self.angle_move.observe(now,self.angles,self.response.velocity,self.c['home_tolerance_deg'],allow_timeout=False)
        if not self.angle_move.active:
            self._restore_native_pitch(goal)
            self._latch_hold(self.sent_target)

    def _advance_angle_move(self):
        self._heartbeat_check();self._require_reference(fresh=True);self._sync_driver_speed()
        now=time.monotonic();dt=min(.1,max(0.,now-self.last_send))
        with self.lock:
            if self.angle_pending:
                yaw,pitch,speed,accepted=self.angle_pending
                self.angle_move.begin(self.angles,(yaw,pitch),speed,accepted,now,self.c)
                self.angle_pending=None
                self.braking=self.soft_hold=False
                self.manual_ramp.reset();self.tracking_ramp.reset()
        if self.native_pitch_requested:
            return self._advance_native_pitch(now)
        waypoint=self.angle_move.waypoint(dt,self.angles,tuple(self._position_lead_deg(a) for a in (0,1)))
        self.last_send=now
        if waypoint is not None and now-self.last_position_send>=self.c['position_period_s']:
            target,velocity=waypoint
            sent=list(self.sent_target)
            for axis,(mid,q,sign) in enumerate(zip(self.c['motor_ids'],target,self.c['motor_signs'])):
                packet=set_single_turn_position(mid,q/sign);units=int.from_bytes(packet[3:5],'big')
                if units!=self.position_units[axis]:
                    self._write(packet);self.position_units[axis]=units
                    self.last_position_send=now
                    if self.angle_move.first_write is None:self.angle_move.first_write=time.monotonic()
                    sent[axis]=decoded_position(self.c,axis,units,self.angles[axis])
            self.sent_target=tuple(sent)
            self.stream_target=target;self.stream_velocity=velocity
        self.manual_target=self.sent_target
        if not self.inertia_test_active:self.guard.update(now,self.angles,self.sent_target)
        self.angle_move.observe(now,self.angles,self.response.velocity,self.c['home_tolerance_deg'],
                                allow_timeout=not self.inertia_test_active)
        if not self.angle_move.active:
            self.inertia_test_active=False
            self.stream_target=self.manual_target=self.sent_target
            self.stream_velocity=(0.,0.);self.soft_hold=True
            self._latch_hold(self.sent_target)

    def _rebase_yaw_for_home(self):
        if not self.c.get('continuous_yaw',False):return
        shift=round(self.angles[0]-signed_mechanical(self.angles[0]),7)
        if not shift:return
        raw_shift=shift/self.c['motor_signs'][0]
        with self.lock:
            for name in ('angles','manual_target','sent_target','stream_target','hold_target'):
                values=getattr(self,name,None)
                if values is not None:setattr(self,name,(values[0]-shift,values[1]))
            self.mechanical=(self.mechanical[0]-raw_shift,self.mechanical[1])
            for name in ('feedback_cache','mechanical_samples'):
                values=list(getattr(self,name))
                if values[0] is not None:
                    if name=='feedback_cache':values[0]=(values[0][0]-raw_shift,values[0][1])
                    else:values[0]=(values[0][0],values[0][1]-raw_shift)
                setattr(self,name,values)
            if self.position_intent[0] is not None:self.position_intent[0]-=shift
            self.hold_jump_candidate[0]=None
        self.response.reset();self.guard.reset()

    def _begin_home(self):
        while any(self.native_tracker.engaged) and not self.native_tracker.restore():
            self._heartbeat_check();self._measure();time.sleep(.005)
        if self.native_pitch_engaged:self._restore_native_pitch()
        self._rebase_yaw_for_home()  # Coordinate-only shift: HOME uses nearest mechanical zero.
        self.home_ramp.reset()
        self.hold_target = None
        self.braking = self.soft_hold = False
        self.manual_ramp.reset()
        self.arm_homing = self.home_requested = self.recovery_active = True
        self.recovery_origin = self._measure()[1]
        self.outside_hold_origin = None
        outside = any(not lo <= q <= hi for q,(lo,hi) in zip(self.recovery_origin, self.c["limits"]))
        self.recovery_rate = self.c["startup_home_rate"] if outside else self.c["home_rate"]

    def _manual_phase(self, moving):
        if (self.angle_move.active or self.angle_pending) and not moving:
            return "MANUAL"  # Existing HOME authorization accepts this active position mode.
        if self.braking and not moving:
            return "BRAKING"
        return super()._manual_phase(moving)

    def _release_hold(self):
        if any(self.native_tracker.engaged):
            self.manual_target=self.stream_target=self.sent_target
            self.stream_velocity=(0.,0.);self.hold_pending=False
            return
        self.hold_target = None  # An intentional manual release defines a new landing pose.
        eligible = (self.was_moving and self.response_context in ("MANUAL","TRACK","HOLD","LOST","COAST")
                    and self.armed and self.reference_valid and not self.recovery_active and not self.home_requested
                    and self.outside_hold_origin is None and not any(self.feedback_issues)
                    and all(lo<=q<=hi for q,(lo,hi) in zip(self.angles,self.c["limits"]))
                    and any(abs(v)>.5 for v in self.stream_velocity))
        if not eligible:
            self.braking = self.soft_hold = False
            return super()._release_hold()
        self.braking,self.soft_hold = True,False
        self.brake_started=time.monotonic()
        source=self.tracking_ramp if self.release_source=="TRACK" else self.manual_ramp
        self.brake_ramp.acceleration=list(source.acceleration)
        # Blend the bounded trajectory speed with filtered encoder velocity;
        # noisy feedback may not reverse or increase the outgoing motion.
        self.stream_velocity=tuple(math.copysign(min(abs(v),max(0.,.7*abs(v)+.3*measured*(1 if v>0 else -1))),v)
                                   if v else 0. for v,measured in zip(self.stream_velocity,self.response.velocity))
        self.manual_target=self.stream_target
        self.hold_pending=False

    def _advance_braking(self):
        self._heartbeat_check();self._require_reference(fresh=True);self._sync_driver_speed()
        now=time.monotonic();dt=min(.1,max(0.,now-self.last_send))
        target,velocity=[],[]
        for axis,(old,v,actual,accel,bounds) in enumerate(zip(self.stream_target,self.stream_velocity,self.angles,
                                                          self.c["brake_accel_deg_s2"],self.c["limits"])):
            cap=self.c["manual_rate"][axis] if self.release_source=="MANUAL" else self.c["tracking_rate"][axis]
            new=self.brake_ramp.advance(axis,v,0.,accel,dt,cap)
            new=math.copysign(max(0.,new*(1 if v>0 else -1)),v) if v else 0.
            candidate=old+(v+new)*.5*dt
            bounded=clip(candidate,*bounds)
            q=clip(bounded,actual-self._position_lead_deg(axis),actual+self._position_lead_deg(axis))
            # Feedback lag can hold a waypoint, but ordinary braking must not
            # retract an already-sent target and pull against ongoing inertia.
            if v and (q-self.sent_target[axis])*v<0:
                q=clip(self.sent_target[axis],*bounds)
            if bounded!=candidate:
                new=0.;self.brake_ramp.reset(axis)
            target.append(q);velocity.append(new)
        self.stream_target,self.stream_velocity=tuple(target),tuple(velocity)
        self.last_send=now
        if now-self.last_position_send>=self.c["position_period_s"]:
            sent=list(self.sent_target)
            for axis,(mid,q,sign) in enumerate(zip(self.c["motor_ids"],target,self.c["motor_signs"])):
                packet=set_single_turn_position(mid,q/sign);units=int.from_bytes(packet[3:5],"big")
                if units!=self.position_units[axis]:
                    self._write(packet);self.position_units[axis]=units;self.last_position_send=now
                    sent[axis]=decoded_position(self.c,axis,units,self.angles[axis])
            self.sent_target=tuple(sent)
        self.guard.update(now,self.angles,self.sent_target)
        if max(abs(v) for v in self.stream_velocity)<.4 or now-self.brake_started>=self.c["brake_timeout_s"]:
            self.braking=False;self.soft_hold=True
            self.stream_target=self.manual_target=self.sent_target
            self.stream_velocity=(0.,0.);self.manual_ramp.reset();self.tracking_ramp.reset();self.brake_ramp.reset()
            self.hold_pending=False
            self._latch_hold(self.sent_target)

    def _advance(self, goal, rate):
        try:
            return self._advance_impl(goal,rate)
        except TrackingReplan:
            # Fresh feedback may move beyond an obsolete hold target when TRACK
            # switches to LOST/HOLD and its lead shrinks. Drop the unsent packet;
            # recapture a local hold on the next cycle, without revoking ARM.
            self.stream_target = self.manual_target = self.angles
            self.stream_velocity = (0.,0.)
            self.manual_ramp.reset()
            self.tracking_ramp.reset()
            self.hold_pending = self.response_context != "TRACK"
            self.last_send = time.monotonic()

    def _advance_impl(self, goal, rate):
        if self.native_restore_pending:self._restore_native_pitch()
        if any(self.native_tracker.engaged) and not self.native_tracker.wanted():
            if not self.native_tracker.restore():
                self._heartbeat_check();self._require_reference(fresh=False)
                return
            goal=self.manual_target
        pause = bool(any(self.feedback_issues) or (not self.inertia_test_active and
                     (self.motion_warning or time.monotonic()-self.command_time > self.c["control_pause_age_s"])))
        if pause:
            if isinstance(self.native_tracker, SpeedTrackingStream): self.native_tracker.stop()
            self.home_ramp.reset()
            self.braking = self.soft_hold = False
            self._heartbeat_check()
            self._require_reference(fresh=False)  # No motion write: retry on the next normal read cycle.
            self.last_send = time.monotonic()
            self.stream_velocity = (0., 0.)
            self.manual_ramp.reset()
            self.tracking_ramp.reset()
            self.motion_paused = True
            self.guard.pause(self.last_send)
            return  # Retain enable, but do not send a new position during the short grace.
        if self.motion_paused:
            self.motion_paused = False
            self.stream_target = self.manual_target = self.hold_target if self._fixed_hold_active() else self.angles
            self.stream_velocity = (0.,0.)
            self.guard.resume(time.monotonic())
        if not self.recovery_active:
            if self.native_tracker.wanted():
                return self.native_tracker.advance()
            if self.c['native_tracking'] and self.response_context=='TRACK' and self.angle_pending is None and not self.angle_move.active:
                self._heartbeat_check();self._require_reference(fresh=True)
                self.last_send=time.monotonic();self.stream_velocity=(0.,0.)
                return  # Do not fall back to old rate packets when the absolute aim is stale.
            if self.angle_cancel:
                active=self.angle_move.active or self.angle_pending is not None
                self._cancel_angle("Operator interrupted")
                if self.native_restore_pending:self._restore_native_pitch()
                if active:
                    was_moving=self.was_moving;self.was_moving=True
                    self.release_source="MANUAL"
                    self._release_hold();self.was_moving=was_moving
                    goal=self.manual_target  # Discard this cycle's obsolete pre-cancel goal.
            if self.angle_move.active or self.angle_pending:
                return self._advance_angle_move()
            if self.braking:
                return self._advance_braking()
            if self._fixed_hold_active():
                return self._advance_fixed_hold()
            if self.soft_hold and not any(self.joystick):
                self._heartbeat_check();self._require_reference(fresh=True)
                self.last_send=time.monotonic()
                self.guard.update(self.last_send,self.angles,self.sent_target)
                return  # Retain final landing target instead of chasing measured overshoot.
            if self.response_context == "TRACK" and self.outside_hold_origin is None:
                try:
                    return self._advance_velocity_stream()
                except TrackingReplan:
                    self.stream_target = self.manual_target = self.angles
                    self.stream_velocity = (0.,0.)
                    self.tracking_ramp.reset()
                    self.last_send = time.monotonic()
                    return
            if self.outside_hold_origin is not None:
                if any(self.joystick) or self.hold_pending:
                    return self._advance_velocity_stream()
                self._heartbeat_check()
                self._require_reference(fresh=True)
                self.last_send = time.monotonic()
                self.guard.update(self.last_send, self.angles, self.sent_target)
                return
            if self.phase == "MANUAL" and not self.hold_pending:
                return self._advance_velocity_stream()
            return super()._advance(goal, rate)
        self._heartbeat_check()
        self._require_reference(fresh=True)
        self._sync_driver_speed()
        if tuple(goal) != (0., 0.):
            raise RuntimeError("Only zero is allowed as a recovery goal")
        now = time.monotonic()
        dt = min(.1, max(0., now-self.last_send))
        target, velocity = [], []
        for axis,(old, old_v, actual, start, speed, accel) in enumerate(zip(self.stream_target, self.stream_velocity,
                self.angles, self.recovery_origin, self.recovery_rate, self._effective_accel())):
            desired = -math.copysign(min(speed, math.sqrt(2*accel*abs(old))), old) if old else 0.
            v = self.home_ramp.advance(axis,old_v,desired,accel,dt,speed)
            q = clip(old+(old_v+v)*.5*dt, min(old, 0.), max(old, 0.))
            q = clip(q, actual-self._position_lead_deg(axis), actual+self._position_lead_deg(axis))
            q = clip(q, min(start, 0.), max(start, 0.))
            q = clip(q, min(actual, 0.), max(actual, 0.))
            if q == 0:
                v = 0.
                self.home_ramp.reset(axis)
            target.append(q)
            velocity.append(v)
        self.stream_target, self.stream_velocity = tuple(target), tuple(velocity)
        self.last_send = now
        if now-self.last_position_send >= self.c["position_period_s"]:
            sent = list(self.sent_target)
            for axis, (mid, q, sign) in enumerate(zip(self.c["motor_ids"], target, self.c["motor_signs"])):
                packet = set_single_turn_position(mid, q/sign)
                units = int.from_bytes(packet[3:5], "big")
                if units != self.position_units[axis]:
                    self._write(packet)
                    self.last_position_send = now
                    self.position_units[axis] = units
                    canonical = signed_mechanical(units/10.)*sign
                    sent[axis] = self.angles[axis]+signed_mechanical(canonical-self.angles[axis])
            self.sent_target = tuple(sent)
        self.guard.update(now, self.angles, self.sent_target)

    def _advance_velocity_stream(self):
        """Continuous manual/tracking rate: do not brake at each short waypoint."""
        self._heartbeat_check()
        self._require_reference(fresh=True)
        self._sync_driver_speed()
        now = time.monotonic()
        dt = min(.1, max(0., now-self.last_send))
        with self.lock:
            vector = self.joystick
        force_hold = self.hold_pending
        target, velocity = [], []
        for axis,(old, old_v, actual, v, speed, accel, limits) in enumerate(zip(self.stream_target,
                self.stream_velocity, self.angles, vector, self.c["max_rate"],
                self._effective_accel(), self.c["limits"])):
            manual = self.response_context == "MANUAL"
            cap = (self.c["manual_rate"][axis] if manual else
                   self.c["prediction_coast_rate"][axis] if self.response_context == "COAST" else
                   self.c["tracking_rate"][axis])
            # The F32C already closes its position/velocity loops. Constant stick
            # input must stay constant; an additional error-derived gain/correction
            # loop turns ordinary following lag and overshoot into speed ripple.
            desired = clip(v*speed, -cap, cap)
            if manual:
                # Position-error damping may slow a stick command to zero, but
                # it must not reverse that command or move an idle axis.
                direction = 1. if v > 0 else -1.
                desired = direction*max(0.,desired*direction) if v else 0.
            lo,hi = limits
            if self.response_context in ("MANUAL","TRACK") and lo <= actual <= hi:
                # Brake before a working limit; retain the strict final target gate.
                distance = max(0., (hi-old) if desired > 0 else (old-lo))
                brake_accel = accel*self.c["adaptive_accel_min"][axis]/self.response.accel[axis]
                # Include time to reverse acceleration through the jerk ramp.
                ramp_s = (2*accel/self.c["manual_jerk_deg_s3"][axis]+self.c["manual_response_s"] if manual else
                          self.c["position_period_s"])
                brake_rate = math.sqrt((brake_accel*ramp_s)**2+2*brake_accel*distance)-brake_accel*ramp_s
                desired = clip(desired, -brake_rate, brake_rate)
            new_v = (self.manual_ramp.advance(axis, old_v, desired, accel, dt, cap) if manual else
                     self.tracking_ramp.advance(axis, old_v, desired, accel, dt, cap) if self.response_context == "TRACK" else
                     clip(desired, old_v-accel*dt, old_v+accel*dt))
            if not manual:
                new_v = clip(new_v,-cap,cap)  # COAST/selection never inherits a faster manual rate.
            candidate = old+(old_v+new_v)*.5*dt if manual else old+new_v*dt
            lo,hi = limits
            bounds = limits
            if self.outside_hold_origin is not None and actual > hi:
                bounds = (hi,max(actual,self.outside_hold_origin[axis]))
                new_v = min(0.,new_v)
                if desired >= 0:
                    candidate,new_v = actual,0.
                else:
                    candidate = old+new_v*dt
            elif self.outside_hold_origin is not None and actual < lo:
                bounds = (min(actual,self.outside_hold_origin[axis]),lo)
                new_v = max(0.,new_v)
                if desired <= 0:
                    candidate,new_v = actual,0.
                else:
                    candidate = old+new_v*dt
            if force_hold:
                candidate,new_v = actual,0.
            q = clip(clip(candidate, actual-self._position_lead_deg(axis), actual+self._position_lead_deg(axis)), *bounds)
            if (self.response_context in ("MANUAL","TRACK") and v and not force_hold and self.outside_hold_origin is None
                    and old_v*v >= 0 and new_v*v >= 0):
                # Small feedback retreats must not retract an already-written
                # forward target. If its lead is temporarily too large, retain
                # it without resending until real feedback catches up. Likewise
                # do not command behind an overshooting motor while the stick
                # is still asking it to move forward. Real reversals are allowed
                # after the rate ramp has decelerated through zero.
                anchor = max(self.sent_target[axis],actual) if v > 0 else min(self.sent_target[axis],actual)
                if (q-anchor)*v < 0:
                    q = clip(anchor,*bounds)
            if q != candidate and candidate != clip(candidate, *bounds):
                # Lead limiting is anti-windup, not a reason to restart the ramp
                # every cycle while the real motor is still catching up.
                new_v = 0.
                self.manual_ramp.reset(axis)
                self.tracking_ramp.reset(axis)
            if force_hold or (self.outside_hold_origin is not None and new_v == 0.):
                self.manual_ramp.reset(axis)
                self.tracking_ramp.reset(axis)
            target.append(q)
            velocity.append(new_v)
        self.stream_target, self.stream_velocity = tuple(target), tuple(velocity)
        self.last_send = now
        if force_hold or now-self.last_position_send >= self.c["position_period_s"]:
            sent = list(self.sent_target)
            for axis,(mid,q,sign) in enumerate(zip(self.c["motor_ids"], target, self.c["motor_signs"])):
                packet = set_single_turn_position(mid, q/sign)
                units = int.from_bytes(packet[3:5], "big")
                if units != self.position_units[axis]:
                    self._write(packet)
                    self.last_position_send = now
                    self.position_units[axis] = units
                    sent[axis] = decoded_position(self.c,axis,units,self.angles[axis])
            self.sent_target = tuple(sent)
            self.hold_pending = False
        if self.response_context != "TRACK":
            self.guard.update(now, self.angles, self.sent_target)

    def _home_and_clear(self):
        try:
            if not self.recovery_active:
                self._begin_home()
                self.stream_target = self.angles
                self.stream_velocity = (0., 0.)
            super()._home_and_clear()
            self.startup_completed = True
            self._latch_hold(self.sent_target)
        finally:
            self.arm_homing = False
            self.recovery_active = False
            self.recovery_origin = None
            self.outside_hold_origin = None

    def _disable_now(self):
        self.home_ramp.reset()
        self.hold_jump_candidate = [None,None]
        self.hold_target = None
        self._cancel_angle("DISARM requested")
        self.braking = self.soft_hold = False
        super()._disable_now()
        if self.native_pitch_engaged:self._restore_native_pitch()
        if any(self.native_tracker.engaged):self.native_tracker.restore()
        self.response.reset()
        self.manual_ramp.reset()
        self.tracking_ramp.reset()
        self.motion_paused = False
        self.position_intent = [None, None]
        self.programmed_rpm = [None, None]
        self.arm_homing = False
        self.recovery_active = False
        self.recovery_origin = None
        self.outside_hold_origin = None

    def _latch_fault(self, reason, reference_lost=False):
        self.home_ramp.reset()
        self.hold_jump_candidate = [None,None]
        self.hold_target = None
        self._cancel_angle(reason)
        self.braking = self.soft_hold = False
        if self.last_feedback_fault is None:
            now = time.monotonic()
            self.last_feedback_fault = dict(reason=reason,time=now,
                actual_deg=self.angles,commanded_deg=self.sent_target,
                planned_velocity_deg_s=self.stream_velocity,control_context=self.response_context,
                accepted_age_s=tuple(None if q is None else now-q[1] for q in self.feedback_cache),
                failed_query_duration_s=tuple(None if t is None else now-t for t in self.feedback_missing_since),
                issues=tuple(self.feedback_issues),phase=self.phase)
        self.response.reset()
        self.manual_ramp.reset()
        self.arm_homing = self.home_requested = self.home_holding = False
        self.recovery_active = False
        self.recovery_origin = None
        self.outside_hold_origin = None
        super()._latch_fault(reason, reference_lost)
        if self.native_pitch_engaged:
            try:self._restore_native_pitch()
            except Exception as exc:print('NATIVE_MODE_RESTORE_FAILED',str(exc),flush=True)
        if any(self.native_tracker.engaged):
            try:self.native_tracker.restore()
            except Exception as exc:print('TRACKING_MODE_RESTORE_FAILED',str(exc),flush=True)

    def _recheck_stationary_required(self):
        # The saved single-turn zero is not recalibrated here. An eccentric
        # disabled pitch may fall; fresh valid dual-axis feedback is sufficient.
        return not self.c["mechanical_zero_confirmed"]

    def _perform_recheck(self):
        self.hold_jump_candidate = [None,None]
        self.recheck_started = time.monotonic()
        self.recheck_finished = None
        self.recheck_previous_fault = dict(reason=self.fault,feedback_fault=self.last_feedback_fault,
                                           disable_reason=self.last_disable_reason,angles=self.angles)
        self.feedback_cache = [None, None]
        self.feedback_missing_since = [None, None]
        self.strict_feedback = True
        try:
            super()._perform_recheck()
        finally:
            self.strict_feedback = False
            self.recheck_finished = time.monotonic()
        self.zero_confirmed = self.c["mechanical_zero_confirmed"]
        self.guard.last_event = None
        self.last_feedback_fault = None
        self.motion_anomalies = {}
        self.motion_warning = None

    def snapshot(self):
        result = super().snapshot()
        if self.c.get('continuous_yaw',False):result['limits']=((None,None),tuple(self.c['limits'][1]))
        result.update(continuous_yaw=self.c.get('continuous_yaw',False),
                      yaw_heading_deg=signed_mechanical(self.angles[0]),yaw_angle_unwrapped_deg=self.angles[0])
        result["app_version"] = self.c["app_version"]
        result["angle_move"] = self.angle_move.snapshot(self.angle_pending)
        result['inertia_test_active']=self.inertia_test_active
        result['native_pitch_engaged']=self.native_pitch_engaged
        result['tracking_motor_control']=self.c['tracking_motor_control']
        result['speed_mode_signs']=self.c['speed_mode_signs']
        result['tracking_command_rpm']=tuple(getattr(self.native_tracker,'command_rpm',(0,0)))
        result.update(native_tracking_enabled=self.c['native_tracking'],native_tracking_engaged=tuple(self.native_tracker.engaged),
                      native_tracking_rate=native_tracking_rates(self.c),native_tracking_accel=self.c['native_tracking_accel'],
                      native_tracking_rate_limit=self.c['native_tracking_rate'])
        result.update(native_tracking_setup_count=self.native_tracker.setup_count,
                      native_tracking_gap_hold_count=self.native_tracker.gap_count,
                      native_aim_rejected_frames=self.native_tracker.aim_rejections,
                      native_measured_velocity_deg_s=tuple(self.native_tracker.velocity),
                      native_damping_deg=tuple(self.native_tracker.damping),
                      native_center_deadzone=self.c['native_center_deadzone'])
        result['planned_velocity_deg_s']=self.stream_velocity
        result.update(hold_return_rate=self.c['hold_return_rate'],home_jerk_deg_s3=self.c['home_jerk_deg_s3'])
        result.update(hold_target_deg=self.hold_target,hold_fixed_active=self._fixed_hold_active(),
                      hold_error_deg=None if self.hold_target is None else tuple(q-a for q,a in zip(self.hold_target,self.angles)),
                      hold_disturbance_limits=result['limits'],hold_block_timeout_s=self.c['hold_block_timeout_s'])
        result["recheck_diagnostic"] = dict(stationary_required=self._recheck_stationary_required(),
            started=self.recheck_started,finished=self.recheck_finished,previous=self.recheck_previous_fault)
        result.update(braking_active=self.braking,soft_hold=self.soft_hold,
                      manual_accel_deg_s2=self.c["manual_accel_deg_s2"],brake_timeout_s=self.c["brake_timeout_s"])
        result.update(last_disable_reason=self.last_disable_reason,disable_history=list(self.disable_history),
                      motor_signs=self.c["motor_signs"],
                      direction_mapping=dict(right_input_mechanical_sign=self.c["joystick_signs"][0],
                                             up_input_mechanical_sign=self.c["joystick_signs"][1]))
        result["protection"] = dict(home_feedback_margin_deg=self.c["recovery_margin_deg"],
            tracking_reverse_deg=self.c["tracking_reverse_deg"],tracking_reverse_persist_s=self.c["tracking_reverse_persist_s"],
            tracking_stall_basis="physical movement range with sustained position error",
            home_hard_margin_deg=self.c["recovery_hard_margin_deg"],
            motion_violation_grace_s=self.c["motion_violation_grace_s"],
            feedback_loss_grace_s=self.c["feedback_loss_grace_s"],
            stall_observation_s=self.c["motion_grace_s"]+self.c["motion_window_s"],
            control_pause_age_s=self.c["control_pause_age_s"],watchdog_s=self.c["watchdog_s"])
        result.update(arm_homing=self.arm_homing,
                      auto_home_on_arm=self.c["auto_home_on_arm"],
                      tracking_rate=self.c["tracking_rate"],
                      suspicious_motion=self.guard.last_event)
        result.update(max_rate=self.c["manual_rate"], motor_rate_ceiling=self.c["max_rate"])
        result.update(manual_jerk_deg_s3=self.c["manual_jerk_deg_s3"],
                      manual_acceleration_deg_s2=tuple(self.manual_ramp.acceleration),
                      command_accel_ceiling_deg_s2=self.c["command_accel_deg_s2"], motor_rpm=self.c["motor_rpm"])
        result["applied_motor_rpm"] = tuple(self.programmed_rpm)
        now = time.monotonic()
        result.update(feedback_degraded=bool(any(self.feedback_issues)),
                      feedback_loss_grace_s=self.c["feedback_loss_grace_s"],
                      feedback_axis_age_s=tuple(None if v is None else now-v[1] for v in self.feedback_cache),
                      feedback_issues=tuple(self.feedback_issues), feedback_rejected_samples=tuple(self.feedback_rejects),
                      feedback_failed_query_duration_s=tuple(None if t is None else now-t for t in self.feedback_missing_since),
                      last_feedback_fault=self.last_feedback_fault,
                      motion_warning=self.motion_warning or self.guard.warning, motion_paused=self.motion_paused,
                      response=self._response_snapshot(), default_saved_zero=self.c["mechanical_zero_confirmed"])
        result["response_context"] = self.response_context
        result.update(position_lead_deg=tuple(self._position_lead_deg(a) for a in (0,1)),
                      position_period_s=self.c["position_period_s"],motor_loop_period_s=self.c["motor_loop_period_s"],
                      position_write_interval_s=tuple(self.position_write_interval),
                      position_write_step_deg=tuple(self.position_write_step),
                      position_write_count=tuple(self.position_write_count),
                      home_accel_deg_s2=self.c["home_accel_deg_s2"],
                      tracking_accel_deg_s2=self.c["tracking_accel_deg_s2"])
        result.update(recovery_active=self.recovery_active, startup_auto_home=self.c["startup_auto_home"],
                      outside_hold=self.outside_hold_origin is not None,
                      outside_manual_inward_only=self.outside_hold_origin is not None,
                      startup_completed=self.startup_completed,
                      startup_initialized=self.startup_initialized, startup_cancelled=self.startup_cancelled,
                      outside_work_limits=any(not lo-self.c["limit_feedback_margin_deg"] <= q <= hi+self.c["limit_feedback_margin_deg"]
                                              for q,(lo,hi) in zip(self.angles, self.c["limits"])))
        return result


# Source: yolo_click_tracker\src\prediction.py
"""Short-horizon image-motion estimate compensated with measured gimbal pose."""


class MotionPredictor:
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self):
        self.previous = None
        self.velocity = (0., 0.)
        self.samples = 0

    @property
    def ready(self):
        return self.samples >= self.c["prediction_min_samples"]

    @property
    def motion_velocity(self):
        return tuple(v if abs(v) >= self.c["prediction_min_velocity"] else 0. for v in self.velocity)

    def observe(self, error, pose, stamp, score=1.):
        if (not self.c["prediction_enabled"] or pose is None or len(pose) != 2
                or not math.isfinite(score) or score < .55 or not all(math.isfinite(v) for v in (*error, *pose, stamp))):
            self.reset()
            return
        current = (tuple(error), tuple(pose), stamp)
        previous = self.previous
        self.previous = current
        if previous is None:
            self.samples = 1
            return
        before, before_pose, before_time = previous
        dt = stamp-before_time
        if not .02 <= dt <= self.c["prediction_max_gap_s"]:
            self.velocity, self.samples = (0., 0.), 1
            return
        # error directions: right/up. input signs map them to mechanical angles.
        displacement = tuple(e-b+s*(p-bp)/f for e,b,s,p,bp,f in zip(
            error, before, self.c["joystick_signs"], pose, before_pose, self.c["camera_fov"]))
        if max(abs(d) for d in displacement) > self.c["prediction_jump_gate"]:
            self.velocity, self.samples = (0., 0.), 1
            return
        alpha = 1-(1-self.c["prediction_velocity_alpha"])**(dt/.1)
        velocity = []
        for delta, old in zip(displacement, self.velocity):
            measured = clip(delta/dt, -self.c["prediction_max_velocity"], self.c["prediction_max_velocity"])
            if measured*old < 0:
                old = 0.  # Do not extrapolate the old direction after a reversal.
            if abs(measured) < self.c["prediction_min_velocity"]/2:
                old *= .3
            v = old+alpha*(measured-old)
            velocity.append(v)
        self.velocity = tuple(velocity)
        self.samples += 1

    def lead(self, error, latency=0.):
        if not self.ready:
            return tuple(error)
        horizon = self.c["prediction_lead_s"]+clip(latency, 0., self.c["prediction_latency_cap_s"])
        limit = self.c["prediction_max_shift"]
        return tuple(clip(e+clip(v*horizon, -limit, limit), -.5, .5)
                     for e,v in zip(error, self.motion_velocity))


# Source: yolo_click_tracker\src\overhead.py
"""Capture-time line-of-sight geometry and a bounded, non-inverting yaw relay."""
def sight_vector(error, pose, c):
    yaw=math.radians(pose[0]);elev=math.radians(pose[1]*c['joystick_signs'][1])
    cy,sy,ce,se=math.cos(yaw),math.sin(yaw),math.cos(elev),math.sin(elev)
    u=2*error[0]*math.tan(math.radians(c['camera_fov'][0]/2))
    v=2*error[1]*math.tan(math.radians(c['camera_fov'][1]/2))
    right=(-sy*c['joystick_signs'][0],cy*c['joystick_signs'][0],0.)
    forward=(ce*cy,ce*sy,se);up=(-se*cy,-se*sy,ce)
    vector=tuple(f+u*r+v*t for f,r,t in zip(forward,right,up))
    length=math.sqrt(sum(q*q for q in vector))
    return tuple(q/length for q in vector)


class OverheadPlanner:
    def __init__(self,c):
        self.c=c;self.reset()

    def reset(self):
        self.state='NORMAL';self.entry_yaw=self.turn_yaw=self.deadline=None
        self.candidate=None;self.previous=None
        self.velocity=(0.,0.);self.turn_count=0;self.elevation=None
        self.last_turn_sign=1.
        self.last_vector=self.vector_previous=None
        self.vector_velocity=(0.,0.,0.)
        self.reacquire_until=None;self.reacquire_candidate=None

    def waiting(self,now):
        return self.reacquire_until is not None and now<=self.reacquire_until

    def reacquire(self,tracks,selected,pose,stamp):
        if not self.waiting(stamp) or pose is None or self.last_vector is None:return None
        age=max(0.,stamp-self.vector_previous[1])
        predicted=tuple(q+v*min(age,.3) for q,v in zip(self.last_vector,self.vector_velocity))
        length=math.sqrt(sum(q*q for q in predicted))
        predicted=tuple(q/length for q in predicted) if length>1e-9 else self.last_vector
        candidates=[]
        for track in tracks:
            if track['key'][0]!=selected[0] or track.get('score',0.)<self.c['new_track_confidence']:continue
            vector=sight_vector((track['cx']-.5,.5-track['cy']),pose,self.c)
            angle=math.degrees(math.acos(clip(sum(a*b for a,b in zip(predicted,vector)),-1.,1.)))
            if angle<=10.:candidates.append(track)
        if len(candidates)!=1:
            self.reacquire_candidate=None;return None
        candidate=candidates[0];key=tuple(candidate['key']);previous=self.reacquire_candidate
        self.reacquire_candidate=(key,stamp)
        if previous is not None and previous[0]==key and 0<stamp-previous[1]<=.2:
            self.reacquire_candidate=None;return candidate
        return None

    @property
    def active(self):return self.state!='NORMAL'

    def _turn_plan(self,pose):
        return dict(goal=(self.turn_yaw,88./self.c['joystick_signs'][1]),
                    velocity=(0.,0.),active=(True,True),state=self.state)

    def missing(self,now,pose):
        if self.state!='TURN' or pose is None:return None
        if now>self.deadline:
            self.state='POLE';return None
        if abs(self.turn_yaw-pose[0])<=8.:
            self.state='POLE';self.entry_yaw=self.turn_yaw;self.candidate=None
            return None
        return self._turn_plan(pose)

    def observe(self,error,capture_pose,current_pose,stamp,score=1.):
        if (not self.c['overhead_tracking'] or not self.c['continuous_yaw']
                or capture_pose is None or current_pose is None):return None
        if self.state=='NORMAL':
            upper=max(capture_pose[1]*self.c['joystick_signs'][1],current_pose[1]*self.c['joystick_signs'][1])
            if upper+abs(error[1])*self.c['camera_fov'][1]*1.2<70.:return None
        vector=sight_vector(error,capture_pose,self.c)
        if self.vector_previous is not None and .01<stamp-self.vector_previous[1]<=.25:
            dt=stamp-self.vector_previous[1];alpha=dt/(.1+dt)
            self.vector_velocity=tuple(v+alpha*((q-old)/dt-v) for v,q,old in
                zip(self.vector_velocity,vector,self.vector_previous[0]))
        else:self.vector_velocity=(0.,0.,0.)
        self.last_vector=vector;self.vector_previous=(vector,stamp)
        az=math.degrees(math.atan2(vector[1],vector[0]))
        elev=math.degrees(math.atan2(vector[2],math.hypot(*vector[:2])))
        self.elevation=elev
        yaw=current_pose[0]+signed_mechanical(az-current_pose[0])
        pitch=clip(elev/self.c['joystick_signs'][1],-90.,self.c['limits'][1][1])
        camera_elev=capture_pose[1]*self.c['joystick_signs'][1]
        if self.state=='NORMAL':
            if max(elev,camera_elev)<70.:return None
            self.state='GEOMETRY'
        if self.state=='TURN':
            plan=self.missing(stamp,current_pose)
            if plan is not None:return plan
        crossed_between_frames=(camera_elev>=75. and elev>=70. and abs(signed_mechanical(az-current_pose[0]))>100.)
        if self.state=='GEOMETRY' and (elev>=84. or crossed_between_frames):
            self.state='POLE';self.entry_yaw=current_pose[0];self.candidate=None
        if self.state=='POLE':
            entry=math.radians(self.entry_yaw)
            behind=vector[0]*math.cos(entry)+vector[1]*math.sin(entry)
            delta=signed_mechanical(az-self.entry_yaw)
            crossing=behind < -math.sin(math.radians(2.)) and abs(delta)>100. and score>=.5
            if crossing:
                before=self.candidate
                self.candidate=stamp
                if before is not None and 0<stamp-before<=.2:
                    if abs(delta)>=170.:delta=180.*self.last_turn_sign
                    self.last_turn_sign=1. if delta>0 else -1.
                    self.turn_yaw=self.entry_yaw+delta
                    self.deadline=stamp+self.c['overhead_turn_timeout_s']
                    self.reacquire_until=self.deadline+.5
                    self.turn_count+=1;self.state='TURN';self.previous=None
                    return self._turn_plan(current_pose)
            else:self.candidate=None
            if elev<78.:
                self.state='GEOMETRY';self.previous=None
            else:
                return dict(goal=(self.entry_yaw,88./self.c['joystick_signs'][1]),
                            velocity=(0.,0.),active=(True,True),state=self.state)
        if self.state=='GEOMETRY' and elev<60. and camera_elev<65.:
            self.state='NORMAL';self.previous=None;self.velocity=(0.,0.);return None
        previous=self.previous
        self.previous=(yaw,pitch,stamp)
        if previous is not None and .01<stamp-previous[2]<=.25:
            dt=stamp-previous[2];alpha=dt/(.1+dt)
            measured=(signed_mechanical(yaw-previous[0])/dt,(pitch-previous[1])/dt)
            cap=self.c['motor_rpm']*6
            self.velocity=tuple(v+alpha*(clip(q,-cap,cap)-v) for v,q in zip(self.velocity,measured))
        else:self.velocity=(0.,0.)
        return dict(goal=(yaw,pitch),velocity=self.velocity,active=(True,True),state=self.state)

    def snapshot(self):
        return dict(state=self.state,turn_yaw_deg=self.turn_yaw,turn_count=self.turn_count,
                    deadline=self.deadline,elevation_deg=self.elevation,reacquire_until=self.reacquire_until)


# Source: yolo_click_tracker\src\tracking.py
"""1101 mainweb semantics: same ID during 2s loss; timeout clears and homes."""


class ClickTracker:
    def __init__(self, c):
        self.c = c
        self.selected = self.last_seen = self.last_tick = None
        self.filtered = [0., 0.]
        self.vector = (0., 0.)
        self.state = "DETECT"
        self.c_next_home = False
        self.predictor = MotionPredictor(c)
        self.prediction_point = None
        self.prediction_active = False
        self.settled = [False,False]
        self.tuning_offset = (0., 0.)
        self.control_error = (0., 0.)
        self.aim_goal = None
        self.aim_velocity=(0.,0.)
        self.native_active_axes=(False,False)
        self.last_tracked_goal=None
        self.last_tracked_velocity=(0.,0.)
        self.overhead=OverheadPlanner(c)
        self.overhead_reacquired=0

    def clear(self, arm_c=False):
        self.selected = self.last_seen = self.last_tick = None
        self.filtered = [0., 0.]
        self.vector = (0., 0.)
        self.state = "DETECT"
        self.c_next_home = arm_c
        self.predictor.reset()
        self.prediction_point = None
        self.prediction_active = False
        self.settled = [False,False]
        self.tuning_offset = (0., 0.)
        self.aim_goal = None
        self.aim_velocity=(0.,0.);self.native_active_axes=(False,False)
        self.last_tracked_goal=None;self.last_tracked_velocity=(0.,0.)
        self.overhead.reset()
        self.overhead_reacquired=0

    def select(self, key, tracks, now):
        key = tuple(key)
        if not any(tuple(t["key"]) == key for t in tracks):
            raise RuntimeError("Target is no longer visible")
        self.clear()
        self.selected, self.last_seen, self.state = key, now, "TRACK"

    def drive(self, now, pose=None):
        if self.state=='OVERHEAD':
            plan=self.overhead.missing(now,pose)
            if plan is None:
                self.state='LOST';self.aim_goal=None;self.native_active_axes=(False,False)
            else:self.aim_goal=plan['goal'];self.native_active_axes=plan['active']
        if self.state == "COAST":
            age = max(0., now-self.last_seen)
            if age >= self.c["prediction_coast_s"]:
                self.state, self.vector = "LOST", (0., 0.)
                self.prediction_point, self.prediction_active = None, False
                self.aim_goal=None;self.native_active_axes=(False,False)
            else:
                fade = 1-age/self.c["prediction_coast_s"]
                self.vector = tuple(clip(v*f, -cap, cap)*fade/rate for v,f,cap,rate in zip(
                    self.predictor.motion_velocity, self.c["camera_fov"], self.c["prediction_coast_rate"], self.c["max_rate"]))
                if self.c['native_tracking'] and self.last_tracked_goal is not None:
                    distance_time=age-age*age/(2*self.c['prediction_coast_s'])
                    self.aim_goal=tuple(q+clip(v*distance_time,-cap,cap) for q,v,cap in
                        zip(self.last_tracked_goal,self.last_tracked_velocity,self.c['native_coast_max_deg']))
                    self.aim_velocity=tuple(v*fade for v in self.last_tracked_velocity)
                    self.native_active_axes=tuple(abs(v)>0 for v in self.last_tracked_velocity)
        return self.vector

    def step(self, tracks, now, pose=None, latency=0., current_pose=None):
        dt = min(.2, max(.02, now-self.last_tick)) if self.last_tick is not None else .1
        self.last_tick = now
        if self.selected is None:
            self.vector = (0., 0.)
            return None
        target = next((t for t in tracks if tuple(t["key"]) == self.selected), None)
        if target is None:
            target=self.overhead.reacquire(tracks,self.selected,pose,now)
            if target is not None:
                self.selected=tuple(target['key']);self.overhead_reacquired+=1;self.predictor.reset()
        if target is None:
            self.aim_goal = None
            self.vector = (0., 0.)
            self.filtered = [0., 0.]
            self.state = "LOST"
            plan=self.overhead.missing(now,current_pose)
            if plan is not None:
                self.state='OVERHEAD';self.aim_goal=plan['goal'];self.aim_velocity=(0.,0.)
                self.native_active_axes=plan['active'];return None
            if now-self.last_seen > self.c["lost_timeout"] and not self.overhead.waiting(now):
                self.clear()
                self.state = "RETURNING"
                return "home"
            if self.predictor.ready and any(self.predictor.motion_velocity) and now-self.last_seen < self.c["prediction_coast_s"]:
                self.state = "COAST"
                self.prediction_active = True
                self.drive(now)
            else:
                self.prediction_point, self.prediction_active = None, False
            return None
        self.last_seen, self.state = now, "TRACK"
        error = (target["cx"]-.5, .5-target["cy"])
        self.predictor.observe(error, pose, now, target.get("score", 1.))
        control_error = error
        if pose is not None and current_pose is not None:
            control_error = tuple(e-clip(sign*(q-old)/f,-.2,.2) for e,sign,q,old,f in zip(
                error,self.c["joystick_signs"],current_pose,pose,self.c["camera_fov"]))
        control_error = tuple(e+offset for e, offset in zip(control_error, self.tuning_offset))
        self.control_error = control_error
        predicted = self.predictor.lead(control_error, latency)
        self.prediction_active = self.predictor.ready and any(self.predictor.motion_velocity)
        # Draw on the capture-time image, not on the newer compensated pose.
        display_prediction=self.predictor.lead(tuple(e+offset for e,offset in zip(error,self.tuning_offset)),latency)
        self.prediction_point = (.5+display_prediction[0], .5-display_prediction[1]) if self.prediction_active else None
        vector = []
        for axis, e in enumerate(predicted):
            zone=self.c['native_center_deadzone'] if self.c['native_tracking'] else self.c['deadzone']
            ratio=self.c['native_center_exit_ratio'] if self.c['native_tracking'] else self.c['tracking_settle_ratio']
            threshold = zone[axis]*(ratio if self.settled[axis] else 1.)
            self.settled[axis] = abs(e) < threshold
            if self.settled[axis]:
                self.filtered[axis] = 0.
            else:
                angular = e*self.c["camera_fov"][axis]
                blend = clip(abs(angular)/self.c["tracking_blend_deg"][axis], 0., 1.)
                gain = self.c["tracking_near_kp"][axis]+blend*(self.c["tracking_far_kp"][axis]-self.c["tracking_near_kp"][axis])
                value = angular*gain*.1*self.c["visual_gain"][axis]/.7
                if value*self.filtered[axis] < 0:
                    self.filtered[axis] = 0.  # Never retain the old push after crossing center.
                # Reference EMA and step caps are specified at 10Hz.
                alpha = 1-(1-self.c["ema"][axis])**(dt/.1)
                self.filtered[axis] += alpha*(value-self.filtered[axis])
            feedforward = self.predictor.motion_velocity[axis]*self.c["camera_fov"][axis]*self.c["prediction_feedforward"] if self.predictor.ready else 0.
            cap = min(self.c["step_cap"][axis]/.1, self.c["tracking_rate"][axis])
            cap = min(cap,max(0.,abs(e)-self.c["deadzone"][axis])*self.c["camera_fov"][axis]/self.c["tracking_stop_horizon_s"])
            rate = clip((self.filtered[axis]/.1+feedforward)*self.c["tracking_speed_multiplier"], -cap, cap)
            if self.settled[axis]:
                rate = 0.
            vector.append(rate/self.c["max_rate"][axis])
        n = max(1., math.hypot(*vector))
        self.vector = tuple(v/n for v in vector)
        self.aim_goal=(tuple(q+sign*e*f*gain*(visual/.7) for q,sign,e,f,gain,visual in
                             zip(current_pose,self.c['joystick_signs'],predicted,self.c['camera_fov'],
                                 self.c['native_aim_gain'],self.c['visual_gain'])) if current_pose is not None else None)
        self.aim_velocity=tuple(sign*v*f for sign,v,f in zip(self.c['joystick_signs'],self.predictor.motion_velocity,self.c['camera_fov']))
        self.native_active_axes=tuple(not settled or (self.predictor.ready and abs(v)>0) for settled,v in zip(self.settled,self.predictor.motion_velocity))
        plan=self.overhead.observe(error,pose,current_pose,now,target.get('score',1.)) if self.c['native_tracking'] else None
        if plan is not None:
            self.aim_goal=plan['goal'];self.aim_velocity=plan['velocity'];self.native_active_axes=plan['active']
            self.prediction_point=None  # Flat-image prediction does not describe zenith geometry.
        self.last_tracked_goal=self.aim_goal;self.last_tracked_velocity=self.aim_velocity
        return None


def contain_rect(iw, ih, viewport=(0, 40, 480, 360)):
    x, y, w, h = viewport
    scale = min(w/iw, h/ih)
    sw, sh = int(iw*scale), int(ih*scale)
    return x+(w-sw)//2, y+(h-sh)//2, sw, sh


def screen_to_normalized(x, y, rect):
    rx, ry, rw, rh = rect
    if not rx <= x < rx+rw or not ry <= y < ry+rh:
        return None
    return (x-rx)/rw, (y-ry)/rh


# Source: yolo_click_tracker\src\software_autotune.py
"""Onboard visual-loop tuning around a stationary selected target."""
import json
import os


class SoftwareAutoTuner:
    KEYS = ("tracking_near_kp", "tracking_far_kp", "ema", "tracking_accel_deg_s2")

    def __init__(self, hub):
        self.hub = hub
        self.axis = None
        self.session = AutoTuneExperiment(self._apply, self._offset)
        self.origin = None
        self.selected = None
        self.last_pose = None
        self.saved_path = None

    @property
    def active(self):
        return self.session.active

    def _apply(self, profile):
        if set(profile) != set(self.KEYS):
            raise ValueError("Invalid software profile keys")
        proposed = dict(self.hub.c, **profile)
        validate_visual_config(proposed)
        for config in (self.hub.c, self.hub.driver.c):
            for key in self.KEYS:
                config[key] = tuple(profile[key])
        self.hub.tracking.filtered = [0., 0.]
        self.hub.tracking.settled = [False, False]

    def _offset(self, degrees):
        offset = [0., 0.]
        offset[self.axis] = degrees/self.hub.c["camera_fov"][self.axis]
        self.hub.tracking.tuning_offset = tuple(offset)
        self.hub.tracking.settled[self.axis] = False

    def start(self, axis):
        if type(axis) is not int or axis not in (0, 1):
            raise ValueError("Choose one tuning axis")
        motor = self.hub.driver.snapshot()
        if (not self.hub.fresh() or self.hub.tracking.selected is None or not motor["armed"]
                or motor["fault"] or motor.get("home_requested") or motor.get("outside_hold")
                or any(self.hub.manual) or not motor["reference_valid"] or motor.get("feedback_degraded")
                or motor["age"] is None or motor["age"] > self.hub.c["pose_max_age_s"]):
            raise RuntimeError("Select a stationary target with fresh feedback and released controls")
        if self.active:
            raise RuntimeError("Software auto tune already running")
        selected = next((track for track in self.hub.latest["tracks"]
                         if tuple(track["key"]) == self.hub.tracking.selected), None)
        if selected is None or any(abs(error*fov) > 4 for error, fov in zip(
                (selected["cx"]-.5, .5-selected["cy"]), self.hub.c["camera_fov"])):
            raise RuntimeError("Wait until the stationary target is within 4 degrees of image center")
        self.axis = axis
        self.saved_path = None
        self.origin = tuple(motor["angles"])
        for angle, limits in zip(self.origin, self.hub.c["limits"]):
            if not limits[0]+6 <= angle <= limits[1]-6:
                raise RuntimeError("Move at least 6 degrees inside working limits before auto tune")
        self.selected = self.hub.tracking.selected
        self.last_pose = None
        self.session.amplitude = 4.
        self.session.tolerance = min(3., self.hub.c["deadzone"][axis]*self.hub.c["camera_fov"][axis]*self.hub.c["tracking_settle_ratio"]+.2)
        baseline = {key: tuple(self.hub.c[key]) for key in self.KEYS}
        candidates = []
        for factor in (.85, 1.15):
            candidate = copy.deepcopy(baseline)
            for key in ("tracking_near_kp", "tracking_far_kp"):
                values = list(candidate[key])
                values[axis] *= factor
                candidate[key] = tuple(values)
            candidates.append(candidate)
        for change in (-.1, .1):
            candidate = copy.deepcopy(baseline)
            values = list(candidate["ema"])
            values[axis] = max(.2, min(.8, values[axis]+change))
            candidate["ema"] = tuple(values)
            candidates.append(candidate)
        candidate = copy.deepcopy(baseline)
        values = list(candidate["tracking_accel_deg_s2"])
        values[axis] *= .85
        candidate["tracking_accel_deg_s2"] = tuple(values)
        candidates.append(candidate)
        self.session.start(baseline, candidates, self.hub.clock())

    def observe(self, frame, motor):
        if not self.active:
            return
        now = frame["stamp"]
        if now <= self.session.last_stamp:
            return  # Discard the in-flight double-buffer frame from before START.
        valid = (motor["armed"] and not motor["fault"] and motor["reference_valid"]
                 and not motor.get("feedback_degraded") and not motor.get("home_requested")
                 and motor["age"] is not None and motor["age"] <= self.hub.c["pose_max_age_s"]
                 and self.hub.tracking.selected == self.selected
                 and self.hub.tracking.state == "TRACK"
                 and all(abs(a-b) <= 8 for a, b in zip(motor["angles"], self.origin)))
        if not valid:
            self.abort("Target or mechanical feedback lost during software auto tune")
            return
        error = self.hub.tracking.control_error[self.axis]*self.hub.c["camera_fov"][self.axis]
        pose = motor["angles"][self.axis]
        speed = 0.
        if self.last_pose is not None:
            speed = (pose-self.last_pose[1])/(now-self.last_pose[0])
        self.last_pose = (now, pose)
        self.session.observe(now, error, speed)
        if not self.active:
            self._offset(0.)
            if self.session.state == "FAILED":
                self.abort(self.session.reason)

    def abort(self, reason="software auto tune cancelled"):
        if not hasattr(self.session, "baseline"):
            return
        self.session.active = False
        self.session.state = "CANCELLED"
        self.session.reason = reason
        self._apply(self.session.baseline)
        self._offset(0.)
        self.session.restore_pending = False
        self.hub.tracking.clear()
        self.hub.manual = (0., 0.)

    def export(self):
        result = self.session.snapshot()
        result.update(axis=self.axis, selected=self.selected, origin=self.origin,
                      saved_path=self.saved_path,
                      sensor_basis="visual error and encoder motion; IMU not enabled")
        return result

    def save(self):
        if self.active or self.session.state != "COMPLETE":
            raise RuntimeError("Finish verification before saving the software profile")
        path = self.hub.c["software_autotune_profile"]
        directory = os.path.dirname(path)
        os.makedirs(directory, exist_ok=True)
        profile = dict(schema=1, model_path=self.hub.c["model_path"],
                       camera_fov=self.hub.c["camera_fov"], motor_signs=self.hub.c["motor_signs"],
                       joystick_signs=self.hub.c["joystick_signs"], limits=self.hub.c["limits"],
                       parameters=self.session.best, experiment=self.export())
        with open(path+".tmp", "w", encoding="utf-8") as output:
            json.dump(profile, output, ensure_ascii=False, allow_nan=False)
        os.replace(path+".tmp", path)
        self.saved_path = path
        return path

    def load(self):
        path = self.hub.c["software_autotune_profile"]
        try:
            with open(path, encoding="utf-8") as source:
                saved = json.load(source)
        except FileNotFoundError:
            return False
        if saved.get("schema") != 1:
            raise ValueError("Unsupported saved software profile")
        for key in ("model_path", "camera_fov", "motor_signs", "joystick_signs", "limits"):
            if json.dumps(saved.get(key)) != json.dumps(self.hub.c[key]):
                raise ValueError("Saved profile configuration differs: " + key)
        self._apply(saved["parameters"])
        return True


# Source: yolo_click_tracker\src\hub.py
"""Arbitrate local/web control; network heartbeats never feed the UART worker."""
from collections import deque


class MotorMailbox:
    def __init__(self, hub, driver):
        self.hub, self.driver, self.c = hub, driver, driver.c

    def snapshot(self):
        return self.driver.snapshot()

    @property
    def arm_requested(self):
        return self.driver.arm_requested

    def input(self, x, y):
        state = self.driver.snapshot()
        if (x or y) and (state.get("home_requested") or (self.driver.arm_requested and not state["armed"])):
            self.hub.manual = (0., 0.)
            return  # Do not queue a manual move behind HOME.
        self.hub.manual = (x, y)
        if x or y:
            self.hub.tracking.clear()

    def request_arm(self, armed, reason="手动 DISARM／停止请求"):
        if armed and not self.hub.fresh():
            raise RuntimeError("Fresh camera/inference required before ARM")
        if not armed:
            if self.hub.software_autotune.active:
                self.hub.software_autotune.abort("DISARM requested")
            self.hub.tracking.clear()
            self.hub.manual = (0., 0.)
        if hasattr(self.driver,"disable_history"):
            self.driver.request_arm(armed,reason=reason)
        else:
            self.driver.request_arm(armed)
        if armed:
            self.hub.tracking.clear()
            self.hub.manual = (0., 0.)
            self.hub.notice = "ARM：使能并保持当前位置；HOME 单独回机械原点"

    def confirm_reference(self):
        self.driver.confirm_reference()

    def request_home(self):
        if not self.hub.fresh():
            raise RuntimeError("Fresh camera/inference required before HOME")
        self.driver.request_home()
        self.hub.tracking.clear()
        self.hub.manual = (0., 0.)

    def request_recheck(self):
        self.driver.request_recheck()
        self.hub.tracking.clear()
        self.hub.manual = (0., 0.)


class TrackingHub(BrowserControl):
    def __init__(self, driver, c, clock=time.monotonic):
        self.driver, self.c = driver, c
        self.tracking = ClickTracker(c)
        self.manual = (0., 0.)
        self.local_owner = True
        self.frames = deque(maxlen=c["frame_history"])
        self.latest = self.video = None
        self.preview_source = None
        self.jpeg_lock = threading.Lock()
        self.last_jpeg_attempt = float("-inf")
        self.performance = {}
        self.timings = StageTimings()
        self.detached_token = None
        self.last_control_rejection = None
        self.frame_id = 0
        super().__init__(MotorMailbox(self, driver), clock, WEB_CONFIG["lease_s"])
        self.software_autotune = SoftwareAutoTuner(self)
        if c["mechanical_zero_confirmed"]:
            self.notice = "使用驱动已保存机械零点；开机自动回零保持" if c["startup_auto_home"] else "使用已保存机械零点；ARM 保持，HOME 单独回零"

    def recheck(self, token):
        value = super().recheck(token)
        if self.c["mechanical_zero_confirmed"]:
            self.notice = "重新检查中；通过后继续使用已保存零点，手动 ARM 或 HOME"
            value["notice"] = self.notice
        return value

    def _disable_request(self, reason):
        self.motor.request_arm(False, reason=reason)

    def _reject_command(self, reason, status=409):
        # A repeated/busy action or mixed button/stick packet is a recoverable
        # operator conflict, not a reason to drop a gravity-loaded camera.
        self.motor.input(0., 0.)
        self.driver.input(0., 0.)
        self.hold_start, self.blocked = None, False
        self.deadline = self.clock()+self.lease_s
        self.last_control_rejection = dict(reason=reason,time=self.clock())
        self.notice = "操作未接受，已松开手动输入并保留使能："+reason
        raise ControlError(self.notice, status)

    def fresh(self):
        return self.latest is not None and 0 <= self.clock()-self.latest["stamp"] <= self.c["vision_max_age"]

    def poll(self):
        with self.lock:
            if self.token and self.clock() >= self.deadline:
                self.background(self.token)
                self.notice = "网页连接超时：手动输入已松开，相机继续跟踪／回零或保持"

    def telemetry(self):
        with self.lock:
            data = self.driver.telemetry()
            data.update(state=self.state(), visual_target=self.tracking.selected)
            return data

    def connect(self):
        with self.lock:
            if not self.fresh():
                raise ControlError("Waiting for fresh YOLO frames")
            self.poll()
            state = self.driver.snapshot()
            if (not self.token and self.local_owner and state["armed"] and self.driver.arm_requested
                    and state["reference_valid"] and not state["fault"] and not any(self.manual)):
                # Take over onboard tracking/home/hold without disarming or clearing ID.
                self.token = secrets.token_urlsafe(24)
                self.detached_token = None
                self.deadline, self.seq = self.clock()+self.lease_s, 0
                self.local_owner = False
                self.notice = "网页已接管；保持当前跟踪／回零／保持状态"
                return {"token": self.token, "state": self.state()}
            result = super().connect()
            self.detached_token = None
            self.local_owner = False
            return result

    def background(self, token):
        with self.lock:
            if not token or token != self.token:
                raise ControlError("控制连接已失效", 403)
            # Revoke the old session before accepting any later queued key command.
            self.detached_token = self.token
            self.token = None
            self.hold_start, self.blocked = None, False
            self.manual = (0., 0.)
            self.local_owner = True
            self.notice = "网页已离开焦点，手动输入已松开；相机继续跟踪／回零或保持"
            return self.state()

    def stop(self, token):
        with self.lock:
            # An explicit STOP racing the background handoff must work on either
            # side of revocation, but cannot stop a newly connected controller.
            if not self.token and token and token == self.detached_token:
                token = None
            result = super().stop(token)
            self.detached_token = None
            return result

    def state(self):
        with self.lock:
            value = super().state()
            value.update(vision_state=self.tracking.state, selected=self.tracking.selected,
                         last_control_rejection=self.last_control_rejection,
                         software_autotune=dict(active=self.software_autotune.active,
                                                state=self.software_autotune.session.state,
                                                reason=self.software_autotune.session.reason,
                                                saved_path=self.software_autotune.saved_path),
                         c_next_home=self.tracking.c_next_home,
                         frame_id=self.latest["id"] if self.latest else None,
                         vision_age=None if self.latest is None else self.clock()-self.latest["stamp"],
                         prediction_active=self.tracking.prediction_active,
                         prediction_center=self.tracking.prediction_point,
                         tracking_error_deg=tuple(e*f for e,f in zip(self.tracking.control_error,self.c['camera_fov'])),
                         tracking_aim_deg=self.tracking.aim_goal, tracking_centered=tuple(self.tracking.settled),
                         overhead=dict(self.tracking.overhead.snapshot(),reacquired=self.tracking.overhead_reacquired),
                         estimated_target_velocity=self.tracking.predictor.motion_velocity,
                         performance=dict(self.performance),
                         control_owner="WEB" if self.token else "LOCAL" if self.local_owner else "NONE")
            return value

    def offer_frame(self, tracks, width, height, stamp, capture_pose=...):
        with self.lock:
            if not (width > 0 and height > 0 and math.isfinite(stamp)):
                raise ValueError("Invalid frame")
            bounded = []
            for t in tracks[:self.c["max_tracks"]]:
                x, y, w, h = t["box"]
                if not all(math.isfinite(v) for v in (x, y, w, h)) or w <= 0 or h <= 0:
                    continue
                tr = dict(t)
                tr["key"] = tuple(t["key"])
                tr["cx"], tr["cy"] = (x+w/2)/width, (y+h/2)/height
                tr["norm_box"] = (x/width, y/height, w/width, h/height)
                bounded.append(tr)
            self.frame_id += 1
            frame = dict(id=self.frame_id, stamp=stamp, width=width, height=height, tracks=bounded)
            self.latest = frame
            self.frames.append(frame)
            if self.fresh():
                motor = self.driver.snapshot()
                if motor["armed"] and not motor["fault"] and not motor.get("home_requested"):
                    if capture_pose is Ellipsis:
                        capture_pose = motor["angles"] if motor["reference_valid"] and not motor.get("feedback_degraded") and motor["age"] is not None and motor["age"] <= self.c["pose_max_age_s"] else None
                    current_pose = motor["angles"] if motor["reference_valid"] and not motor.get("feedback_degraded") and motor["age"] is not None and motor["age"] <= self.c["pose_max_age_s"] else None
                    event = self.tracking.step(bounded, stamp, capture_pose, max(0., self.clock()-stamp),current_pose)
                    self.software_autotune.observe(frame, motor)
                    if event == "home":
                        self.motor.request_home()
                        self.notice = "目标丢失超过 2 秒，按 1101 逻辑释放并回原点"
            return frame

    def publish_media(self, frame, jpeg):
        with self.lock:
            self.video = dict(id=frame["id"], stamp=frame["stamp"], jpeg=bytes(jpeg))

    def offer_preview_image(self, frame, image):
        # Runtime hands off a finished overlay. Keep only the newest source.
        with self.lock:
            self.preview_source = (frame, image)

    def close_preview(self):
        with self.lock:
            self.preview_source = None
        with self.jpeg_lock:
            pass  # Let any in-flight encode release its image before camera teardown.

    def media(self):
        with self.lock:
            now = self.clock()
            cached = self.video
            if cached is not None and now-cached["stamp"] > self.c["vision_max_age"]:
                cached = None
            source = self.preview_source
            if (source is None or now-source[0]["stamp"] > self.c["vision_max_age"]
                    or (cached is not None and cached["id"] == source[0]["id"])
                    or now-self.last_jpeg_attempt < 1/self.c["web_preview_fps"]
                    or not self.jpeg_lock.acquire(blocking=False)):
                return cached
            self.last_jpeg_attempt = now
        try:
            frame, image = source
            started = self.clock()
            jpeg = image.to_jpeg(self.c["jpeg_quality"]).to_bytes()
            self.timings.add("web_encode", (self.clock()-started)*1000)
            with self.lock:
                self.performance["web_encode_ms"] = (self.clock()-started)*1000
                if self.clock()-frame["stamp"] > self.c["vision_max_age"]:
                    return None
                self.publish_media(frame, jpeg)
                return self.video
        finally:
            self.jpeg_lock.release()

    def select_at(self, frame_id, nx, ny):
        if type(frame_id) is not int or any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 1 for v in (nx, ny)):
            raise ControlError("Invalid click coordinates", 400)
        motor = self.driver.snapshot()
        if motor.get('angle_move',{}).get('active'):
            raise ControlError("请先停止角度测试，再选择跟踪目标")
        if not motor["armed"] or motor["fault"] or motor.get("home_requested") or motor.get("outside_hold") or not self.fresh():
            raise ControlError("先 ARM，等待 HOME 完成，并确认实时画面")
        frame = next((f for f in self.frames if f["id"] == frame_id), None)
        if frame is None or self.clock()-frame["stamp"] > self.c["vision_max_age"]:
            raise ControlError("点击画面已过期，请在最新画面重新选择")
        hits = []
        for tr in frame["tracks"]:
            x, y, w, h = tr["norm_box"]
            if x <= nx <= x+w and y <= ny <= y+h:
                hits.append(tr)
        if not hits:
            raise ControlError("请点击检测框内")
        chosen = min(hits, key=lambda t: t["norm_box"][2]*t["norm_box"][3])
        try:
            self.tracking.select(chosen["key"], self.latest["tracks"], self.clock())
        except RuntimeError as exc:
            raise ControlError(str(exc))
        self.manual = (0., 0.)
        self.notice = "已锁定 %s ID=%s" % (chosen["label"], chosen["key"])

    def c_action(self):
        if hasattr(self.driver,'cancel_angle_move'):
            self.driver.cancel_angle_move()
        if self.tracking.c_next_home:
            self.motor.request_home()
            self.notice = "第二次 C：返回原点，到位后保持使能"
        else:
            self.tracking.clear(arm_c=True)
            self.manual = (0., 0.)
            self.notice = "已清除目标并保持当前位置；再按一次 C 返回原点"

    def command(self, body):
        with self.lock:
            action = body.get("action")
            if action in ("pi", "record_start", "record_stop", "pi_autotune", "pi_autotune_cancel", "pi_autotune_save"):
                raise ControlError("使用独立电机调试程序调整 PI", 400)
            if self.software_autotune.active and (body.get("x") or body.get("y") or action in ("arm", "home", "zero")):
                self.software_autotune.abort("manual control requested")
            if action not in ("select", "c", "autotune", "autotune_cancel", "autotune_save", "angle_start", "angle_cancel", "inertia_start", "native_start"):
                value = super().command(body)
                if action == "arm":
                    self.notice = "ARM：使能并保持当前位置；HOME 单独回机械原点"
                    value["notice"] = self.notice
                return value
            if body.get("x") != 0 or body.get("y") != 0:
                raise ControlError("松开摇杆后选择目标或按 C", 400)
            request = dict(body, action="")
            super().command(request)  # Shared token/sequence/deadline checks.
            try:
                if action in ("angle_start","inertia_start","native_start"):
                    if self.software_autotune.active:
                        raise RuntimeError("先停止软件整定")
                    if not self.fresh():
                        raise RuntimeError("相机数据暂时过期，请等待恢复")
                    if action in ('inertia_start','native_start'):
                        hold=self.driver.snapshot().get('hold_target_deg')
                        yaw=hold[0] if hold else self.driver.snapshot()['angles'][0]
                        self.driver.request_angle_move(yaw,body.get('pitch'),body.get('speed'),inertia_test=True,
                                                       native_pitch=action=='native_start',native_accel=body.get('accel',3))
                    else:
                        self.driver.request_angle_move(body.get('yaw'),body.get('pitch'),body.get('speed'))
                    self.tracking.clear();self.manual=(0.,0.)
                    self.notice="角度测试已提交；相机本地执行，到位后保持使能"
                elif action == "angle_cancel":
                    self.driver.cancel_angle_move()
                    self.notice="已请求停止角度测试并保持使能"
                elif action == "autotune":
                    if self.driver.snapshot().get('angle_move',{}).get('active'):
                        raise RuntimeError("先停止角度测试")
                    self.software_autotune.start(body.get("axis"))
                elif action == "autotune_cancel":
                    self.software_autotune.abort()
                elif action == "autotune_save":
                    self.notice = "已保存软件整定结果 " + self.software_autotune.save()
                elif action == "c":
                    if self.software_autotune.active:
                        self.software_autotune.abort()
                    self.c_action()
                else:
                    if self.software_autotune.active:
                        self.software_autotune.abort()
                    self.select_at(body.get("frame_id"), body.get("nx"), body.get("ny"))
            except (RuntimeError, ValueError, OSError) as exc:
                raise ControlError(str(exc))
            return self.state()

    def local_action(self, action=None, vector=(0., 0.), click=None):
        with self.lock:
            self.poll()
            if self.software_autotune.active and (action or click or any(vector)):
                self.software_autotune.abort("local control requested")
            if action == "stop":
                self._stop("屏幕 DISARM：已请求失能")
                self.local_owner = False
                return
            if self.token:
                raise ControlError("网页正在控制；先 DISARM 再使用屏幕")
            self.local_owner = True
            if action == "arm":
                self.motor.request_arm(True)
            elif action == "zero":
                self.motor.confirm_reference()
            elif action == "recheck":
                self.motor.request_recheck()
            elif action == "home":
                self.motor.request_home()
            elif action == "c":
                self.c_action()
            elif click:
                self.select_at(*click)
            else:
                self.motor.input(*vector)

    def tick(self):
        with self.lock:
            self.poll()
            motor = self.driver.snapshot()
            if self.software_autotune.active and not motor["armed"]:
                self.software_autotune.abort("motor disabled")
            if motor["fault"]:
                if self.software_autotune.active:
                    self.software_autotune.abort("motor fault")
                self.tracking.clear()
                self.manual = (0., 0.)
            if motor.get('inertia_test_active'):
                self.driver.set_response_context('HOLD');self.driver.input(0.,0.)
                return  # The UART-owned test is independent of vision/PC latency.
            if not self.fresh():
                if self.software_autotune.active:
                    self.software_autotune.abort("vision feedback expired")
                if motor["armed"] or self.driver.arm_requested:
                    age = float("inf") if self.latest is None else self.clock()-self.latest["stamp"]
                    if age > self.c["vision_disable_age_s"]:
                        self.motor.request_arm(False,reason="VISION_TIMEOUT：相机／YOLO 数据 %.2fs 未更新" % age)
                        self.notice = "相机／YOLO 持续无有效画面，已请求失能"
                    else:
                        if hasattr(self.driver, "set_response_context"):
                            self.driver.set_response_context("HOLD")
                        self.driver.input(0., 0.)
                        self.notice = "相机／YOLO 暂时延迟，暂停运动并保持使能"
                return
            if not self.token and not self.local_owner:
                return
            if self.tracking.selected is not None:
                vector = self.tracking.drive(self.clock(),motor['angles'])
                context = self.tracking.state
                if self.c['native_tracking'] and context in ('COAST','OVERHEAD') and self.tracking.aim_goal is not None:
                    context='TRACK'  # Bounded local prediction / committed zenith relay; no browser dependency.
            else:
                vector = tuple(v*r/cap for v,r,cap in zip(self.manual, self.c["manual_rate"], self.c["max_rate"]))
                context = "MANUAL" if any(vector) else "HOLD"
            if self.c['native_tracking'] and context=='TRACK' and self.tracking.aim_goal is None:
                context,vector='HOLD',(0.,0.)
            if hasattr(self.driver, "set_response_context"):
                self.driver.set_response_context(context)
            if hasattr(self.driver,'offer_tracking_goal'):
                self.driver.offer_tracking_goal(self.tracking.aim_goal if context=='TRACK' else None,
                    self.latest['stamp'],self.tracking.native_active_axes,velocity=self.tracking.aim_velocity)
            self.driver.input(*vector)  # Only the live application loop feeds motor heartbeat.


WEB_HTML = '<!doctype html><html lang="zh-CN"><meta charset="utf-8">\n<meta name="viewport" content="width=device-width,initial-scale=1"><title>YOLO 点击跟踪</title>\n<style>\n*{box-sizing:border-box}body{margin:0;background:#0b1420;color:#e9f1f8;font:16px system-ui,sans-serif}\nmain{max-width:900px;margin:auto;padding:28px}h1{font-size:26px;margin-bottom:4px}.muted{color:#9cacbd}\n.panel{border:1px solid #2d4259;border-radius:16px;background:#131f2e;padding:22px;margin-top:18px}\n.row{display:flex;gap:26px;align-items:center;flex-wrap:wrap}button{font:inherit;cursor:pointer;border:0;border-radius:9px;padding:13px 20px;background:#285467;color:white;margin:5px 6px 5px 0}\nbutton:disabled{opacity:.4;cursor:default}.stop{background:#b23341}.big{font-size:32px;font-variant-numeric:tabular-nums}\n#pad{width:260px;height:260px;position:relative;border:3px solid #526e88;border-radius:50%;touch-action:none;user-select:none;background:linear-gradient(transparent 49.7%,#304052 50%,transparent 50.3%),linear-gradient(90deg,transparent 49.7%,#304052 50%,transparent 50.3%)}\n#knob{position:absolute;width:48px;height:48px;border-radius:50%;background:#28c8bf;left:103px;top:103px;pointer-events:none}\n#notice{color:#ffd18a;white-space:pre-wrap;overflow-wrap:anywhere}kbd{background:#2b4056;padding:2px 7px;border-radius:4px}\ninput[type=number]{width:112px;padding:10px;border:1px solid #526e88;border-radius:7px;background:#0b1420;color:#e9f1f8;font:inherit}label{display:inline-block;margin:6px 16px 6px 0}h2{font-size:20px;margin-top:0}\n</style><main><h1>YOLO · 点击跟踪</h1><div class="muted">MECH ONLY · 机械角度模式</div>\n<div id="buildInfo" class="muted"></div>\n<p id="mountInfo" class="muted">自动初始化回零 · 左右、上下反向 · PITCH −100°～+30° · 平滑位置控制</p>\n<p id="speedInfo" class="muted">摇杆 YAW 240°/s · PITCH 180°/s；HOME 108 / 90°/s</p>\n<p id="holdInfo" class="muted">保持参考：等待新鲜状态</p>\n<div class="panel"><div class="row"><div><span id="link">未连接控制</span> · <b id="phase">--</b></div><button id="connect">连接控制</button><button id="recheck">重新检查状态</button><button id="stop" class="stop">DISARM / 失能</button></div>\n<div id="notice">加载状态…</div></div>\n<div class="panel"><img id="video" style="display:block;width:100%;height:auto;cursor:crosshair" alt="等待相机画面"><p id="visionInfo">正在连接相机…</p><button id="clear">C：清除目标</button></div><div class="panel row"><div><div class="muted">↑ PITCH · YAW →</div><div id="pad" tabindex="0" aria-label="按住拖动摇杆"><div id="knob"></div></div></div><div>\n<div class="muted">YAW / 水平角度</div><div class="big" id="yaw">--°</div><br>\n<div class="muted">PITCH / 俯仰角度</div><div class="big" id="pitch">--°</div><p id="feedback" class="muted">--</p>\n<p id="mechanical" class="muted">机械反馈：--</p>\n<p id="zeroState">机械零点未确认</p><button id="zero">ZERO OK</button><button id="arm">ARM / 使能保持</button><button id="home">HOME / 回原点</button>\n<p id="armReason" class="muted"></p><p id="responseInfo" class="muted">指令／反馈误差：--</p><a href="/api/telemetry" download="gimbal_feedback.json" style="color:#65d4d0">下载指令与反馈日志</a></div></div>\n<section class="panel"><h2>软件自动整定</h2><p>选择静止目标并松开摇杆，测试会逐轴做约 ±4° 的视线偏移。完成复测后可以保存板端配置。</p><label>测试轴 <select id="autoAxis"><option value="1">Y / PITCH</option><option value="0">X / YAW</option></select></label><button id="autoStart">开始软件整定</button><button id="autoCancel">取消整定</button><button id="autoSave">保存验证结果</button><a href="/api/autotune" download="software-autotune.json" style="color:#65d4d0">导出报告</a><p id="autoStatus"></p></section>\n<section class="panel"><h2>目标角度测试</h2>\n<p class="muted">填写相对已保存机械零点的绝对角度。修改数值不会运动，点击“开始运动”后才执行。相机本地规划，到位后保持使能；网页离开焦点后继续执行。</p>\n<label>YAW / ° <input id="angleYaw" type="number" min="-150" max="150" step="0.1" value="0"></label>\n<label>PITCH / ° <input id="anglePitch" type="number" min="-100" max="30" step="0.1" value="0"></label>\n<label>速度上限 / °/s <input id="angleSpeed" type="number" min="1" max="180" step="1" value="30"></label>\n<div><button id="angleCurrent">填入当前角度</button><button id="angleStart">开始运动</button><button id="angleCancel">停止并保持</button></div>\n<p id="angleStatus">尚未开始角度测试</p><p id="angleTiming" class="muted">开始后显示请求往返与板端响应时间。</p>\n<p class="muted">正负方向与页面角度读数一致，不套用摇杆反向。停止并保持、摇杆、C 或 HOME 可中断；DISARM 请求失能。单次指令排除了持续网页输入的影响，仍包含板端与串口调度。</p></section>\n<p>先在图像中点击检测框锁定；按 C 清除目标，再按一次 C 回原点。按住摇杆或 <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> 转动；松开保持位置。<kbd>空格</kbd> 失能。</p>\n<p class="muted">单次连续移动最多 8 秒，达到后松开再按。离开窗口会松开手动输入，相机继续跟踪／回零或保持；返回后自动接回控制。DISARM／空格请求失能。HOME 完成后保持使能，累计角度不参与控制。</p></main>\n<script>\n\'use strict\';\nconst $=id=>document.getElementById(id), keys=new Set();\nconst CLIENT_VERSION=\'speed-track-v38\';\nlet token=null,seq=0,vector=[0,0],pointer=null,pending=\'\',busy=false,lastState=null,pendingSelection=null,displayedFrame=null;\nlet resumeOnFocus=false,detachedToken=null,commandError=\'\';\nlet angleClickTime=null,angleQueueMs=null,angleRequestMs=null;\nlet stateUnknown=false;\nfunction release(){vector=[0,0];keys.clear();pointer=null;draw();}\nfunction markStateUnknown(){\n stateUnknown=true;\n $(\'phase\').textContent=\'状态已过期\';$(\'link\').textContent=\'设备无响应 · 电机状态未确认\';\n $(\'yaw\').textContent=$(\'pitch\').textContent=\'--\';\n $(\'feedback\').textContent=\'未收到当前状态，请先恢复相机连接\';\n $(\'holdInfo\').textContent=\'保持参考与电机状态未确认\';\n $(\'angleStatus\').textContent=\'当前运动状态未确认；恢复连接后读取板端状态\';\n for(const id of [\'connect\',\'recheck\',\'arm\',\'home\',\'zero\',\'clear\',\'angleStart\',\'angleCancel\',\'angleCurrent\',\'autoStart\',\'autoCancel\',\'autoSave\'])$(id).disabled=true;\n}\nfunction draw(){ $(\'knob\').style.transform=`translate(${vector[0]*96}px,${-vector[1]*96}px)`; }\nfunction render(s){if(stateUnknown){commandError=\'\';stateUnknown=false;}lastState=s;$(\'visionInfo\').textContent=`${s.vision_state||\'DETECT\'}${s.prediction_active?\' / 预测补偿\':\'\'} · 目标 ${s.selected||\'无\'} · 控制源 ${s.control_owner||\'--\'}`;$(\'clear\').textContent=s.c_next_home?\'C：回原点\':\'C：清除目标\';$(\'clear\').disabled=!token;$(\'phase\').textContent=s.phase;$(\'link\').textContent=token?\'控制已连接\':s.connected?\'另一网页正在控制\':\'未连接控制\';\n $(\'buildInfo\').textContent=s.app_version?`界面 ${CLIENT_VERSION} · 程序 ${s.app_version}`:\'\';\n const perf=s.performance;if(perf)$(\'visionInfo\').textContent+=` · YOLO ${Number(perf.inference_fps_actual||0).toFixed(1)}fps · 延迟 ${Number(perf.capture_to_result_ms||0).toFixed(0)}ms`;\n if(s.limits&&s.joystick_signs)$(\'mountInfo\').textContent=`左右${s.joystick_signs[0]<0?\'反向\':\'正常\'} · 上下${s.joystick_signs[1]<0?\'反向\':\'正常\'} · ${s.continuous_yaw?\'YAW 360°连续旋转 · \':\'\'}PITCH ${s.limits[1][0]}°～${s.limits[1][1]}° · 平滑位置控制`;\n if(s.outside_work_limits)$(\'mountInfo\').textContent+=\' · 当前超出工作限位：ARM 保持，可向内手动调整或单独 HOME\';\n const trackRate=s.native_tracking_enabled&&s.native_tracking_rate?s.native_tracking_rate:s.tracking_rate;\n if(trackRate&&s.max_rate)$(\'speedInfo\').textContent=`摇杆 YAW ${s.max_rate[0]}°/s · PITCH ${s.max_rate[1]}°/s；跟踪 ${trackRate[0]} / ${trackRate[1]}°/s；HOME ${s.home_rate[0]} / ${s.home_rate[1]}°/s`;\n else if(s.max_rate&&s.home_rate)$(\'speedInfo\').textContent=`手动目标上限 YAW ${s.max_rate[0]}°/s · PITCH ${s.max_rate[1]}°/s；HOME ${s.home_rate[0]} / ${s.home_rate[1]}°/s`;\n if(s.tracking_motor_control===\'speed\')$(\'speedInfo\').textContent+=\' · 跟踪速度控制（RPM）\';\n if(s.native_tracking_enabled&&s.native_tracking_rate_limit===null)$(\'speedInfo\').textContent+=\' · 跟踪随电机速度档位\';\n if(s.overhead&&s.overhead.state!==\'NORMAL\')$(\'speedInfo\').textContent+=s.overhead.state===\'TURN\'?\' · 过顶转向中（保持相机正向）\':\' · 高仰角跟踪\';\n const valid=s.reference_valid&&s.control_feedback_seen!==false;\n $(\'holdInfo\').textContent=s.hold_fixed_active&&s.hold_target_deg?`锁定保持参考：YAW ${s.hold_target_deg[0].toFixed(1)}° / PITCH ${s.hold_target_deg[1].toFixed(1)}°；短时外力偏移后仍回此参考`:\'当前未处于固定角度保持；运动完成后锁定参考\';\n const degree=v=>Number.isFinite(v)?v.toFixed(1)+\'°\':\'--\';\n $(\'yaw\').textContent=valid?degree(s.angles[0]):\'--\';$(\'pitch\').textContent=valid?degree(s.angles[1]):\'--\';\n const age=Number.isFinite(s.age)?s.age.toFixed(2)+\'s\':\'尚无有效控制反馈\';\n $(\'feedback\').textContent=`控制反馈：${age} · REF ${s.reference_valid?\'VALID\':\'INVALID\'}`;\n const ma=s.mechanical_angles||[], da=s.mechanical_feedback_age||[];\n const diag=i=>`${degree(ma[i])}（${Number.isFinite(da[i])?da[i].toFixed(2)+\'s 前\':\'尚未收到\'}）`;\n $(\'mechanical\').textContent=`诊断机械反馈 YAW ${diag(0)} / PITCH ${diag(1)}`;\n const response=s.response;if(response){const pair=(v,d=1)=>(v||[]).map(x=>Number(x).toFixed(d)).join(\' / \');$(\'responseInfo\').textContent=`指令−反馈误差 ${s.armed?pair(response.error_deg)+\'°\':\'--（失能不调参）\'} · 软件加速度 ${pair(response.software_accel_deg_s2,0)}°/s² · 响应倍率 ${pair(response.drive_scale,2)} · ${(response.status||[]).join(\' / \')}`;}\n $(\'zeroState\').textContent=s.default_saved_zero&&s.zero_confirmed?\'默认使用驱动已保存的机械零点（未重新标定）\':s.startup_auto_home&&s.zero_confirmed?\'使用驱动已保存的机械零点（自动初始化回零）\':s.zero_confirmed?\'已确认现有机械零点\':\'机械零点未确认\';\n const auto=s.software_autotune||{active:false,state:\'IDLE\'};\n $(\'autoStart\').disabled=!token||!s.armed||!s.selected||auto.active||s.home_requested;\n $(\'autoCancel\').disabled=!token||!auto.active;\n $(\'autoSave\').disabled=!token||auto.active||auto.state!==\'COMPLETE\';\n $(\'autoStatus\').textContent=auto.state+(auto.reason?\' · \'+auto.reason:\'\')+(auto.saved_path?\' · 已保存 \'+auto.saved_path:\'\');\n const checking=[\'RECHECK_PENDING\',\'RECHECKING\'].includes(s.phase);\n const arming=!!s.arm_homing;\n $(\'notice\').textContent=s.outside_hold?\'ARM 已保持当前超限姿态；摇杆只允许向正常范围内调整，HOME 单独回零\':arming?\'正在从当前位置回机械零点（允许起始位置在工作限位外）；到位前屏蔽手动与目标选择\':checking?\'正在重新检查角度、限位与静止状态，请保持轴静止\':s.fault|| (s.hold_blocked?\'已达到连续移动上限，请松开后再按\':s.phase===\'HOME_HOLD\'?\'已回机械原点，保持使能；可继续摇杆控制，DISARM 请求失能\':s.recheck_status===\'PASSED\'&&!token?(s.zero_confirmed?\'重新检查通过，当前失能；连接控制 → ARM 或 HOME\':\'重新检查通过，当前失能；连接控制 → ZERO OK → 手动使能\'):s.notice);\n if(s.armed&&s.feedback_degraded&&!s.fault)$(\'notice\').textContent=`机械查询暂时失败：保持使能、暂停新运动；连续查询失败 ${Math.round((s.feedback_loss_grace_s||.1)*1000)}ms 才失能`;\n if(s.armed&&s.motion_warning&&!s.fault)$(\'notice\').textContent=\'暂时运动偏差：保持使能并复核；\'+s.motion_warning;\n else if(s.armed&&s.motion_paused&&!s.feedback_degraded&&!s.fault)$(\'notice\').textContent=\'控制暂时延迟，暂停新指令并保持使能\';\n if(!s.armed&&!s.arm_homing&&!s.fault&&!checking){$(\'notice\').textContent=\'当前已失能；\'+(s.last_disable_reason?s.last_disable_reason.reason:(s.notice||\'先 ARM 使能保持，再选择目标\'));}\n if(commandError&&!s.fault)$(\'notice\').textContent=commandError;\n const usable=!!token&&s.ready&&!s.fault;\n $(\'connect\').disabled=!!token||s.connected||!s.ready||!!s.fault||(s.armed&&s.control_owner!==\'LOCAL\'&&s.phase!==\'HOME_HOLD\');\n $(\'recheck\').disabled=!s.can_recheck||(s.connected&&!token);\n $(\'zero\').disabled=!usable||s.armed;$(\'arm\').disabled=!usable||s.armed||!s.zero_confirmed;\n $(\'home\').disabled=!usable||!s.armed||!s.zero_confirmed||s.home_requested;\n const move=s.angle_move||{active:false,state:\'IDLE\'},limits=s.limits||[[-150,150],[-100,30]];\n if(s.inertia_test_active)$(\'notice\').textContent=\'临时 pitch 惯性测试：相机本地执行，角度与反馈有效性门禁保留；停止测试后恢复常规保护\';\n for(const [i,id] of [\'angleYaw\',\'anglePitch\'].entries()){ $(id).min=i===0&&s.continuous_yaw?\'\':limits[i][0];$(id).max=i===0&&s.continuous_yaw?\'\':limits[i][1]; }\n $(\'angleSpeed\').max=Math.min(...(s.max_rate||[240,180]));\n $(\'angleStart\').disabled=!usable||!s.armed||!!s.home_requested||!!s.outside_hold||move.active||pending===\'angle_start\'||!!auto.active;\n $(\'angleCancel\').disabled=!usable||!move.active;\n $(\'angleCurrent\').disabled=!valid;\n const moveNames={IDLE:\'尚未开始\',PENDING:\'指令待执行\',MOVING:\'本地运动中\',SETTLING:\'到位稳定中\',COMPLETE:\'已到位，保持使能\',CANCELLED:\'已中断\'};\n $(\'angleStatus\').textContent=(moveNames[move.state]||move.state)+(move.target_deg?` · 目标 YAW ${move.target_deg[0]}° / PITCH ${move.target_deg[1]}°`:\'\')+(move.reason?\' · \'+move.reason:\'\');\n const ms=v=>Number.isFinite(v)?Math.round(v)+\'ms\':\'--\';\n $(\'angleTiming\').textContent=`网页排队 ${ms(angleQueueMs)} · 请求往返 ${ms(angleRequestMs)} · 板端开始 ${ms(move.queue_ms)} · 首次下发 ${ms(move.first_write_ms)} · 反馈位移≥0.3° ${ms(move.first_motion_ms)}`;\n $(\'armReason\').textContent=arming?\'回零完成后保持使能；DISARM 可中止回零\':checking?\'重新检查不会自动使能，也不会改机械零点\':s.fault?\'使能已阻止：\'+s.fault+(s.fault.includes(\'after disable\')?\'。失能后仍检测到运动／偏离零位，请支撑并检查负载，静止后点“重新检查状态”。\':\'。排查后点“重新检查状态”。\'):\n  !s.ready?\'等待电机就绪\':!token?\'先连接控制\':s.armed?\'已使能；DISARM / 空格请求失能\':\n  !s.zero_confirmed?\'确认已有机械零点正确，再点 ZERO OK\':\'当前失能；任意位置 ARM 使能保持，再单独 HOME 回机械零点\';\n}\nasync function post(path,body){const ctl=new AbortController(),timer=setTimeout(()=>ctl.abort(),450);\n try{const r=await fetch(path,{method:\'POST\',headers:{\'Content-Type\':\'application/json\'},body:JSON.stringify(body),signal:ctl.signal,cache:\'no-store\'});\n const j=await r.json();if(!r.ok){const err=Error(j.error||r.status);err.status=r.status;throw err;}return j;}finally{clearTimeout(timer);}}\nfunction stop(){resumeOnFocus=false;pendingSelection=null;const old=token||detachedToken;token=null;detachedToken=null;pending=\'\';release();navigator.sendBeacon(\'/api/stop\',new Blob([JSON.stringify({token:old})],{type:\'application/json\'}));\n $(\'link\').textContent=\'已请求失能\';}\nfunction background(){const old=token;release();pending=\'\';pendingSelection=null;if(!old)return;\n token=null;detachedToken=old;resumeOnFocus=true;\n const body=JSON.stringify({token:old});\n if(!navigator.sendBeacon(\'/api/background\',new Blob([body],{type:\'application/json\'})))fetch(\'/api/background\',{method:\'POST\',headers:{\'Content-Type\':\'application/json\'},body,keepalive:true}).catch(()=>{});\n $(\'link\').textContent=\'相机自主跟踪／保持；返回窗口后自动接回\';}\nasync function connectControl(){release();const j=await post(\'/api/connect\',{});token=j.token;detachedToken=null;seq=0;resumeOnFocus=false;commandError=\'\';render(j.state);if(document.hidden||!document.hasFocus())background();}\n$(\'connect\').onclick=async()=>{try{await connectControl();}catch(e){$(\'notice\').textContent=e.message;}};\n$(\'recheck\').onclick=async()=>{const old=token;release();pending=\'\';try{const s=await post(\'/api/recheck\',{token:old});token=null;seq=0;render(s);}catch(e){$(\'notice\').textContent=e.message;}};\n$(\'stop\').onclick=stop;\nfor(const action of [\'zero\',\'arm\',\'home\'])$(action).onclick=()=>{release();pending=action;};\n$(\'autoStart\').onclick=()=>{release();pendingSelection={axis:Number($(\'autoAxis\').value)};pending=\'autotune\';};\n$(\'autoCancel\').onclick=()=>{release();pending=\'autotune_cancel\';};\n$(\'autoSave\').onclick=()=>{release();pending=\'autotune_save\';};\n$(\'angleCurrent\').onclick=()=>{if(lastState?.reference_valid&&lastState.angles){$(\'angleYaw\').value=Number(lastState.angles[0]).toFixed(1);$(\'anglePitch\').value=Number(lastState.angles[1]).toFixed(1);}};\n$(\'angleStart\').onclick=()=>{\n if(!token||pending===\'angle_start\')return;\n const ids=[\'angleYaw\',\'anglePitch\',\'angleSpeed\'],raw=ids.map(id=>String($(id).value??\'\').trim()),v=raw.map(Number),limits=lastState?.limits||[[-150,150],[-100,30]],cap=Math.min(...(lastState?.max_rate||[240,180]));\n if(raw.some(x=>x===\'\')||v.some(x=>!Number.isFinite(x))||(!lastState?.continuous_yaw&&(v[0]<limits[0][0]||v[0]>limits[0][1]))||v[1]<limits[1][0]||v[1]>limits[1][1]||v[2]<1||v[2]>cap){commandError=\'请填写限位内的目标角度及有效速度\';$(\'notice\').textContent=commandError;return;}\n release();pendingSelection={yaw:v[0],pitch:v[1],speed:v[2]};pending=\'angle_start\';angleClickTime=Date.now();angleQueueMs=angleRequestMs=null;$(\'angleStart\').disabled=true;\n};\n$(\'angleCancel\').onclick=()=>{release();pendingSelection=null;pending=\'angle_cancel\';};\nconst pad=$(\'pad\');\nfunction point(e){const r=pad.getBoundingClientRect();let x=(e.clientX-r.left-r.width/2)/(r.width*.4),y=(r.top+r.height/2-e.clientY)/(r.height*.4);let d=Math.hypot(x,y);\n if(d<.15)vector=[0,0];else{const a=Math.min(1,(d-.15)/.85);vector=[x/d*a,y/d*a];}draw();}\npad.onpointerdown=e=>{if(!token||pointer!==null)return;e.preventDefault();keys.clear();pointer=e.pointerId;pad.setPointerCapture(pointer);point(e);};\npad.onpointermove=e=>{if(e.pointerId===pointer)point(e);};\npad.onpointerup=pad.onpointercancel=pad.onlostpointercapture=e=>{if(e.pointerId===pointer)release();};\nfunction keyVector(){let x=Number(keys.has(\'ArrowRight\'))-Number(keys.has(\'ArrowLeft\')),y=Number(keys.has(\'ArrowUp\'))-Number(keys.has(\'ArrowDown\')),n=Math.max(1,Math.hypot(x,y));vector=[x/n,y/n];draw();}\nfunction keyboardAllowed(e){return !e.defaultPrevented&&!e.ctrlKey&&!e.altKey&&!e.metaKey&&!document.hidden&&document.hasFocus()&&!e.target?.isContentEditable&&![\'INPUT\',\'TEXTAREA\',\'SELECT\'].includes(e.target?.tagName);}\nwindow.addEventListener(\'keydown\',e=>{if(!keyboardAllowed(e))return;if(e.code===\'Space\'){e.preventDefault();stop();return;}if(!token||!e.key.startsWith(\'Arrow\'))return;e.preventDefault();if(pointer===null){keys.add(e.key);keyVector();}});\nwindow.addEventListener(\'keyup\',e=>{if(keys.delete(e.key))keyVector();});\nwindow.addEventListener(\'blur\',background);window.addEventListener(\'pagehide\',background);\nwindow.addEventListener(\'focus\',()=>{tick();});\ndocument.addEventListener(\'visibilitychange\',()=>{if(document.hidden)background();else tick();});\nasync function tick(){if(busy)return;busy=true;const current=token;try{if(current){const action=pending;pending=\'\';const v=action?[0,0]:vector.slice();const extra=[\'select\',\'autotune\',\'angle_start\'].includes(action)?pendingSelection:{};pendingSelection=null;const sentAt=Date.now();if(action===\'angle_start\')angleQueueMs=sentAt-angleClickTime;const s=await post(\'/api/control\',{token:current,seq:++seq,x:v[0],y:v[1],action,...extra});if(action===\'angle_start\')angleRequestMs=Date.now()-sentAt;if(action||v.some(x=>x!==0))commandError=\'\';if(token===current)render(s);}\n else{if(document.hidden)return;const r=await fetch(\'/api/state\',{cache:\'no-store\',signal:AbortSignal.timeout(450)});const s=await r.json();render(s);\n if(resumeOnFocus&&!document.hidden&&document.hasFocus()&&!s.connected&&s.ready&&!s.fault&&s.control_owner===\'LOCAL\')await connectControl();}}\n catch(e){release();if(e.status>=400&&e.status<500&&e.status!==401&&e.status!==403){commandError=\'操作未接受：\'+e.message+\'；保持当前状态，请重新选择\';}\n else{if(current&&token===current)background();markStateUnknown();commandError=\'设备连接失败：\'+e.message+\'；已停止发送手动输入，交回控制请求是否到达尚未确认，电机状态未确认\';}\n $(\'notice\').textContent=commandError;}finally{busy=false;setTimeout(tick,token?80:500);}}\n\nfunction clearTarget(){if(!token)return;release();pendingSelection=null;pending=\'c\';}\n$(\'clear\').onclick=clearTarget;\n$(\'video\').onclick=e=>{if(lastState?.arm_homing||lastState?.home_requested){$(\'notice\').textContent=\'请等待回零完成\';return;}if(!token||displayedFrame===null){$(\'notice\').textContent=\'等待自动回零完成后连接控制，等待最新画面\';return;}\n if(!lastState?.armed||lastState?.outside_hold){commandError=lastState?.outside_hold?\'当前位置超限：先向内调整或单独 HOME，进入范围后再选目标\':\'当前失能：先 ARM，再选择目标\';$(\'notice\').textContent=commandError;return;}\n const r=$(\'video\').getBoundingClientRect();release();pendingSelection={frame_id:displayedFrame,nx:(e.clientX-r.left)/r.width,ny:(e.clientY-r.top)/r.height};pending=\'select\';};\nwindow.addEventListener(\'keydown\',e=>{if(token&&keyboardAllowed(e)&&e.code===\'KeyC\'&&!e.repeat){e.preventDefault();clearTarget();}});\nlet currentURL=null;\nasync function frameTick(){let nextURL=null;try{\n if(document.hidden)return;\n const r=await fetch(\'/api/frame\',{cache:\'no-store\',signal:AbortSignal.timeout(600)});if(!r.ok)throw Error(\'视频暂不可用\');\n const id=Number(r.headers.get(\'X-Frame-Id\')),blob=await r.blob();nextURL=URL.createObjectURL(blob);displayedFrame=null;\n await new Promise((resolve,reject)=>{const img=$(\'video\');img.onload=resolve;img.onerror=()=>reject(Error(\'图像解码失败\'));img.src=nextURL;});\n displayedFrame=id;if(currentURL)URL.revokeObjectURL(currentURL);currentURL=nextURL;nextURL=null;\n }catch(e){displayedFrame=null;$(\'visionInfo\').textContent=e.message;}finally{if(nextURL)URL.revokeObjectURL(nextURL);setTimeout(frameTick,250);}}\nframeTick();\n\ntick();\n</script></html>\n'


# Source: yolo_click_tracker\src\runtime.py
"""MaixCAM2 runtime. No microphone or sound-direction resources are opened."""
PANEL_BUTTONS = {name: (490, 42+i*36, 145, 32) for i, name in enumerate(
    ("arm", "stop", "zero", "recheck", "c", "home"))}


class LocalPanel:
    def __init__(self, c):
        self.c = c
        self.pressed = self.captured = False
        self.started = 0.

    def touch(self, hub, x, y, pressed, frame, now):
        edge = pressed and not self.pressed
        self.pressed = bool(pressed)
        if not pressed:
            if self.captured:
                self.captured = False
                hub.local_action(vector=(0, 0))
            return
        sx, sy, sw, sh = PANEL_BUTTONS["stop"]
        if sx <= x < sx+sw and sy <= y < sy+sh:
            self.captured = False
            hub.local_action("stop")
            return
        if edge:
            for name, (bx, by, bw, bh) in PANEL_BUTTONS.items():
                if bx <= x < bx+bw and by <= y < by+bh:
                    hub.local_action(name)
                    return
            if math.hypot(x-560, y-330) <= 57:
                self.captured, self.started = True, now
            elif frame:
                point = screen_to_normalized(x, y, contain_rect(frame["width"], frame["height"]))
                if point:
                    hub.local_action(click=(frame["id"], *point))
        if self.captured:
            dx, dy = (x-560)/50, (330-y)/50
            length = math.hypot(dx, dy)
            if now-self.started >= self.c["max_touch_hold_s"] or length < self.c["joystick_deadzone"]:
                vector = (0., 0.)
            else:
                strength = min(1., (length-self.c["joystick_deadzone"])/(1-self.c["joystick_deadzone"]))
                vector = (dx/length*strength, dy/length*strength)
            hub.local_action(vector=vector)


def prepare_image(img, c, model, camera_rotated=False):
    from maix import image
    if c["rotate_180"] and not camera_rotated:
        import cv2
        if img.format() != image.Format.FMT_RGB888:
            img = img.to_format(image.Format.FMT_RGB888)
        rgb = image.image2cv(img, ensure_bgr=False, copy=True)
        img = image.cv2image(cv2.rotate(rgb, cv2.ROTATE_180), bgr=False, copy=True)
    if img.format() != model.input_format():
        img = img.to_format(model.input_format())
    return img


def draw_visual(img, frame, selected, prediction=None):
    from maix import image
    green, yellow = image.Color.from_rgb(40, 230, 130), image.Color.from_rgb(255, 210, 60)
    for tr in frame["tracks"]:
        chosen = tuple(tr["key"]) == selected
        color = yellow if chosen else green
        x, y, w, h = map(int, tr["box"])
        img.draw_rect(x, y, w, h, color, 2)
        img.draw_string(max(0, x), max(0, y-18), "%s %d:%d %.2f" % (tr["label"], *tr["key"], tr["score"]), color)
        if chosen:
            for px, py in tr.get("trail", [])[-40:]:
                img.draw_circle(int(px), int(py), 2, yellow, -1)
            img.draw_line(int(tr["cx"]*img.width()), int(tr["cy"]*img.height()), img.width()//2, img.height()//2, yellow, 1)
    cx, cy = img.width()//2, img.height()//2
    img.draw_line(cx-10, cy, cx+10, cy, yellow, 1)
    img.draw_line(cx, cy-10, cx, cy+10, yellow, 1)
    if prediction is not None:
        px = int(clip(prediction[0], 0., 1.)*(img.width()-1))
        py = int(clip(prediction[1], 0., 1.)*(img.height()-1))
        color = image.Color.from_rgb(45, 195, 255)
        img.draw_circle(px, py, 5, color, 2)
        img.draw_string(max(0, px-12), max(0, py-18), "PRED", color)


def draw_panel(disp, img, frame, hub, timings=None, cache=None):
    from maix import image
    started = time.monotonic()
    state = hub.state()
    if timings is not None:
        timings.add("panel_state", (time.monotonic()-started)*1000)
    rgb = image.Color.from_rgb
    cache = {} if cache is None else cache
    if "canvas" not in cache:
        cache["canvas"] = image.Image(640, 480, image.Format.FMT_RGB888, bg=rgb(10, 16, 26))
    canvas = cache["canvas"]
    mode = "HOMING" if state.get("arm_homing") else state["vision_state"]
    header = "YOLO CLICK | %s | %s" % (mode, state["phase"])
    if cache.get("header") != header:
        canvas.draw_rect(0, 0, 640, 38, rgb(10, 16, 26), -1)
        canvas.draw_string(8, 10, header, rgb(240, 240, 240), scale=1.3)
        cache["header"] = header
    x, y, w, h = contain_rect(frame["width"], frame["height"])
    video_started = time.monotonic()
    if cache.get("frame_id") != frame["id"]:
        resized = img.resize(w, h)
        if timings is not None:
            timings.add("panel_resize", (time.monotonic()-video_started)*1000)
        canvas.draw_image(x, y, resized)
        cache["frame_id"] = frame["id"]
    if state["vision_age"] is None or state["vision_age"] > hub.c["vision_max_age"]:
        canvas.draw_string(10, 46, "VISION STALE - CLICK DISABLED", rgb(255, 70, 80), scale=1.3)
    labels = {"arm": "ARM / HOLD", "stop": "DISARM", "zero": "ZERO OK", "recheck": "RECHECK",
              "c": "C: HOME" if state["c_next_home"] else "C: CLEAR", "home": "HOME"}
    if cache.get("labels") != labels:
        for name, (bx, by, bw, bh) in PANEL_BUTTONS.items():
            if cache.get("labels", {}).get(name) == labels[name]:
                continue
            canvas.draw_rect(bx, by, bw, bh, rgb(155, 40, 50) if name == "stop" else rgb(35, 65, 85), -1)
            canvas.draw_string(bx+10, by+8, labels[name], rgb(240, 240, 240))
        cache["labels"] = labels
    vector = tuple(hub.manual)
    if cache.get("vector") != vector:
        canvas.draw_rect(490, 270, 150, 120, rgb(10, 16, 26), -1)
        canvas.draw_circle(560, 330, 55, rgb(70, 110, 140), 2)
        canvas.draw_circle(int(560+vector[0]*36), int(330-vector[1]*36), 12, rgb(40, 215, 205), -1)
        cache["vector"] = vector
    if cache.get("owner") != state["control_owner"]:
        canvas.draw_rect(490, 390, 150, 20, rgb(10, 16, 26), -1)
        canvas.draw_string(492, 395, state["control_owner"], rgb(120, 190, 210))
        cache["owner"] = state["control_owner"]
    important = (state["fault"], str(state["selected"]), hub.notice)
    if started-cache.get("footer_time", float("-inf")) >= .2 or cache.get("important") != important:
        canvas.draw_rect(0, 410, 640, 70, rgb(10, 16, 26), -1)
        canvas.draw_string(8, 412, "Y %.1f P %.1f | selected %s" % (*state["angles"], str(state["selected"])), rgb(230, 235, 240))
        canvas.draw_string(8, 438, str(state["fault"] or hub.notice)[:83], rgb(255, 195, 70))
        perf = state.get("performance", {})
        canvas.draw_string(8, 461, "YOLO %.1ffps | result age %.0fms | C clear, C again HOME" % (
            perf.get("inference_fps_actual", 0.), perf.get("capture_to_result_ms") or 0.), rgb(140, 160, 180))
        cache.update(footer_time=started, important=important)
    composed = time.monotonic()
    if timings is not None:
        timings.add("panel_compose", (composed-started)*1000)
    disp.show(canvas)
    if timings is not None:
        timings.add("display_show", (time.monotonic()-composed)*1000)


def tracker_main():
    from maix import app, camera, display, touchscreen, sys as maix_sys
    if str(maix_sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART3 mapping requires MaixCAM2")
    validate_visual_config(CONFIG)
    print("YOLO_CLICK VERSION", CONFIG["app_version"], flush=True)
    driver = VisualMotorService(dict(CONFIG))
    hub = TrackingHub(driver, CONFIG)
    try:
        hub.software_autotune.load()
    except (ValueError, OSError, TypeError, KeyError) as exc:
        print("SOFTWARE_AUTOTUNE profile load skipped", str(exc), flush=True)
    cam = disp = touch = server = server_thread = vision = pipeline = renderer = None
    frame = preview = None
    panel = LocalPanel(CONFIG)
    timings = StageTimings()
    try:
        print("YOLO_CLICK STEP model.load", CONFIG["model_path"], flush=True)
        vision = NativeVision(CONFIG)
        pipeline = DetectionPipeline(vision, CONFIG)
        print("YOLO_CLICK STEP camera.open", flush=True)
        cam = camera.Camera(vision.model.input_width(), vision.model.input_height(), vision.model.input_format(),
                            fps=CONFIG["camera_fps"], buff_num=CONFIG["camera_buffers"])
        camera_rotated = configure_camera_rotation(cam, CONFIG)
        print("NPU_MODEL %s %s %dx%d dual_buff=%s" % (
            CONFIG["model_kind"], CONFIG["model_path"], vision.model.input_width(), vision.model.input_height(), CONFIG["dual_buff"]), flush=True)
        disp, touch = display.Display(), touchscreen.TouchScreen()
        # Display.show also transports JPEG to the IDE when its preview is enabled.
        # Physical display remains full resolution; reduce only IDE JPEG quality.
        display.set_trans_image_quality(70)
        # Validate camera/model before starting the motor thread.
        result = None
        for _ in range(2 if CONFIG["dual_buff"] else 1):
            img = cam.read(block=True, block_ms=200)
            if img is None:
                raise RuntimeError("No initial camera frame")
            stamp = time.monotonic()
            img = prepare_image(img, CONFIG, vision.model, camera_rotated)
            result = pipeline.push(img, stamp, None)
        if result is None:
            raise RuntimeError("YOLO pipeline did not produce an initial frame")
        img, tracks, stamp, capture_pose = result
        result = None
        frame = hub.offer_frame(tracks, img.width(), img.height(), stamp, capture_pose)
        hub.performance = dict(pipeline.stats(), camera_rotation=camera_rotated)
        draw_visual(img, frame, None)
        panel_cache = {}
        draw_panel(disp, img, frame, hub, timings, panel_cache)
        preview = img
        if CONFIG["web_enabled"]:
            hub.offer_preview_image(frame, img)
        last_visual_id = frame["id"]
        def render(current, picture, selected, prediction):
            nonlocal last_visual_id
            if current["id"] != last_visual_id:
                started = time.monotonic()
                draw_visual(picture, current, selected, prediction)
                timings.add("overlay", (time.monotonic()-started)*1000)
                last_visual_id = current["id"]
                # The finished source is immutable from this point, including idle redraws.
                if server is not None:
                    hub.offer_preview_image(current, picture)
            draw_panel(disp, picture, current, hub, timings, panel_cache)
        renderer = LatestPreview(render, CONFIG["preview_fps"], timings)
        renderer.displayed = frame
        renderer.offer(frame, img, None, None)
        del img
        if CONFIG["web_enabled"]:
            server = make_http_server(hub, (WEB_CONFIG["host"], WEB_CONFIG["port"]))
        driver.start()
        renderer.thread.start()
        if server is not None:
            server_thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .05}, daemon=True)
            server_thread.start()
        print("YOLO_CLICK READY | no microphone | UART3 B2/B3 | " + (
            "AUTOMATIC STARTUP HOME: motor may move immediately" if CONFIG["startup_auto_home"] else "START DISABLED"), flush=True)
        if server is not None:
            try:
                import psutil
                ips = {a.address for values in psutil.net_if_addrs().values() for a in values
                       if a.family == socket.AF_INET and not a.address.startswith("127.")}
            except Exception:
                ips = {"CAMERA_IP"}
            for ip in sorted(ips):
                print("COMPUTER OPEN: http://%s:%d" % (ip, WEB_CONFIG["port"]), flush=True)
        last_infer = last_log = last_metrics = last_control_tick = 0.
        last_loop = time.monotonic()
        control_latencies = deque(maxlen=120)
        empty_camera_polls = 0
        while not app.need_exit():
            now = time.monotonic()
            timings.add("main_loop_interval", (now-last_loop)*1000)
            last_loop = now
            if renderer.error is not None:
                raise RuntimeError("Local preview failed") from renderer.error
            x, y, pressed = touch.read()
            try:
                panel.touch(hub, x*640/disp.width(), y*480/disp.height(), pressed, renderer.displayed, now)
            except (RuntimeError, ControlError) as exc:
                hub.notice = str(exc)
            control_ticked = False
            now = time.monotonic()
            if now-last_infer >= 1/CONFIG["inference_fps"]:
                # Empty polls leave the inference slot open for the next available frame.
                img = cam.read(block=False)
                capture_read_ms = (time.monotonic()-now)*1000
                timings.add("camera_read", capture_read_ms)
                if img is not None:
                    last_infer = now
                    stamp = time.monotonic()
                    capture_pose = driver.capture_pose()
                    vision.preview_target = hub.tracking.selected
                    prepared = time.monotonic()
                    img = prepare_image(img, CONFIG, vision.model, camera_rotated)
                    timings.add("prepare_image", (time.monotonic()-prepared)*1000)
                    result = pipeline.push(img, stamp, capture_pose)
                    timings.add("detect_and_track", pipeline.api_ms)
                    if result is None:
                        hub.tick()
                        continue
                    img, tracks, stamp, capture_pose = result
                    offered = time.monotonic()
                    current = hub.offer_frame(tracks, img.width(), img.height(), stamp, capture_pose)
                    timings.add("offer_frame", (time.monotonic()-offered)*1000)
                    hub.performance.update(pipeline.stats(), camera_rotation=camera_rotated,
                                           camera_read_ms=capture_read_ms)
                    tick_started = time.monotonic()
                    hub.tick()  # Submit control before handing the image to the display worker.
                    last_control_tick = time.monotonic()
                    timings.add("control_tick", (time.monotonic()-tick_started)*1000)
                    control_ticked = True
                    control_ms = (time.monotonic()-stamp)*1000
                    control_latencies.append(control_ms)
                    hub.performance["capture_to_control_tick_ms"] = control_ms
                    renderer.offer(current, img, hub.tracking.selected, hub.tracking.prediction_point)
                    del img
                else:
                    empty_camera_polls += 1
            if not control_ticked and time.monotonic()-last_control_tick >= .01:
                tick_started = time.monotonic()
                hub.tick()
                last_control_tick = time.monotonic()
                timings.add("control_tick", (last_control_tick-tick_started)*1000)
            if now-last_metrics >= 1:
                ordered = sorted(control_latencies)
                hub.performance.update(
                    camera_empty_polls=empty_camera_polls,
                    capture_to_control_samples=len(ordered),
                    capture_to_control_p50_ms=ordered[math.ceil(.5*len(ordered))-1] if ordered else None,
                    capture_to_control_p95_ms=ordered[math.ceil(.95*len(ordered))-1] if ordered else None,
                )
                hub.performance.update(renderer.stats(), stages=dict(timings.snapshot(), **hub.timings.snapshot(), **driver.timings.snapshot()),
                                       timestamp_basis="camera.read return; excludes sensor/ISP queue and LCD scanout")
                last_metrics = now
            if now-last_log >= 5:
                print("YOLO_CLICK " + json.dumps(dict(version=CONFIG["app_version"], phase=driver.phase,
                    fps=hub.performance.get("inference_fps_actual"), display_fps=hub.performance.get("local_preview_fps_actual"),
                    control_p95_ms=hub.performance.get("capture_to_control_p95_ms"), fault=driver.fault)), flush=True)
                last_log = now
            time.sleep(.003)
    finally:
        driver.request_arm(False,reason="SCRIPT_EXIT：脚本退出／异常清理")
        driver.close()
        if renderer is not None:
            renderer.close()
        preview = result = img = None
        if pipeline is not None:
            pipeline.close()
        if server is not None:
            if server_thread is not None:
                server.shutdown()
                server_thread.join(1)
            server.server_close()
        hub.close_preview()
        for resource in (touch, cam, disp):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    print("YOLO_CLICK CLOSE_FAILED", str(exc))
        vision = None
        print("YOLO_CLICK STOP: disable requested", flush=True)


if __name__ == "__main__":
    tracker_main()
