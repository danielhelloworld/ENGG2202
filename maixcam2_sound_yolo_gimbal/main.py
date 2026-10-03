# Generated standalone MaixVision entry. Edit CONFIG below for board use.

# ---- src\config.py ----
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


# ---- vendor\micarray.py ----
"""MaixCAM2 + MA-USB8 UART sound-direction display.

Open this single file in MaixVision and run it. The core classes are importable
without MaixPy, so another application can call SoundDirectionSensor.feed_bytes()
or poll() and read snapshot(). No camera, YOLO, thermal module or audio input.
"""

import math
import statistics
import time


# Standalone wiring: MA-USB8 TX0 -> MaixCAM2 A31 (UART1 RX),
# MA-USB8 RX0 -> A30 (UART1 TX), common GND. Power MA-USB8 by its USB-C.
# UART1 leaves Thermal160 UART2 and MaixVision communication UART4 alone.
UART_PORT = "/dev/ttyS1"
UART_BAUD = 2_000_000
ROTATION_DEG = 0  # Clockwise screen correction: 0/90/180/270.
MIRROR_X = False
DISPLAY_INTERVAL_S = 0.08
STALE_AFTER_S = 1.0
CENTER_DEAD_ZONE = 0.14  # About one 16x16 cell; do not invent a bearing near center.

HEADER = b"\xff" * 16
FRAME_SIZE = 272
MAX_BUFFER = FRAME_SIZE * 64
PIN_MAP = {
    "/dev/ttyS1": (("A30", "UART1_TX"), ("A31", "UART1_RX")),
    "/dev/ttyS2": (("B0", "UART2_TX"), ("B1", "UART2_RX")),
    "/dev/ttyS4": (("A21", "UART4_TX"), ("A22", "UART4_RX")),
}


class HotmapFrameParser:
    """Incremental 16xFF + 256-byte parser; confirms the following header."""

    def __init__(self):
        self.buffer = bytearray()
        self.frames = 0
        self.discarded_bytes = 0

    def feed(self, chunk):
        if not chunk:
            return []
        self.buffer.extend(chunk)
        if len(self.buffer) > MAX_BUFFER:
            drop = len(self.buffer) - MAX_BUFFER
            del self.buffer[:drop]
            self.discarded_bytes += drop
        output = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                drop = max(0, len(self.buffer) - len(HEADER) + 1)
                del self.buffer[:drop]
                self.discarded_bytes += drop
                break
            if start:
                del self.buffer[:start]
                self.discarded_bytes += start
            if len(self.buffer) < FRAME_SIZE + len(HEADER):
                break
            if self.buffer[FRAME_SIZE:FRAME_SIZE + len(HEADER)] != HEADER:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            output.append(bytes(self.buffer[16:FRAME_SIZE]))
            del self.buffer[:FRAME_SIZE]
            self.frames += 1
        return output


def orient_xy(x, y, rotation=0, mirror=False):
    """Transform x-right/y-up heatmap coordinates to mounted orientation."""
    if rotation not in (0, 90, 180, 270):
        raise ValueError("rotation must be 0/90/180/270")
    if rotation == 90:
        x, y = y, -x
    elif rotation == 180:
        x, y = -x, -y
    elif rotation == 270:
        x, y = -y, x
    return (-x if mirror else x), y


class SoundDirectionEstimator:
    """Estimate the dominant region in a hotmap; angle is NOT calibrated DOA."""

    def __init__(self, smoothing_s=0.18, stale_after_s=STALE_AFTER_S):
        self.smoothing_s = smoothing_s
        self.stale_after_s = stale_after_s
        self.last_time = None
        self.center = None
        self.last = {"valid": False, "reason": "no_data", "quality": 0.0}

    def update(self, payload, now=None):
        if len(payload) != 256:
            raise ValueError("hotmap payload must be 256 bytes")
        now = time.monotonic() if now is None else float(now)
        if self.last_time is not None and now < self.last_time:
            raise ValueError("timestamp moved backward")
        gap = 2.0 if self.last_time is None else now - self.last_time
        self.last_time = now
        values = list(payload)
        background = statistics.median(values)
        sigma = 1.4826 * statistics.median(abs(v - background) for v in values)
        contrast = max(values) - background
        threshold = max(6.0, 3.0 * sigma, 0.25 * contrast)
        hot = {i for i, value in enumerate(values) if value > background + threshold}
        components = []
        while hot:
            seed = hot.pop()
            stack = [seed]
            cells = [seed]
            while stack:
                index = stack.pop()
                row, col = divmod(index, 16)
                for rr in range(max(0, row - 1), min(16, row + 2)):
                    for cc in range(max(0, col - 1), min(16, col + 2)):
                        neighbour = rr * 16 + cc
                        if neighbour in hot:
                            hot.remove(neighbour)
                            stack.append(neighbour)
                            cells.append(neighbour)
            if len(cells) >= 3:
                weights = [(values[i] - background - threshold) ** 1.5 for i in cells]
                components.append((sum(weights), cells, weights))
        if not components:
            self.center = None
            self.last = {"valid": False, "reason": "low_contrast_or_noise", "quality": 0.0,
                         "background": background, "contrast": contrast, "component_count": 0}
            return self.snapshot(now)
        components.sort(key=lambda item: item[0], reverse=True)
        mass, cells, weights = components[0]
        dominance = mass / sum(item[0] for item in components)
        if dominance < 0.60:
            self.center = None
            self.last = {"valid": False, "reason": "ambiguous_multiple_sources", "quality": 0.0,
                         "background": background, "contrast": contrast,
                         "component_count": len(components), "dominance": dominance}
            return self.snapshot(now)
        x = sum(((i % 16) - 7.5) / 7.5 * weight for i, weight in zip(cells, weights)) / mass
        y = sum((7.5 - (i // 16)) / 7.5 * weight for i, weight in zip(cells, weights)) / mass
        if (self.center is None or gap >= 2 or self.smoothing_s <= 0
                or math.hypot(x - self.center[0], y - self.center[1]) > 0.5):
            self.center = (x, y)
        else:
            factor = -math.expm1(-gap / self.smoothing_s)
            self.center = (self.center[0] + (x - self.center[0]) * factor,
                           self.center[1] + (y - self.center[1]) * factor)
        rows = [i // 16 for i in cells]
        cols = [i % 16 for i in cells]
        self.last = {
            "valid": True, "reason": "ok", "vector_xy": self.center,
            "centroid_vector_xy": (x, y),
            "raw_bbox_rc": (min(rows), min(cols), max(rows) + 1, max(cols) + 1),
            "quality": dominance * min(1.0, contrast / max(12.0, 6.0 * sigma)),
            "dominance": dominance, "region_cells": len(cells),
            "background": background, "contrast": contrast, "component_count": len(components),
        }
        return self.snapshot(now)

    def snapshot(self, now=None, rotation=0, mirror=False):
        now = time.monotonic() if now is None else float(now)
        result = dict(self.last)
        age = None if self.last_time is None else max(0.0, now - self.last_time)
        result["age_seconds"] = age
        result["coordinate_system"] = "normalized_heatmap_x_right_y_up"
        result["rotation"] = rotation
        result["mirror"] = bool(mirror)
        if age is not None and age >= self.stale_after_s:
            result["valid"] = False
            result["reason"] = "stale"
        if not result["valid"]:
            for key in ("centroid_vector_xy", "raw_bbox_rc", "region_cells"):
                result.pop(key, None)
            result["vector_xy"] = None
            result["unit_direction_xy"] = None
            result["image_bearing_deg"] = None
            result["quality"] = 0.0
            return result
        x, y = orient_xy(*result["vector_xy"], rotation, mirror)
        cx, cy = orient_xy(*result["centroid_vector_xy"], rotation, mirror)
        result["vector_xy"] = [x, y]
        result["centroid_vector_xy"] = [cx, cy]
        length = math.hypot(x, y)
        result["unit_direction_xy"] = [x / length, y / length] if length >= CENTER_DEAD_ZONE else None
        result["image_bearing_deg"] = (math.degrees(math.atan2(x, y)) % 360
                                        if length >= CENTER_DEAD_ZONE else None)
        return result


class SoundDirectionSensor:
    """Reusable stream API: feed_bytes(), poll(), snapshot(), close().

    Instantiate without a serial port for integration with an external reader.
    open_uart() is intentionally explicit, so importing never claims a port.
    """

    def __init__(self, rotation=0, mirror=False, stale_after_s=STALE_AFTER_S):
        orient_xy(0, 0, rotation, mirror)
        self.rotation = rotation
        self.mirror = mirror
        self.parser = HotmapFrameParser()
        self.estimator = SoundDirectionEstimator(stale_after_s=stale_after_s)
        self.serial = None
        self.last_payload = None
        self.total_frames = 0

    def feed_bytes(self, data, now=None):
        now = time.monotonic() if now is None else float(now)
        frames = self.parser.feed(data)
        for payload in frames:
            self.last_payload = payload
            self.estimator.update(payload, now)
            self.total_frames += 1
        return len(frames)

    def open_uart(self, port=UART_PORT, baudrate=UART_BAUD, configure_pins=True):
        if self.serial is not None:
            raise RuntimeError("UART already open")
        from maix import err, pinmap
        from maix.peripheral import uart
        if configure_pins:
            if port not in PIN_MAP:
                raise ValueError("Unknown pin mapping; set configure_pins=False only after checking hardware")
            for pin, function in PIN_MAP[port]:
                err.check_raise(pinmap.set_pin_function(pin, function), "pinmap " + pin)
        self.serial = uart.UART(port=port, baudrate=baudrate)
        return self.serial

    def poll(self, read_size=4096, timeout_ms=2):
        if self.serial is None:
            raise RuntimeError("UART not open")
        try:
            data = self.serial.read(read_size, timeout=timeout_ms)
        except TypeError:
            data = self.serial.read(read_size, timeout_ms)
        return self.feed_bytes(bytes(data)) if data else 0

    def snapshot(self, now=None):
        result = self.estimator.snapshot(now, self.rotation, self.mirror)
        result["frames"] = self.total_frames
        result["discarded_bytes"] = self.parser.discarded_bytes
        return result

    def close(self):
        serial, self.serial = self.serial, None
        if serial is not None:
            serial.close()


# ---- vendor\protocol.py ----
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


# ---- src\core.py ----
"""Pure sound -> slew -> settle -> search -> locked-ID visual controller."""
import math


def clip(value, low, high):
    return max(low, min(high, value))


def sound_to_angles(sound, c):
    if not c["sound_calibrated"] or not sound.get("valid") or sound.get("quality", 0) < c["sound_quality"]:
        return None
    age = sound.get("age_seconds")
    vector = sound.get("vector_xy")
    if age is None or not 0 <= age <= c["sound_fresh_s"] or vector is None:
        return None
    if not all(math.isfinite(v) for v in vector):
        return None
    neighbours = sorted((math.hypot(vector[0] - s[0], vector[1] - s[1]), s) for s in c["sound_samples"])
    neighbours = [(d, s) for d, s in neighbours[:3] if d <= c["sound_radius"]]
    if not neighbours:
        return None  # No extrapolation outside measured neighbourhoods.
    # Reject overlapping calibration regions with incompatible directions.
    if any(max(s[i] for _, s in neighbours) - min(s[i] for _, s in neighbours) > 35 for i in (2, 3)):
        return None
    if neighbours[0][0] < 1e-6:
        return tuple(neighbours[0][1][2:])
    weights = [1 / max(d * d, 1e-8) for d, _ in neighbours]
    return tuple(sum(w * s[i] for w, (_, s) in zip(weights, neighbours)) / sum(weights) for i in (2, 3))


class SoundVisualController:
    def __init__(self, c):
        self.c = c
        self.state = "WAIT_SOUND"
        self.selected = None
        self.last_seen = None
        self.target = None
        self.anchor = None
        self.since = 0.0
        self.settled_at = None
        self.sound_start = None
        self.sound_anchor = None
        self.last_sound_frame = -1
        self.candidate = None
        self.hits = 0
        self.last_tick = None
        self.last_visual = None
        self.filtered = [0.0, 0.0]
        self.reason = "waiting for calibrated sound"

    def reset(self, now, angles, sound_frame=-1):
        self.state, self.selected = "WAIT_SOUND", None
        self.target = tuple(angles)
        self.since = now
        self.sound_start = self.sound_anchor = None
        self.last_sound_frame = sound_frame
        self.candidate, self.hits = None, 0
        self.filtered = [0.0, 0.0]
        self.last_visual = None
        self.reason = "waiting for new sound"

    def select(self, key, tracks, now, angles):
        if self.state == "FAULT" or not any(t["key"] == key for t in tracks):
            return False
        self.selected, self.last_seen, self.state = key, now, "TRACK"
        self.target = tuple(angles)
        self.filtered = [0.0, 0.0]
        self.last_visual = now
        return True

    def fail(self, reason):
        self.state, self.reason, self.selected = "FAULT", reason, None

    def _bounded(self, target):
        return tuple(clip(v, *limits) for v, limits in zip(target, self.c["limits"]))

    def step(self, now, sound, tracks, angles, vision_fresh=True, new_vision=True):
        dt = min(.15, max(0.0, now - self.last_tick)) if self.last_tick is not None else .1
        self.last_tick = now
        if self.target is None:
            self.target = tuple(angles)
        if self.state == "FAULT":
            return None
        frame = sound.get("frames", -1)
        if self.state == "WAIT_SOUND":
            direction = sound_to_angles(sound, self.c)
            if direction is None:
                self.sound_start = self.sound_anchor = None
                self.reason = "sound invalid, stale or outside calibration"
            elif frame != self.last_sound_frame:
                self.last_sound_frame = frame
                if self.sound_anchor is None or max(abs(a-b) for a, b in zip(direction, self.sound_anchor)) > self.c["sound_stable_deg"]:
                    self.sound_start, self.sound_anchor = now, direction
                elif now - self.sound_start >= self.c["sound_stable_s"]:
                    # FIXED BASE: calibrated absolute direction, never add present yaw.
                    self.target = self.anchor = self._bounded(direction)
                    self.state, self.since, self.settled_at = "SLEW", now, None
                    self.reason = "turning to sound direction"
        elif self.state == "SLEW":
            if now - self.since > self.c["turn_timeout"]:
                self.fail("motor did not reach sound direction")
                return None
            if max(abs(a-b) for a, b in zip(angles, self.target)) <= self.c["settle_tolerance"]:
                if self.settled_at is None:
                    self.settled_at = now
                if now - self.settled_at >= self.c["settle_s"]:
                    self.state, self.since = "SEARCH", now
                    self.reason = "looking near sound direction"
            else:
                self.settled_at = None
        elif self.state == "SEARCH":
            if now - self.since > self.c["search_timeout"]:
                self.reset(now, angles, frame)
            else:
                radius = min(self.c["search_amplitude"], self.c["search_rate"] * self.c["search_timeout"] / (2 * math.pi))
                # A continuous bounded sweep beginning at the sound direction.
                offset = radius * math.sin((now - self.since) * 2 * math.pi / self.c["search_timeout"])
                self.target = self._bounded((self.anchor[0] + offset, self.anchor[1]))
                if new_vision:
                    ranked = sorted((math.hypot((t["cx"]-.5)*2, (t["cy"]-.5)*2), t["key"], t) for t in tracks) if vision_fresh else []
                    valid = bool(ranked and ranked[0][0] <= self.c["acquire_radius"])
                    if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < self.c["ambiguity_margin"]:
                        valid = False
                        self.reason = "ambiguous targets: tap one"
                    key = ranked[0][1] if valid else None
                    self.hits = self.hits + 1 if key is not None and key == self.candidate else (1 if key is not None else 0)
                    self.candidate = key
                    if self.hits >= self.c["confirmation_frames"]:
                        self.select(key, tracks, now, angles)
        elif self.state in ("TRACK", "LOST"):
            found = next((t for t in tracks if t["key"] == self.selected), None) if vision_fresh else None
            if found is None:
                if self.state != "LOST":
                    self.target = tuple(angles)  # Cancel the previous moving target.
                    self.filtered = [0.0, 0.0]
                self.state, self.reason = "LOST", "holding for same ID"
                if now - self.last_seen >= self.c["lost_timeout"]:
                    self.reset(now, angles, frame)
            elif new_vision:
                dt = min(.2, max(0.0, now - self.last_visual)) if self.last_visual is not None else .1
                self.last_visual = now
                self.last_seen, self.state, self.reason = now, "TRACK", "tracking selected ID"
                errors = (found["cx"] - .5, found["cy"] - .5)
                result = []
                for axis in range(2):
                    err = errors[axis]
                    if abs(err) < self.c["deadzone"][axis]:
                        self.filtered[axis] = 0.0
                        result.append(angles[axis])
                        continue
                    angle_error = math.degrees(math.atan(2 * err * math.tan(math.radians(self.c["camera_fov"][axis] / 2))))
                    correction = angle_error * self.c["pixel_sign"][axis] * self.c["visual_gain"][axis]
                    alpha = 1 - (1 - self.c["ema"][axis]) ** (dt * self.c["inference_fps"])
                    self.filtered[axis] += alpha * (correction - self.filtered[axis])
                    cap = min(self.c["step_cap"][axis], self.c["max_rate"][axis] * dt)
                    result.append(angles[axis] + clip(self.filtered[axis], -cap, cap))
                self.target = self._bounded(result)
        return self.target


# ---- src\motor.py ----
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
                    return raw / 10.0
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
        angles = tuple((r-z)*s for r, z, s in zip(raw, self.reference_zero, self.c["motor_signs"]))
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


# ---- src\vision.py ----
"""Native Maix YOLO + per-class ByteTrack (mainweb behavioral port)."""


class NativeVision:
    def __init__(self, c):
        from maix import nn, tracker
        self.c, self.api = c, tracker
        if c["model_kind"] not in ("YOLO11", "YOLOv8"):
            raise ValueError("Choose YOLO11 or YOLOv8 to match the .mud model")
        self.model = getattr(nn, c["model_kind"])(model=c["model_path"], dual_buff=False)
        self.trackers = {}

    def detect(self, img):
        objects = self.model.detect(img, conf_th=self.c["confidence"], iou_th=self.c["iou"])
        grouped = {}
        for obj in objects:
            cid = int(obj.class_id)
            if self.c["class_ids"] is not None and cid not in self.c["class_ids"]:
                continue
            grouped.setdefault(cid, []).append(self.api.Object(obj.x, obj.y, obj.w, obj.h, cid, obj.score))
        for cid in grouped:
            if cid not in self.trackers:
                # Lost buffer is frames; actual control timeout below is wall-clock 2s.
                self.trackers[cid] = self.api.ByteTracker(max(1, int(self.c["inference_fps"] * self.c["lost_timeout"])), .5, .5, .8, 60)
        output = []
        width, height = img.width(), img.height()
        for cid, tracker_instance in self.trackers.items():
            for track in tracker_instance.update(grouped.get(cid, [])):
                if track.lost or not track.history:
                    continue
                obj = track.history[-1]
                output.append({"key": (cid, int(track.id)), "box": (obj.x, obj.y, obj.w, obj.h),
                               "cx": (obj.x + obj.w/2) / width, "cy": (obj.y + obj.h/2) / height,
                               "score": float(track.score), "label": self.model.labels[cid],
                               "trail": [(o.x + o.w/2, o.y + o.h/2) for o in track.history]})
        return output


def touch_to_image(x, y, iw, ih, dw, dh):
    scale = min(dw / iw, dh / ih)
    return ((x - (dw - iw*scale)/2) / scale, (y - (dh - ih*scale)/2) / scale)


def draw_tracking(img, tracks, controller, sound, motor, fps):
    from maix import image
    green = image.Color.from_rgb(60, 240, 110)
    yellow = image.Color.from_rgb(255, 210, 50)
    white = image.Color.from_rgb(235, 240, 250)
    for tr in tracks:
        selected = tr["key"] == controller.selected
        color = yellow if selected else green
        x, y, w, h = map(int, tr["box"])
        img.draw_rect(x, y, w, h, color, 2)
        img.draw_string(x, max(42, y-18), "%s %d:%d %.2f" % (tr["label"], *tr["key"], tr["score"]), color)
        if selected:
            for px, py in tr["trail"]:
                img.draw_circle(int(px), int(py), 2, yellow, -1)
            img.draw_line(int(tr["cx"]*img.width()), int(tr["cy"]*img.height()), img.width()//2, img.height()//2, yellow, 2)
    cx, cy = img.width()//2, img.height()//2
    img.draw_line(cx-12, cy, cx+12, cy, white, 1)
    img.draw_line(cx, cy-12, cx, cy+12, white, 1)
    img.draw_rect(0, 0, img.width(), 38, image.COLOR_BLACK, -1)
    img.draw_string(4, 3, "STOP   CLEAR   %s   %s" % ("DISARM" if motor["armed"] else "ARM", controller.state), yellow)
    img.draw_string(4, 20, "%s Y%.1f P%.1f %.1ffps" % ("SIM" if motor["simulated"] else "MOTOR", *motor["angles"], fps), white)
    img.draw_rect(0, img.height()-36, img.width(), 36, image.COLOR_BLACK, -1)
    vector = sound.get("vector_xy")
    mic_text = "MIC %s Q%.2f" % (str(vector and [round(v, 2) for v in vector]), sound.get("quality", 0))
    img.draw_string(4, img.height()-34, mic_text, white)
    img.draw_string(4, img.height()-17, controller.reason[:80], yellow)


# ---- src\runtime.py ----
"""MaixVision entry; all hardware construction is inside run_board()."""
import json


def run_board(c=CONFIG):
    validate_config(c)
    from maix import app, camera, display, touchscreen, image, sys as maix_sys
    if str(maix_sys.device_name()).lower() != "maixcam2":
        raise RuntimeError("This application and pin assignment require MaixCAM2")
    sensor = SoundDirectionSensor(c["mic_rotation"], c["mic_mirror"], c["sound_fresh_s"])
    motor = MotorService(c)
    controller = SoundVisualController(c)
    cam = disp = touch = vision = None
    try:
        print("STEP model.load", c["model_path"])
        vision = NativeVision(c)
        print("STEP camera.open")
        cam = camera.Camera(vision.model.input_width(), vision.model.input_height(), vision.model.input_format())
        disp, touch = display.Display(), touchscreen.TouchScreen()
        print("STEP microphone.open", c["mic_port"])
        sensor.open_uart(c["mic_port"], c["mic_baud"])
        # Prove camera and inference return before starting the motor service.
        print("STEP camera.first_frame")
        first = cam.read(block=True, block_ms=200)
        if first is None:
            raise RuntimeError("No first camera frame")
        print("STEP detector.first_frame")
        vision.detect(first)
        del first
        print("STEP motor.start", "REAL" if c["motor_enabled"] else "SIMULATION")
        motor.start()
        deadline = time.monotonic() + 3
        while not motor.snapshot()["ready"]:
            state = motor.snapshot()
            if state["fault"] or time.monotonic() > deadline or app.need_exit():
                raise RuntimeError(state["fault"] or "Motor initialization interrupted/timed out")
            time.sleep(.01)
        pressed_before = False
        last_log = last_frame = time.monotonic()
        last_inference = 0.0
        last_result_time = 0.0
        tracks, fps = [], 0.0
        previously_armed = False
        while not app.need_exit():
            sensor.poll()
            img = cam.read(block=True, block_ms=200)
            now = time.monotonic()
            if img is None:
                if now - last_frame > c["vision_max_age"]:
                    raise RuntimeError("Camera stream stale")
                continue  # Never refresh motor heartbeat without a current camera frame.
            last_frame = now
            new_vision = now - last_inference >= 1 / c["inference_fps"]
            if new_vision:
                captured = now
                tracks = vision.detect(img)
                now = time.monotonic()
                fps = 1 / max(.001, now - last_inference) if last_inference else 0.0
                last_inference, last_result_time = now, captured
            fresh = now - last_result_time <= c["vision_max_age"]
            if not fresh:
                tracks = []
                raise RuntimeError("YOLO result stale; motor stopped")
            state = motor.snapshot()
            if state["fault"]:
                controller.fail(state["fault"])
            elif not state["ready"] or state["age"] > c["watchdog_s"]:
                raise RuntimeError("Motor feedback stale")
            sound = sensor.snapshot(now)
            if not state["fault"] and state["armed"] != previously_armed:
                controller.reset(now, state["angles"], sound.get("frames", -1))
            previously_armed = state["armed"]
            tx, ty, pressed = touch.read()
            if pressed and not pressed_before:
                x, y = touch_to_image(tx, ty, img.width(), img.height(), disp.width(), disp.height())
                if 0 <= y < 38 and 0 <= x < 62:
                    break
                elif 0 <= y < 38 and 62 <= x < 132 and not state["fault"]:
                    controller.reset(now, state["angles"], sound.get("frames", -1))
                elif 0 <= y < 38 and 132 <= x < 228 and not state["fault"]:
                    motor.request_arm(not state["armed"])
                    controller.reset(now, state["angles"], sound.get("frames", -1))
                elif y >= 38:
                    hits = [t for t in tracks if t["box"][0] <= x <= t["box"][0]+t["box"][2] and t["box"][1] <= y <= t["box"][1]+t["box"][3]]
                    if hits and state["armed"] and not state["fault"]:
                        chosen = min(hits, key=lambda t: t["box"][2]*t["box"][3])
                        controller.select(chosen["key"], tracks, now, state["angles"])
            pressed_before = pressed
            target = None
            if not state["fault"]:
                if state["armed"]:
                    target = controller.step(now, sound, tracks, state["angles"], fresh, new_vision)
                    if target is None:
                        raise RuntimeError(controller.reason)
                    motor.submit(target)
                else:
                    controller.reset(now, state["angles"], sound.get("frames", -1))
                    controller.state, controller.reason = "DISARMED", "feedback only; tap ARM to start"
            draw_tracking(img, tracks, controller, sound, state, fps)
            disp.show(img, fit=image.Fit.FIT_CONTAIN)
            if now - last_log >= 1:
                print("SOUND_YOLO", json.dumps({"state": controller.state, "selected": controller.selected,
                      "sound": sound, "motor": state, "target": target, "fps": fps}))
                last_log = now
            del img
    except Exception as exc:
        controller.fail(str(exc))
        print("SYSTEM_FAULT", str(exc))
        raise
    finally:
        # Stop motors before closing peripherals, including partial-init failures.
        motor.close()
        for resource in (sensor, cam, touch, disp):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    print("CLOSE_FAILED", str(exc))
        vision = None
        print("SOUND_YOLO_STOP")


if __name__ == "__main__":
    run_board()
