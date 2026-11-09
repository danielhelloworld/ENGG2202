# Generated standalone MaixCAM2 browser controller. Edit CONFIG / TUNING_CONFIG / WEB_CONFIG.
# Default: enables motors and returns to saved mechanical zero automatically.


# ---- src/config.py ----
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
    if len(c["command_accel_deg_s2"]) != 2 or any(not math.isfinite(v) or not 0 < v <= 100 for v in c["command_accel_deg_s2"]):
        raise ValueError("Invalid software command acceleration")
    if len(c["limits"]) != 2:
        raise ValueError("Two limits required")
    for lo, hi in c["limits"]:
        if not all(math.isfinite(v) for v in (lo, hi)) or not -180 < lo < 0 < hi < 180:
            raise ValueError("Limits must include zero and stay inside signed single-turn range")
    if c["limits"][1][0] < -100 or c["limits"][1][1] > 30:
        raise ValueError("Pitch must stay within -100..30")
    if not 1 <= c["motor_rpm"] <= 5 or not 1 <= c["motor_acceleration"] <= 30:
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


# ---- vendor/protocol.py ----
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


# ---- vendor/motor_base.py ----
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


# ---- src/service.py ----
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


# ---- src/mechanical_only.py ----
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
                    sent[axis] = round(signed_mechanical(units/10.0)*sign, 7)
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


# ---- src/autotune_core.py ----
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


# ---- src/pi_autotune.py ----
"""F32C volatile PI search, advanced exclusively by the existing UART owner."""
import json
import os


class F32CPiAutoTuner:
    def __init__(self, service):
        self.service = service
        self.axis = None
        self.origin = None
        self.target = None
        self.last_sample = None
        self.detached = False
        self.saved_path = None
        self.last_applied = None
        self.session = AutoTuneExperiment(self._apply, self._offset, amplitude=1., segment_s=1.5)

    @property
    def active(self):
        return self.session.active

    def check_start(self, axis, kp, ki):
        s = self.service
        if type(axis) is not int or axis not in (0, 1):
            raise ValueError("Choose one PI tuning axis")
        if any(type(v) is not int or not 0 <= v <= 256 for v in (kp, ki)):
            raise ValueError("Known baseline KP and KI in 0..256 are required")
        if (self.active or not s.armed or not s.arm_requested or not s.ready or s.fault
                or s.home_requested or s.startup_active or any(s.joystick)
                or s.pi_pending is not None or s.pi_applying):
            raise RuntimeError("PI auto tune requires an idle, enabled hold")
        if s.c["position_mode"] != 4:
            raise RuntimeError("PI response trials require single-turn direct mode 4")
        if not s.reference_valid or time.monotonic()-s.feedback_time > s.c["reference_max_age_s"]:
            raise RuntimeError("Fresh mechanical feedback is required")
        origin = tuple(s.sent_target)
        if any(abs(a-b) > .3 for a, b in zip(origin, s.angles)):
            raise RuntimeError("Wait for the existing target to settle")
        if any(not lo+3 <= q <= hi-3 for q, (lo, hi) in zip(origin, s.c["limits"])):
            raise RuntimeError("Move at least 3 degrees inside limits before PI auto tune")
        return origin

    def start(self, axis, kp, ki, now):
        origin = self.check_start(axis, kp, ki)
        self.axis, self.origin = axis, origin
        self.last_sample = None
        self.saved_path = None
        self.last_applied = None
        baseline = dict(kp=kp, ki=ki)
        candidates = []
        for key in ("kp", "ki"):
            for direction in (-1, 1):
                profile = dict(baseline)
                profile[key] = max(0, min(256, profile[key]+direction*max(1, round(profile[key]*.15))))
                if profile != baseline and profile not in candidates:
                    candidates.append(profile)
        self.session.start(baseline, candidates, now)

    def _apply(self, profile):
        s = self.service
        s._heartbeat_check()
        s._require_reference(fresh=True)
        mid = s.c["motor_ids"][self.axis]
        if s._feedback(mid, FEEDBACK_SPEED) != 0:
            raise RuntimeError("Motor is still moving; PI profile not applied")
        old = self.last_applied
        self.last_applied = None
        s.pi_last_sent[self.axis] = None
        s.pi_applying = True
        try:
            for index, key in enumerate(("kp", "ki")):
                if old is None or old[index] != profile[key]:
                    s._write(_frame(mid, (0x0F, 0x10)[index], profile[key].to_bytes(2, "big")))
            s.pi_last_sent[self.axis] = [profile["kp"], profile["ki"]]
            self.last_applied = list(s.pi_last_sent[self.axis])
            s.pi_status = "自动整定临时 PI 已发送，等待响应测试"
        finally:
            s.pi_applying = False

    def _offset(self, degrees):
        s = self.service
        q = self.origin[self.axis]+degrees
        mid = s.c["motor_ids"][self.axis]
        packet = set_single_turn_position(mid, q/s.c["motor_signs"][self.axis])
        units = int.from_bytes(packet[3:5], "big")
        if units != s.position_units[self.axis]:
            s._write(packet)  # Existing reference, lead-distance and limit gate.
            s.position_units[self.axis] = units
            s.last_position_send = time.monotonic()
        target = list(s.sent_target)
        target[self.axis] = round(signed_mechanical(units/10.)*s.c["motor_signs"][self.axis], 7)
        self.target = target[self.axis]
        s.sent_target = s.manual_target = s.stream_target = tuple(target)
        s.stream_velocity = (0., 0.)

    def advance(self, now):
        if not self.active:
            return
        if now <= self.session.last_stamp:
            return
        s = self.service
        valid = (s.armed and s.arm_requested and not s.fault and s.reference_valid
                 and now-s.feedback_time <= s.c["reference_max_age_s"] and not s.home_requested
                 and all(abs(a-b) <= 2.5 for a, b in zip(s.angles, self.origin)))
        speed = 0.
        if self.last_sample is not None and now > self.last_sample[0]:
            speed = (s.angles[self.axis]-self.last_sample[1])/(now-self.last_sample[0])
        self.last_sample = (now, s.angles[self.axis])
        self.session.observe(now, self.target-s.angles[self.axis], speed, valid)
        s.last_send = now
        if not self.active:
            if self.session.state == "FAILED":
                s.request_arm(False)
                raise RuntimeError(self.session.reason)
            if self.detached:
                s.startup_waiting_owner = True

    def cancel(self, now):
        self.session.cancel(now)

    def interrupted(self, reason):
        if self.active:
            self.session.fail(reason)

    def export(self):
        return dict(self.session.snapshot(), axis=self.axis, origin=self.origin,
                    motor_id=self.service.c["motor_ids"][self.axis] if self.axis is not None else None,
                    saved_path=self.saved_path,
                    parameters_basis="Known user baseline; writes have no documented ACK/readback",
                    motion="+1 deg, center, -1 deg, center")

    def save(self):
        if self.active or self.session.state != "COMPLETE":
            raise RuntimeError("Finish verification before saving the PI profile")
        path = "/root/gimbal_autotune/pi_profile_axis%d.json" % self.axis
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path+".tmp", "w", encoding="utf-8") as output:
            json.dump(self.export(), output, ensure_ascii=False, allow_nan=False)
        os.replace(path+".tmp", path)
        self.saved_path = path
        return path


# ---- src/tuning.py ----
"""Load-aware drive profiles and measured traces; never saves motor parameters."""
from collections import deque


# These retain the existing drive settings. They are NOT measured PID presets.
# Rates/accelerations are yaw, pitch; edit each stage independently after testing.
TUNING_CONFIG = {
    "arm_order": (1, 0),          # Establish pitch holding torque first.
    "startup_auto_home": True,  # User requested fast return to SAVED mechanical zero.
    "startup_hold_s": 0.0,      # No extra commissioning hold delay.
    "startup_timeout_s": 30.0,
    "startup_settle_s": 0.2,
    "record_seconds": 60.0,
    "record_samples": 2400,
    "profiles": {
        "startup": {"rpm": (5, 5), "acceleration": (10, 10),
                    "rate": (24.0, 16.0), "target_acceleration": (80.0, 60.0)},
        "manual": {"rpm": (5, 5), "acceleration": (10, 10),
                   "rate": (24.0, 16.0), "target_acceleration": (80.0, 60.0)},
        "home": {"rpm": (5, 5), "acceleration": (10, 10),
                 "rate": (10.0, 8.0), "target_acceleration": (80.0, 60.0)},
    },
}


def validate_tuning_config(c, tuning):
    if tuple(sorted(tuning["arm_order"])) != (0, 1):
        raise ValueError("arm_order must contain yaw=0 and pitch=1 exactly once")
    if not 0 <= tuning["startup_hold_s"] <= 3:
        raise ValueError("startup_hold_s must be in 0..3 seconds")
    if type(tuning["startup_auto_home"]) is not bool:
        raise ValueError("startup_auto_home must be a boolean")
    if not 1 <= tuning["startup_timeout_s"] <= 60 or not .1 <= tuning["startup_settle_s"] <= 1:
        raise ValueError("Invalid startup timeout or settle time")
    if not 1 <= tuning["record_seconds"] <= 120 or not 100 <= tuning["record_samples"] <= 6000:
        raise ValueError("Invalid bounded recording size")
    for name in ("startup", "manual", "home"):
        profile = tuning["profiles"][name]
        for key in ("rpm", "acceleration", "rate", "target_acceleration"):
            if len(profile[key]) != 2:
                raise ValueError("Profiles require one value per axis")
        for axis in range(2):
            rpm, acceleration = profile["rpm"][axis], profile["acceleration"][axis]
            rate, target_accel = profile["rate"][axis], profile["target_acceleration"][axis]
            if type(rpm) is not int or not 1 <= rpm <= c["motor_rpm"]:
                raise ValueError("Profile RPM must respect the existing motor speed cap")
            if type(acceleration) is not int or not 1 <= acceleration <= 30:
                raise ValueError("Profile drive acceleration must be in 1..30")
            if not math.isfinite(rate) or not 0 < rate <= rpm*6:
                raise ValueError("Profile target rate exceeds motor RPM")
            if not math.isfinite(target_accel) or not 0 < target_accel <= 100:
                raise ValueError("Profile software acceleration must be in 0..100")


def summarize_tuning_trace(samples):
    """Sampling statistics, not a fitted plant model or an autotuning result."""
    if len(samples) < 2:
        return {"samples": len(samples), "axes": []}
    duration = samples[-1]["time_s"]-samples[0]["time_s"]
    gaps = [b["time_s"]-a["time_s"] for a, b in zip(samples, samples[1:])]
    result = {"samples": len(samples), "duration_s": duration,
              "sample_rate_hz": (len(samples)-1)/duration if duration > 0 else None,
              "max_gap_s": max(gaps), "axes": []}
    tail = [row for row in samples if row["time_s"] >= samples[-1]["time_s"]-1.0]
    for axis in range(2):
        active = [r for r in samples if r["armed"] and r["target_deg"] is not None]
        errors = [r["angle_deg"][axis]-r["target_deg"][axis] for r in active]
        # A stationary end segment needs observed coverage, a fixed sent target,
        # torque requested throughout, and no sampling gap hidden in the window.
        holding = (len(tail) >= 3 and tail[-1]["time_s"]-tail[0]["time_s"] >= .6
                   and all(r["armed"] and r["target_deg"] is not None for r in tail)
                   and max(b["time_s"]-a["time_s"] for a, b in zip(tail, tail[1:])) <= .2)
        if holding:
            targets = [r["target_deg"][axis] for r in tail]
            holding = max(targets)-min(targets) < .051
        values = [r["angle_deg"][axis] for r in tail]
        result["axes"].append({
            "axis": ("YAW", "PITCH")[axis],
            "tracking_rmse_deg": math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
            "tracking_peak_error_deg": max(map(abs, errors)) if errors else None,
            "holding_window_s": tail[-1]["time_s"]-tail[0]["time_s"] if holding else None,
            "holding_peak_to_peak_deg": max(values)-min(values) if holding else None,
            "holding_mean_error_deg": sum(v-t for v, t in zip(values, targets))/len(values) if holding else None,
        })
    return result


class TuningMotorService(MechanicalOnlyService):
    """Same UART owner and mechanical guards; PI can change at a stationary hold."""
    def __init__(self, c, transport_factory=None, tuning=None):
        import copy
        self.tuning_config = copy.deepcopy(TUNING_CONFIG if tuning is None else tuning)
        validate_tuning_config(c, self.tuning_config)
        c = dict(c)
        c["max_rate"] = tuple(self.tuning_config["profiles"]["manual"]["rate"])
        c["home_rate"] = tuple(self.tuning_config["profiles"]["home"]["rate"])
        super().__init__(c, transport_factory)
        self.drive_profile = None
        self._drive_values = [None, None]
        self.pi_last_sent = [None, None]  # No documented PI readback command.
        self.pi_status = "电机 PI 未读取；输入框不是实机值"
        self.pi_pending = None
        self.pi_applying = False
        self.history = deque(maxlen=64)
        self.trace_lock = threading.RLock()
        self.trace = None
        self.latest_tuning_sample = None
        self.sample_seq = 0
        self.startup_attempted = False
        self.startup_active = False
        self.startup_waiting_owner = False
        self.startup_deadline = None
        self.startup_status = "PENDING" if self.tuning_config["startup_auto_home"] else "OFF"
        self.pi_autotune = F32CPiAutoTuner(self)
        self.autotune_request = None

    @property
    def autotune_active(self):
        return self.pi_autotune.active or self.autotune_request is not None

    def request_pi_autotune(self, axis, kp, ki):
        if type(axis) is not int or axis not in (0, 1) or any(type(v) is not int or not 0 <= v <= 256 for v in (kp, ki)):
            raise ValueError("Choose an axis and supply the known KP/KI baseline")
        if self.autotune_active or not self.armed or self.home_requested or any(self.joystick):
            raise RuntimeError("Wait for an enabled stationary hold")
        self.pi_autotune.check_start(axis, kp, ki)
        self.pi_autotune.detached = False
        self.autotune_request = (axis, kp, ki)

    def request_home(self):
        if self.autotune_active:
            raise RuntimeError("Cancel PI auto tune before HOME")
        super().request_home()

    def cancel_pi_autotune(self):
        self.autotune_request = None
        self.pi_autotune.cancel(time.monotonic())

    def _manual_phase(self, moving):
        if self.pi_autotune.detached and self.pi_autotune.session.state in ("COMPLETE", "CANCELLED"):
            return "AUTOTUNE_HOLD"
        return super()._manual_phase(moving)

    def snapshot(self):
        result = super().snapshot()
        with self.lock:
            tuning = dict(pi_last_sent=list(self.pi_last_sent), pi_status=self.pi_status,
                          pi_busy=self.pi_pending is not None or self.pi_applying,
                          drive_profile=self.drive_profile, profiles=self.tuning_config["profiles"],
                          startup_hold_s=self.tuning_config["startup_hold_s"],
                          startup_status=self.startup_status)
            tuning["autotune"] = self.pi_autotune.export()
            tuning["autotune"]["active"] = self.autotune_active
        with self.trace_lock:
            tuning.update(latest=self.latest_tuning_sample, recording=bool(self.trace and self.trace["recording"]),
                          count=len(self.trace["samples"]) if self.trace else 0,
                          result=self.trace.get("summary") if self.trace else None,
                          record_reason=self.trace.get("end_reason") if self.trace else None)
        result["tuning"] = tuning
        result["arm_requested"] = self.arm_requested
        result["startup_takeover"] = self.startup_waiting_owner
        result["autotune_takeover"] = self.autotune_active and self.pi_autotune.detached
        return result

    def input(self, x, y):
        if self.autotune_active and (x or y):
            raise RuntimeError("Cancel PI auto tune and wait for rollback before manual movement")
        if self.phase == "ARMING_HOLD":
            x = y = 0.0  # Never queue a joystick target during startup holding.
        super().input(x, y)

    def request_arm(self, armed):
        if armed:
            with self.lock:
                if self.pi_pending is not None or self.pi_applying:
                    raise RuntimeError("Wait for temporary PI writes before ARM")
        else:
            self.autotune_request = None
            self.pi_autotune.interrupted("DISARM interrupted PI auto tune; baseline restore pending")
            self.startup_waiting_owner = False
            with self.lock:
                if self.pi_pending is not None:
                    self.pi_pending = None
                    self.pi_status = "临时 PI 请求已取消，未发送"
        super().request_arm(armed)

    def request_recheck(self):
        with self.lock:
            if self.pi_pending is not None or self.pi_applying:
                raise RuntimeError("Wait for temporary PI writes before RECHECK")
        super().request_recheck()

    def claim_startup_hold(self):
        """Transfer an autonomous zero hold to a real controller, without DISARM."""
        with self.lock:
            if not (self.startup_waiting_owner and self.armed and self.ready
                    and self.reference_valid and not self.fault and self.phase in ("HOME_HOLD", "AUTOTUNE_HOLD")):
                return False
            self.command_time = time.monotonic()
            self.startup_waiting_owner = False
            self.pi_autotune.detached = False
            return True

    def _heartbeat_check(self):
        if self.pi_autotune.active:
            if self.abort_requested or self.stop_event.is_set() or not self.arm_requested:
                raise UserDisarm("PI auto tune stopped")
            return  # Session owns a bounded, local experiment clock.
        if self.startup_active or self.startup_waiting_owner:
            if self.abort_requested or self.stop_event.is_set() or not self.arm_requested:
                raise UserDisarm("Autonomous startup/hold cancelled")
            if self.startup_active and time.monotonic() >= self.startup_deadline:
                raise RuntimeError("Startup HOME timeout")
            return  # Local autonomous transaction/hold, not a synthetic browser heartbeat.
        super()._heartbeat_check()

    def _control_loop(self):
        if not self.startup_attempted:
            self.startup_attempted = True
            if self.tuning_config["startup_auto_home"]:
                if self.abort_requested or self.stop_event.is_set():
                    self.startup_status = "CANCELLED"
                    return super()._control_loop()
                self.startup_active = True
                self.startup_status = "HOMING"
                self.startup_deadline = time.monotonic()+self.tuning_config["startup_timeout_s"]
                self.abort_requested = False
                self.zero_confirmed = True  # Explicit configuration trusts the saved zero.
                self.arm_requested = True
                try:
                    self.manual_target = self.sent_target = self._arm_current()
                    self.armed = True
                    self.last_send = time.monotonic()
                    self.home_requested = True
                    settle = self.c["home_settle_s"]
                    try:
                        self.c["home_settle_s"] = self.tuning_config["startup_settle_s"]
                        self._home_and_clear()
                    finally:
                        self.c["home_settle_s"] = settle
                    self.startup_waiting_owner = True
                    self.startup_status = "HOME_HOLD"
                except UserDisarm:
                    self.startup_status = "CANCELLED"
                    self._disable_now()
                    self.phase = "DISABLED"
                finally:
                    self.startup_active = False
        return super()._control_loop()

    def request_speed_pi(self, axis, kp=None, ki=None):
        if self.autotune_active:
            raise RuntimeError("Wait for PI auto tune to finish before changing gains")
        if type(axis) is not int or axis not in (0, 1):
            raise ValueError("Choose YAW or PITCH")
        if (kp is None and ki is None) or any(v is not None and (type(v) is not int or not 0 <= v <= 256) for v in (kp, ki)):
            raise ValueError("KP/KI must be integers in 0..256")
        with self.lock:
            if (self.fault or not self.ready or not self.reference_valid or self.startup_active
                    or self.home_requested or any(self.joystick)
                    or self.phase not in ("DISABLED", "ARMED", "HOME_HOLD")):
                raise RuntimeError("Apply PI at a stationary hold or while DISABLED")
            if self.pi_pending is not None or self.pi_applying:
                raise RuntimeError("A PI request is already pending")
            self.pi_pending = (axis, kp, ki)
            self.abort_requested = False  # Explicit new request after connect/DISARM.
            self.pi_status = "等待串口线程检查静止并发送临时 PI"

    def _write(self, data):
        if data[2] not in (0x0F, 0x10):
            return super()._write(data)
        time.sleep(.002)
        self._require_reference(fresh=True)
        if self.armed:
            self._heartbeat_check()
        if (not self.pi_applying or self.home_requested or any(self.joystick) or self.abort_requested
                or self.stop_event.is_set()):
            raise UserDisarm("PI write cancelled")
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete PI UART write; parameter state unknown")

    def _process_pi(self):
        with self.lock:
            request, self.pi_pending = self.pi_pending, None
            if request is None:
                return
            self.pi_applying = True
        axis, kp, ki = request
        wrote = False
        try:
            if self.home_requested or any(self.joystick) or self.abort_requested or self.startup_active:
                raise UserDisarm("PI request cancelled")
            # Require fresh stationary mechanical observations of BOTH axes.
            now = time.monotonic()
            with self.lock:
                recent = [s for s in self.history if s[0] >= now-.45]
            if (len(recent) < 3 or recent[-1][0]-recent[0][0] < .3
                    or max(b[0]-a[0] for a, b in zip(recent, recent[1:])) > .2):
                raise ValueError("Keep supported axes stationary for at least 0.3 s, then retry")
            if any(max(s[1][a] for s in recent)-min(s[1][a] for s in recent) > .2 for a in range(2)):
                raise ValueError("Axes are moving; support the load before changing PI")
            mid = self.c["motor_ids"][axis]
            if self._feedback(mid, FEEDBACK_SPEED) != 0:
                raise ValueError("Selected axis speed is not zero")
            # Validate the complete pair before sending either parameter.
            old = self.pi_last_sent[axis] or [None, None]
            updated = list(old)
            self.pi_last_sent[axis] = None
            wrote = True  # Even an incomplete first write can change device state.
            for index, (opcode, value) in enumerate(((0x0F, kp), (0x10, ki))):
                if value is not None:
                    self._write(_frame(mid, opcode, value.to_bytes(2, "big")))
                    updated[index] = value
            self.pi_last_sent[axis] = updated
            self.pi_status = "%s 临时 PI 已发送：KP=%s / KI=%s，无参数 ACK/回读；未保存" % (("YAW", "PITCH")[axis], updated[0], updated[1])
        except (ValueError, UserDisarm) as exc:
            self.pi_status = str(exc) + ("；参数状态未知" if wrote else "；未发送 PI")
            if wrote:
                raise
        except Exception:
            self.pi_status = "PI 通信失败，参数状态未知；未保存"
            raise
        finally:
            with self.lock:
                self.pi_applying = False

    def _profile(self, name):
        profile = self.tuning_config["profiles"][name]
        for axis, mid in enumerate(self.c["motor_ids"]):
            values = (profile["rpm"][axis], profile["acceleration"][axis])
            if self._drive_values[axis] != values:
                self._heartbeat_check()
                self._write(set_speed(mid, values[0]))
                self._heartbeat_check()
                self._write(set_acceleration(mid, values[1]))
                self._drive_values[axis] = values
        self.c["command_accel_deg_s2"] = tuple(profile["target_acceleration"])
        self.drive_profile = name
        return profile

    def _arm_current(self):
        _, angles = self._measure()
        for mid in self.c["motor_ids"]:
            self._write(select_mode(mid, self.c["position_mode"]))
        self._drive_values = [None, None]  # Reapply after possible motor power cycle.
        self._profile("startup")
        for mid, angle in zip(self.c["motor_ids"], self.mechanical):
            self._write(set_single_turn_position(mid, angle))
        self.sent_target = self.manual_target = angles
        for axis in self.tuning_config["arm_order"]:
            self._write(enable(self.c["motor_ids"][axis]))
        self.phase = "ARMING_HOLD"
        deadline = time.monotonic()+self.tuning_config["startup_hold_s"]
        while time.monotonic() < deadline:
            self._heartbeat_check()
            self._measure()
            if self.stop_event.wait(.02):
                raise UserDisarm("Startup hold cancelled")
        self.stream_target = angles
        self.stream_velocity = (0.0, 0.0)
        # Cache the actual preloaded target, never a later sagged feedback pose.
        self.position_units = [int.from_bytes(set_single_turn_position(mid, q/sign)[3:5], "big")
                               for mid, q, sign in zip(self.c["motor_ids"], angles, self.c["motor_signs"])]
        self.last_position_send = time.monotonic()
        self.hold_pending = False
        self.guard.reset()
        return angles

    def _advance(self, goal, rate):
        if self.autotune_request is not None:
            request, self.autotune_request = self.autotune_request, None
            self.pi_autotune.start(*request, time.monotonic())
        if self.pi_autotune.active:
            self.pi_autotune.advance(time.monotonic())
            return
        name = "startup" if self.startup_active else "home" if self.phase in ("RETURNING", "SETTLING", "HOME_HOLD") else "manual"
        profile = self._profile(name)
        return super()._advance(goal, profile["rate"])

    def _measure(self):
        result = super()._measure()
        now = time.monotonic()
        with self.lock:
            self.history.append((now, self.angles))
        self._record_sample(now)
        if self.ready:
            self._process_pi()  # This method is called only by the UART owner.
        return result

    def start_tuning_capture(self, note=""):
        if not isinstance(note, str) or len(note) > 200:
            raise ValueError("Recording note must be at most 200 characters")
        state = super().snapshot()
        if not state["ready"] or state["fault"] or not state["reference_valid"]:
            raise RuntimeError("Valid mechanical feedback required before recording")
        with self.trace_lock:
            if self.trace and self.trace["recording"]:
                raise RuntimeError("Stop the current recording first")
            self.trace = dict(schema=1, note=note, recording=True, samples=[],
                              started_monotonic_s=time.monotonic(), started_unix_s=time.time(),
                              limits=self.c["limits"], motor_ids=self.c["motor_ids"],
                              profiles=self.tuning_config["profiles"],
                              position_mode=self.c["position_mode"],
                              pi_source="last commands sent, NOT device readback",
                              angle_resolution_deg=.1, summary=None, end_reason=None)

    def stop_tuning_capture(self, reason="manual"):
        with self.trace_lock:
            if self.trace and self.trace["recording"]:
                self.trace["recording"] = False
                self.trace["end_reason"] = reason
                self.trace["summary"] = summarize_tuning_trace(self.trace["samples"])

    def _record_sample(self, now):
        self.sample_seq += 1
        row = dict(seq=self.sample_seq, time_s=now,
                   feedback_time_s=[s[0] if s else None for s in self.mechanical_samples],
                   angle_deg=self.angles, target_deg=self.sent_target,
                   armed=bool(self.armed or (self.arm_requested and self.phase == "ARMING_HOLD")),
                   phase=self.phase, profile=self.drive_profile,
                   input=self.joystick, pi_last_sent=[list(p) if p else None for p in self.pi_last_sent])
        with self.trace_lock:
            self.latest_tuning_sample = row
            if self.trace and self.trace["recording"]:
                self.trace["samples"].append(row)
                if (now-self.trace["started_monotonic_s"] >= self.tuning_config["record_seconds"]
                        or len(self.trace["samples"]) >= self.tuning_config["record_samples"]):
                    self.stop_tuning_capture("recording limit")

    def export_tuning(self):
        with self.trace_lock:
            if not self.trace:
                if hasattr(self.pi_autotune.session, "baseline"):
                    return dict(pi_autotune=self.pi_autotune.export())
                raise ValueError("Start a recording or PI auto tune first")
            result = dict(self.trace, samples=list(self.trace["samples"]))
        result["summary"] = summarize_tuning_trace(result["samples"])
        result["pi_autotune"] = self.pi_autotune.export()
        return result

    def _record_fault(self, exc):
        self.autotune_request = None
        self.pi_autotune.interrupted(str(exc))
        self.startup_attempted = True
        self.startup_active = self.startup_waiting_owner = False
        self.startup_status = "FAULT"
        self.stop_tuning_capture("fault: " + str(exc))
        super()._record_fault(exc)

    def close(self):
        self.stop_tuning_capture("application closed")
        super().close()


# ---- src/web.py ----
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

        def reply(self, status, body, content_type="application/json; charset=utf-8", filename=None):
            data = body.encode("utf-8") if isinstance(body, str) else json.dumps(body, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("X-Content-Type-Options", "nosniff")
            if filename:
                self.send_header("Content-Disposition", 'attachment; filename="%s"' % filename)
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError, socket.timeout):
                pass

        def do_GET(self):
            if self.path == "/":
                self.reply(200, WEB_HTML, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self.reply(200, control.state())
            elif self.path == "/api/tuning/export":
                try:
                    self.reply(200, control.motor.export_tuning(), filename="f32c-tuning.json")
                except ValueError as exc:
                    self.reply(409, {"error": str(exc)})
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


WEB_HTML = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>F32C 电脑控制</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#0b1420;color:#e9f1f8;font:16px system-ui,sans-serif}
main{max-width:900px;margin:auto;padding:28px}h1{font-size:26px;margin-bottom:4px}.muted{color:#9cacbd}
.panel{border:1px solid #2d4259;border-radius:16px;background:#131f2e;padding:22px;margin-top:18px}
.row{display:flex;gap:26px;align-items:center;flex-wrap:wrap}button{font:inherit;cursor:pointer;border:0;border-radius:9px;padding:13px 20px;background:#285467;color:white;margin:5px 6px 5px 0}
button:disabled{opacity:.4;cursor:default}.stop{background:#b23341}.big{font-size:32px;font-variant-numeric:tabular-nums}
#pad{width:260px;height:260px;position:relative;border:3px solid #526e88;border-radius:50%;touch-action:none;user-select:none;background:linear-gradient(transparent 49.7%,#304052 50%,transparent 50.3%),linear-gradient(90deg,transparent 49.7%,#304052 50%,transparent 50.3%)}
#knob{position:absolute;width:48px;height:48px;border-radius:50%;background:#28c8bf;left:103px;top:103px;pointer-events:none}
#notice{color:#ffd18a;white-space:pre-wrap;overflow-wrap:anywhere}kbd{background:#2b4056;padding:2px 7px;border-radius:4px}
input,select{font:inherit;color:#e9f1f8;background:#0b1420;border:1px solid #526e88;border-radius:6px;padding:8px;margin:4px;max-width:100%}input[type=number]{width:95px}#note{width:100%}.tune-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}#trace{width:100%;height:210px;background:#0b1420;border-radius:8px;margin-top:10px}#piStatus,#recordStatus,#metrics{white-space:pre-wrap;overflow-wrap:anywhere}.download{color:#5bd8d0;display:inline-block;padding:12px}
</style><main><h1>F32C · 电脑控制</h1><div class="muted">MECH ONLY · 机械角度模式</div>
<p id="mountInfo" class="muted">倒装：左右反向 · PITCH −100°～+30° · 平滑位置控制</p>
<p id="speedInfo" class="muted">目标速度上限：YAW 24°/s · PITCH 16°/s</p>
<div class="panel"><div class="row"><div><span id="link">未连接控制</span> · <b id="phase">--</b></div><button id="connect">连接控制</button><button id="recheck">重新检查状态</button><button id="stop" class="stop">DISARM / 失能</button></div>
<div id="notice">加载状态…</div></div>
<div class="panel row"><div><div class="muted">↑ PITCH · YAW →</div><div id="pad" tabindex="0" aria-label="按住拖动摇杆"><div id="knob"></div></div></div><div>
<div class="muted">YAW / 水平角度</div><div class="big" id="yaw">--°</div><br>
<div class="muted">PITCH / 俯仰角度</div><div class="big" id="pitch">--°</div><p id="feedback" class="muted">--</p>
<p id="mechanical" class="muted">机械反馈：--</p>
<p id="zeroState">机械零点未确认</p><button id="zero">ZERO OK</button><button id="arm">手动使能 / ARM</button><button id="home">HOME / 回原点</button>
<p id="armReason" class="muted"></p></div></div>
<p>按住摇杆或 <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> 转动；松开保持位置。<kbd>空格</kbd> 失能。</p>
<p class="muted">单次连续移动最多 8 秒，达到后松开再按。切到后台或失去焦点会请求失能。HOME 完成后保持使能，累计角度不参与控制。调整后可在失能状态点“重新检查状态”。</p>
<section class="panel" id="tuningPanel">
<h2 style="margin-top:0;font-size:21px">负载与 PI 调试</h2>
<p class="muted">启动后自动快速回已保存的机械零点并保持使能。启动、手动、HOME 的两轴驱动配置独立。失能时相机会下垂，请支撑俯仰负载。</p>
<p id="profileInfo" class="muted">启动 / 手动 / HOME 使用独立的两轴驱动配置。</p>
<div class="tune-row"><label>轴 <select id="tuneAxis"><option value="0">X / YAW</option><option value="1">Y / PITCH</option></select></label><label>Kp <input id="kp" type="number" min="0" max="256" step="1" placeholder="留空不改"></label><label>Ki <input id="ki" type="number" min="0" max="256" step="1" placeholder="留空不改"></label><button id="applyPi" disabled>原地保持时临时发送 PI</button></div>
<p id="piStatus" class="muted">PI 未读取，输入框用于填写已知基线。</p>
<p>PI 自动整定会围绕当前位置做 +1°、返回、−1°、返回的单轴测试。开始前填写已知 Kp 和 Ki 基线。</p>
<button id="piAutoStart">开始 PI 自动整定</button><button id="piAutoCancel">取消并恢复基线</button><button id="piAutoSave">保存验证配置</button><p id="piAutoStatus"></p>
<label class="muted" for="note">本轮备注（负载、姿态、参数来源）</label><input id="note" maxlength="200" placeholder="例如：带相机，PITCH -20°，原参数未知，原地保持">
<div class="tune-row"><button id="record" disabled>开始记录</button><a class="download" href="/api/tuning/export" download="f32c-tuning.json">导出本轮 JSON</a></div>
<div id="recordStatus" class="muted">先记录原参数下的保持状态，记录按钮不会自动试转。</div>
<canvas id="trace" width="900" height="240" aria-label="所选轴目标角度与实测角度曲线"></canvas>
<div class="muted">青色：实测机械角度 · 黄色：已发送目标 · 图为网页采样，导出保留串口线程的原始采样时间。</div>
<p id="metrics" class="muted"></p><p class="muted">最多记录 60 秒。角度分辨率 0.1°，采样曲线不能排除高频振动。PI 可在原地保持使能时修改，无需每轮失能；留空的参数不发送。</p>
</section></main>
<script>
'use strict';
const $=id=>document.getElementById(id), keys=new Set();
let token=null,seq=0,vector=[0,0],pointer=null,pending='',pendingTune={},busy=false,lastState=null;
const traceRows=[];
function release(){vector=[0,0];keys.clear();pointer=null;draw();}
function draw(){ $('knob').style.transform=`translate(${vector[0]*96}px,${-vector[1]*96}px)`; }
function render(s){lastState=s;$('phase').textContent=s.phase;$('link').textContent=token?'控制已连接':s.connected?'另一网页正在控制':'未连接控制';
 if(s.limits&&s.joystick_signs)$('mountInfo').textContent=`左右${s.joystick_signs[0]<0?'反向':'正常'} · PITCH ${s.limits[1][0]}°～${s.limits[1][1]}° · 平滑位置控制`;
 if(s.max_rate&&s.home_rate)$('speedInfo').textContent=`手动目标上限 YAW ${s.max_rate[0]}°/s · PITCH ${s.max_rate[1]}°/s；HOME ${s.home_rate[0]} / ${s.home_rate[1]}°/s`;
 const valid=s.reference_valid&&s.control_feedback_seen!==false;
 const degree=v=>Number.isFinite(v)?v.toFixed(1)+'°':'--';
 $('yaw').textContent=valid?degree(s.angles[0]):'--';$('pitch').textContent=valid?degree(s.angles[1]):'--';
 const age=Number.isFinite(s.age)?s.age.toFixed(2)+'s':'尚无有效控制反馈';
 $('feedback').textContent=`控制反馈：${age} · REF ${s.reference_valid?'VALID':'INVALID'}`;
 const ma=s.mechanical_angles||[], da=s.mechanical_feedback_age||[];
 const diag=i=>`${degree(ma[i])}（${Number.isFinite(da[i])?da[i].toFixed(2)+'s 前':'尚未收到'}）`;
 $('mechanical').textContent=`诊断机械反馈 YAW ${diag(0)} / PITCH ${diag(1)}`;
 $('zeroState').textContent=s.zero_confirmed?(['HOMING','HOME_HOLD'].includes(s.tuning?.startup_status)?'采用已保存的机械零点':'机械零点已确认'):'机械零点未确认';
 const checking=['RECHECK_PENDING','RECHECKING'].includes(s.phase);
 $('notice').textContent=checking?'正在重新检查角度、限位与静止状态，请保持轴静止':s.fault|| (s.hold_blocked?'已达到连续移动上限，请松开后再按':s.phase==='HOME_HOLD'?'已回机械原点，保持使能；可继续摇杆控制，DISARM 请求失能':s.recheck_status==='PASSED'&&!token?'重新检查通过，当前失能；连接控制 → ZERO OK → 手动使能':s.notice);
 const usable=!!token&&s.ready&&!s.fault;
 $('connect').disabled=!!token||s.connected||!s.ready||!!s.fault||(s.armed&&!s.startup_takeover&&!s.autotune_takeover);
 $('recheck').disabled=!s.can_recheck||(s.connected&&!token);
 $('zero').disabled=!usable||s.armed||s.arm_requested;$('arm').disabled=!usable||s.armed||s.arm_requested||!s.zero_confirmed;
 $('home').disabled=!usable||!s.armed||s.home_requested;
 $('armReason').textContent=s.phase==='ARMING_HOLD'?'启动保持中：已请求使能并保持当前姿态，请保持摇杆松开':checking?'重新检查不会自动使能，也不会改机械零点':s.fault?'使能已阻止：'+s.fault+(s.fault.includes('after disable')?'。失能后仍检测到运动／偏离零位，请支撑并检查负载，静止后点“重新检查状态”。':'。排查并移回允许范围后，点“重新检查状态”。'):
  !s.ready?'等待电机就绪':!token?'先连接控制':s.armed?'已使能；DISARM / 空格请求失能':
  !s.zero_confirmed?'确认已有机械零点正确，再点 ZERO OK':'当前失能；点击“手动使能 / ARM”后才会使能';
 renderTuning(s);
}
function renderTuning(s){const t=s.tuning;if(!t)return;
 const usable=!!token&&s.ready&&!s.fault;
 const auto=t.autotune||{active:false,state:'IDLE'};
 $('piAutoStart').disabled=!usable||!s.armed||auto.active||s.home_requested;
 $('piAutoCancel').disabled=!usable||!auto.active;
 $('piAutoSave').disabled=!usable||auto.active||auto.state!=='COMPLETE';
 $('piAutoStatus').textContent=auto.state+(auto.reason?' · '+auto.reason:'')+(auto.restore_pending?' · 基线恢复待确认':'')+(auto.saved_path?' · 已保存 '+auto.saved_path:'');
 if(auto.active)$('home').disabled=true;
 $('applyPi').disabled=!usable||auto.active||!['DISABLED','ARMED','HOME_HOLD'].includes(s.phase)||s.home_requested||t.pi_busy;
 $('record').disabled=!usable;$('record').textContent=t.recording?'停止记录':'开始记录';
 const pair=p=>p?`Kp ${p[0]??'未知'} / Ki ${p[1]??'未知'}`:'未知';
 $('piStatus').textContent=`${s.tuning_error||t.pi_status}\n最近发送：YAW ${pair(t.pi_last_sent[0])}；PITCH ${pair(t.pi_last_sent[1])}（无实机回读）`;
 const p=t.profiles||{}, describe=n=>p[n]?`${n}: RPM ${p[n].rpm.join('/')} · 目标速度 ${p[n].rate.join('/')}°/s`:n;
 $('profileInfo').textContent=`启动 ${t.startup_status} · 当前配置 ${t.drive_profile||'尚未使能'}\n${['startup','manual','home'].map(describe).join('；')}`;
 $('recordStatus').textContent=`${t.recording?'记录中':'记录已停止'} · ${t.count} 个串口样本${t.record_reason?' · '+t.record_reason:''}`;
 const f=v=>Number.isFinite(v)?v.toFixed(3):'--';
 $('metrics').textContent=t.result&&t.result.axes? t.result.axes.map(a=>`${a.axis}：跟踪 RMSE ${f(a.tracking_rmse_deg)}°；末段固定目标峰峰值 ${f(a.holding_peak_to_peak_deg)}°；平均误差 ${f(a.holding_mean_error_deg)}°`).join('\n'):'';
 if(t.latest&&s.reference_valid&&Number.isFinite(s.age)&&s.age<.5){const r=t.latest,last=traceRows[traceRows.length-1];
  if(last&&r.seq<last.seq)traceRows.length=0;
  if(!last||last.seq!==r.seq){traceRows.push(r);if(traceRows.length>260)traceRows.shift();}}
 drawTrace();
}
function drawTrace(){const c=$('trace'),g=c.getContext('2d');if(!g)return;const w=c.width,h=c.height,axis=Number($('tuneAxis').value);g.clearRect(0,0,w,h);if(!traceRows.length)return;
 const end=traceRows[traceRows.length-1].time_s,rows=traceRows.filter(r=>r.time_s>=end-12),values=rows.flatMap(r=>[r.angle_deg[axis],r.target_deg?.[axis]]).filter(Number.isFinite);
 if(!values.length)return;const lo=Math.min(...values)-.5,hi=Math.max(lo+2,...values.map(v=>v+.5)),x=t=>55+(t-(end-12))/12*(w-75),y=v=>h-25-(v-lo)/(hi-lo)*(h-45);
 g.font='15px system-ui';g.fillStyle='#9cacbd';g.strokeStyle='#2d4259';for(let i=0;i<4;i++){const v=lo+(hi-lo)*i/3;g.beginPath();g.moveTo(55,y(v));g.lineTo(w-20,y(v));g.stroke();g.fillText(v.toFixed(1)+'°',3,y(v)+5);}g.fillText('-12s',55,h-5);g.fillText('现在',w-55,h-5);
 for(const [key,color] of [['angle_deg','#28c8bf'],['target_deg','#ffd18a']]){g.strokeStyle=color;g.lineWidth=2;g.beginPath();let previous=null;for(const r of rows){const v=r[key]?.[axis];if(!Number.isFinite(v)){previous=null;continue;}if(previous===null||r.time_s-previous>.3)g.moveTo(x(r.time_s),y(v));else g.lineTo(x(r.time_s),y(v));previous=r.time_s;}g.stroke();}}
function queueTuning(action,fields={}){release();pendingTune=fields;pending=action;}
$('applyPi').onclick=()=>{const raw=[$('kp').value.trim(),$('ki').value.trim()],nums=raw.map(v=>v===''?null:Number(v));if(nums.every(v=>v===null)||nums.some(v=>v!==null&&(!Number.isInteger(v)||v<0||v>256))){$('piStatus').textContent='填写至少一个 0～256 的整数；留空不改，没有读取到当前参数。';return;}queueTuning('pi',{axis:Number($('tuneAxis').value),kp:nums[0],ki:nums[1]});};
$('record').onclick=()=>queueTuning(lastState?.tuning?.recording?'record_stop':'record_start',{note:$('note').value});
$('piAutoStart').onclick=()=>{
 const raw=[$('kp').value.trim(),$('ki').value.trim()];
 const values=raw.map(Number);
 if(raw.some(v=>v==='')||values.some(v=>!Number.isInteger(v)||v<0||v>256)){
  $('piAutoStatus').textContent='先填写已知的 Kp 和 Ki 基线，范围为 0～256 整数';
  return;
 }
 queueTuning('pi_autotune',{axis:Number($('tuneAxis').value),kp:values[0],ki:values[1]});
};
$('piAutoCancel').onclick=()=>queueTuning('pi_autotune_cancel');
$('piAutoSave').onclick=()=>queueTuning('pi_autotune_save');
$('tuneAxis').onchange=drawTrace;
async function post(path,body){const ctl=new AbortController(),timer=setTimeout(()=>ctl.abort(),450);
 try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:ctl.signal,cache:'no-store'});
 const j=await r.json();if(!r.ok)throw Error(j.error||r.status);return j;}finally{clearTimeout(timer);}}
function stop(){const old=token;token=null;pending='';pendingTune={};release();navigator.sendBeacon('/api/stop',new Blob([JSON.stringify({token:old})],{type:'application/json'}));
 $('link').textContent='已请求失能';}
$('connect').onclick=async()=>{release();try{const j=await post('/api/connect',{});token=j.token;seq=0;render(j.state);}catch(e){$('notice').textContent=e.message;}};
$('recheck').onclick=async()=>{const old=token;release();pending='';try{const s=await post('/api/recheck',{token:old});token=null;seq=0;render(s);}catch(e){$('notice').textContent=e.message;}};
$('stop').onclick=stop;
for(const action of ['zero','arm','home'])$(action).onclick=()=>{release();pending=action;};
const pad=$('pad');
function point(e){const r=pad.getBoundingClientRect();let x=(e.clientX-r.left-r.width/2)/(r.width*.4),y=(r.top+r.height/2-e.clientY)/(r.height*.4);let d=Math.hypot(x,y);
 if(d<.15)vector=[0,0];else{const a=Math.min(1,(d-.15)/.85);vector=[x/d*a,y/d*a];}draw();}
pad.onpointerdown=e=>{if(!token||pointer!==null||lastState?.phase==='ARMING_HOLD'||lastState?.tuning?.autotune?.active)return;e.preventDefault();keys.clear();pointer=e.pointerId;pad.setPointerCapture(pointer);point(e);};
pad.onpointermove=e=>{if(e.pointerId===pointer)point(e);};
pad.onpointerup=pad.onpointercancel=pad.onlostpointercapture=e=>{if(e.pointerId===pointer)release();};
function keyVector(){let x=Number(keys.has('ArrowRight'))-Number(keys.has('ArrowLeft')),y=Number(keys.has('ArrowUp'))-Number(keys.has('ArrowDown')),n=Math.max(1,Math.hypot(x,y));vector=[x/n,y/n];draw();}
window.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();stop();return;}if(['INPUT','SELECT','TEXTAREA'].includes(e.target?.tagName))return;if(!e.key.startsWith('Arrow'))return;e.preventDefault();if(token&&pointer===null&&lastState?.phase!=='ARMING_HOLD'&&!lastState?.tuning?.autotune?.active){keys.add(e.key);keyVector();}});
window.addEventListener('keyup',e=>{if(keys.delete(e.key))keyVector();});
function stopIfOwned(){
 if(!token)return;
 if(lastState?.tuning?.autotune?.active){
  const old=token;token=null;pending='';release();
  navigator.sendBeacon('/api/tuning/background',new Blob([JSON.stringify({token:old})],{type:'application/json'}));
 }else stop();
}
window.addEventListener('blur',stopIfOwned);window.addEventListener('pagehide',stopIfOwned);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopIfOwned();});
async function tick(){if(busy)return;busy=true;const current=token;try{if(current){const action=pending,fields=pendingTune;pending='';pendingTune={};const v=action?[0,0]:vector.slice();const s=await post('/api/control',{...fields,token:current,seq:++seq,x:v[0],y:v[1],action});if(token===current)render(s);}
 else{const r=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(450)});render(await r.json());}}
 catch(e){if(current&&token===current)stop();$('notice').textContent='连接失败 / '+e.message+'；请重新连接';}finally{busy=false;setTimeout(tick,80);}}
tick();
</script></html>'''


def web_main():
    from maix import app, sys as maix_sys
    if str(maix_sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART3 mapping requires MaixCAM2")
    validate_config(CONFIG)
    motor = TuningMotorService(dict(CONFIG))
    control = BrowserControl(motor, lease_s=WEB_CONFIG["lease_s"])
    server = None
    thread = None
    try:
        # Bind before UART startup: a port conflict must not leave hardware running.
        server = make_http_server(control, (WEB_CONFIG["host"], WEB_CONFIG["port"]))
        print("F32C WEB | MECH ONLY | AUTO STARTUP HOME=%s | saved mechanical zero" % TUNING_CONFIG["startup_auto_home"], flush=True)
        motor.start()
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        thread.start()
        print("F32C WEB | MECH ONLY | UART3 B2/B3 115200 | startup/manual/home profiles", flush=True)
        print("MOTOR_MOUNT yaw_input=%d pitch_limits=%s mode=%d position_period=%.3fs" % (
            CONFIG["joystick_signs"][0], CONFIG["limits"][1], CONFIG["position_mode"],
            CONFIG["position_period_s"]), flush=True)
        addresses = []
        try:
            import psutil
            addresses = [a.address for items in psutil.net_if_addrs().values() for a in items
                         if a.family == socket.AF_INET and not a.address.startswith("127.")]
        except Exception:
            pass
        for ip in sorted(set(addresses)):
            print("COMPUTER OPEN: http://%s:%d" % (ip, WEB_CONFIG["port"]), flush=True)
        print("同一网络电脑打开 http://相机IP:%d；使用 MaixVision 连接的相机 IP。" % WEB_CONFIG["port"], flush=True)
        last_log = 0
        while not app.need_exit():
            control.poll()  # Never synthesizes browser heartbeat.
            if time.monotonic()-last_log >= 1:
                print("MOTOR_WEB " + json.dumps(control.state()), flush=True)
                last_log = time.monotonic()
            time.sleep(0.02)
    finally:
        motor.request_arm(False)
        motor.close()
        if server is not None:
            if thread is not None:
                server.shutdown()
                thread.join(1)
            server.server_close()


if __name__ == "__main__":
    web_main()
