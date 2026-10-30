# Generated standalone MaixCAM2 motor demo; CONFIG is editable below.


# ---- src\config.py ----
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


# ---- vendor\motor_base.py ----
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


# ---- src\service.py ----
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
            if self.home_requested or self.phase not in ("ARMED", "MANUAL", "HOME_HOLD"):
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
        while not self.stop_event.wait(0.02):
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


# ---- src\mechanical_only.py ----
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
            for _ in range(3):
                if self.abort_requested or self.stop_event.is_set():
                    raise UserDisarm("Recheck cancelled")
                # Bypass the old reference gate only for fresh read-only checks.
                # Existing fault remains latched until the complete check passes.
                self._read_measurement()
                for mid in self.c["motor_ids"]:
                    speed = self._feedback(mid, FEEDBACK_SPEED)
                    if speed != 0:
                        raise RuntimeError("Recheck: ID=%d still moving, speed=%g RPM" % (mid, speed))
                samples.append(self.mechanical)
                if self.stop_event.wait(0.08):
                    raise UserDisarm("Recheck stopped")
            if any(max(s[a] for s in samples)-min(s[a] for s in samples) > self.c["reference_stationary_deg"] for a in range(2)):
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
                    self.position_units[axis] = units
                    sent[axis] = round(signed_mechanical(units/10.0)*sign, 7)
            self.sent_target = tuple(sent)
            self.last_position_send = now
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


# ---- src\ui.py ----
"""640x480 logical touch UI; callbacks enqueue requests, never touch UART."""
import json

BUTTONS = {
    "arm": (16, 390, 120, 50), "stop": (146, 390, 120, 50),
    "zero": (276, 390, 140, 50), "home": (426, 390, 198, 50),
    "recheck": (352, 335, 270, 44),
}


class JoystickUI:
    def __init__(self, c):
        self.c = c
        self.pressed = False
        self.captured = False
        self.started = None
        self.vector = (0.0, 0.0)
        self.blocked = False

    def touch(self, x, y, pressed, now):
        action = None
        new_press = pressed and not self.pressed
        if not pressed:
            self.captured = self.blocked = False
            self.started = None
            self.vector = (0.0, 0.0)
        else:
            # DISARM also works if a captured finger slides onto its button.
            bx, by, bw, bh = BUTTONS["stop"]
            if bx <= x < bx+bw and by <= y < by+bh:
                action = "stop"
                self.captured = False
                self.vector = (0.0, 0.0)
            elif new_press:
                for name, (bx, by, bw, bh) in BUTTONS.items():
                    if bx <= x < bx+bw and by <= y < by+bh:
                        action = name
                        break
                if action is None and math.hypot(x-175, y-235) <= 112:
                    self.captured = True
                    self.started = now
            if self.captured:
                if now - self.started >= self.c["max_touch_hold_s"]:
                    self.blocked = True
                dx, dy = (x-175)/100.0, (235-y)/100.0
                length = math.hypot(dx, dy)
                if self.blocked or length <= self.c["joystick_deadzone"]:
                    self.vector = (0.0, 0.0)
                else:
                    strength = min(1.0, (length-self.c["joystick_deadzone"])/(1-self.c["joystick_deadzone"]))
                    self.vector = (dx/length*strength, dy/length*strength)
        self.pressed = bool(pressed)
        return self.vector, action


def draw_screen(disp, snapshot, ui, notice=""):
    from maix import image
    rgb = image.Color.from_rgb
    img = image.Image(640, 480, image.Format.FMT_RGB888, bg=rgb(10, 16, 26))
    white, muted, cyan, amber = rgb(235, 242, 250), rgb(145, 160, 180), rgb(40, 215, 205), rgb(255, 195, 70)
    img.draw_string(16, 12, "F32C JOYSTICK | MECH ONLY", white, scale=1.5)
    state = snapshot["phase"]
    img.draw_string(16, 48, state, amber if snapshot["fault"] else cyan, scale=1.3)
    img.draw_string(16, 80, "Release: hold pose | DISARM: torque off", muted)
    img.draw_circle(175, 235, 106, rgb(70, 95, 120), 3)
    img.draw_circle(175, 235, 50, rgb(35, 55, 75), 1)
    img.draw_line(65, 235, 285, 235, rgb(40, 60, 80), 1)
    img.draw_line(175, 125, 175, 345, rgb(40, 60, 80), 1)
    dx, dy = ui.vector
    img.draw_circle(int(175+dx*86), int(235-dy*86), 24, cyan, -1)
    img.draw_string(140, 108, "+PITCH", muted)
    img.draw_string(286, 226, "-YAW" if ui.c["joystick_signs"][0] < 0 else "+YAW", muted)
    img.draw_string(16, 352, "LIFT FINGER TO RESTART" if ui.blocked else "Hold joystick; max 8s per touch", amber if ui.blocked else muted)
    x, y = snapshot["angles"]
    valid = snapshot["reference_valid"] and snapshot.get("control_feedback_seen", True)
    img.draw_string(352, 120, "YAW   %7.2f deg" % x if valid else "YAW   --", white, scale=1.2)
    img.draw_string(352, 150, "PITCH %7.2f deg" % y if valid else "PITCH --", white, scale=1.2)
    raw = snapshot["raw_feedback"]
    mech = snapshot["mechanical_angles"]
    for i, label in enumerate(("TOTAL UNUSED", "MECH")):
        values = raw if i == 0 else mech
        text = " / ".join("--" if v is None else "%.1f" % v for v in values)
        img.draw_string(352, 194+i*26, label + " " + text, muted)
    img.draw_string(352, 252, "REF " + ("VALID" if snapshot["reference_valid"] else "INVALID"), cyan if snapshot["reference_valid"] else amber)
    img.draw_string(352, 278, "ZERO " + ("USER CONFIRMED" if snapshot["zero_confirmed"] else "NOT CONFIRMED"), muted)
    age = snapshot["age"]
    img.draw_string(352, 304, "FEEDBACK AGE --" if age is None else "FEEDBACK AGE %.2fs" % age, muted)
    labels = {"arm": "ARM", "stop": "DISARM", "zero": "ZERO OK", "home": "HOME", "recheck": "RECHECK STATUS"}
    for name, (bx, by, bw, bh) in BUTTONS.items():
        color = rgb(145, 40, 45) if name == "stop" else rgb(35, 65, 85)
        img.draw_rect(bx, by, bw, bh, color, -1)
        img.draw_string(bx+12, by+15, labels[name], white, scale=1.2)
    message = snapshot["fault"] or notice or "HOME holds zero ARMED; RECHECK while DISABLED."
    img.draw_string(16, 451, str(message)[:78], amber)
    disp.show(img)


def main():
    from maix import app, display, touchscreen, sys
    if str(sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART3 pin mapping is for MaixCAM2")
    validate_config(CONFIG)
    motor = MechanicalOnlyService(dict(CONFIG))
    disp = ts = None
    ui = JoystickUI(CONFIG)
    try:
        disp = display.Display()
        ts = touchscreen.TouchScreen()
        print("MOTOR_JOYSTICK_START UART3 B2->motor RX, B3<-motor TX; 115200 8N1; common GND")
        print("START DISABLED. ZERO OK confirms existing physical calibration; does not change it.")
        print("MECHANICAL ONLY: mode=%d position=0x03 feedback=2; yaw input reversed; pitch -100..30" % CONFIG["position_mode"])
        motor.start()
        last_draw = last_log = 0.0
        notice = "Verify saved physical zero, then ZERO OK -> ARM"
        while not app.need_exit():
            now = time.monotonic()
            x, y, pressed = ts.read()
            x, y = x*640/disp.width(), y*480/disp.height()
            vector, action = ui.touch(x, y, pressed, now)
            motor.input(*vector)
            try:
                if action == "stop":
                    motor.request_arm(False)
                    notice = "DISARM requested; torque-off has no feedback ACK"
                elif action == "zero":
                    motor.confirm_reference()
                    notice = "User confirmed saved physical zero; no 0x0A/0x08 sent"
                elif action == "arm":
                    motor.request_arm(True)
                    notice = "ARM requested at current measured position"
                elif action == "home":
                    motor.request_home()
                    notice = "Returning using mechanical feedback; cumulative angle untouched"
                elif action == "recheck":
                    motor.request_recheck()
                    notice = "Checking while disabled; then ZERO OK and manual ARM"
            except Exception as exc:
                notice = str(exc)
                print("MOTOR_UI_REQUEST_REJECTED", notice)
            snapshot = motor.snapshot()
            if now-last_draw >= 0.08:
                draw_screen(disp, snapshot, ui, notice)
                last_draw = now
            if now-last_log >= 1:
                print("MOTOR_JOYSTICK " + json.dumps(snapshot), flush=True)
                last_log = now
            time.sleep(0.01)
    finally:
        motor.close()
        for resource in (ts, disp):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    print("MOTOR_UI_CLOSE_FAILED", str(exc))
        print("MOTOR_JOYSTICK_STOP (disable requests sent; not electrical power cutoff)")


if __name__ == "__main__":
    main()
