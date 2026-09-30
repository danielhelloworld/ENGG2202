"""WHEELTEC F32C TTL protocol and two-axis gimbal driver.

The implementation follows the manuals shipped with the F32C gimbal:
115200-8-N-1, 0x7A head, 0x7B tail, big-endian payload and XOR BCC.
Angles in protocol frames are signed 0.1 degree units.
"""

from __future__ import annotations

import math
import struct
import time


FRAME_HEAD = 0x7A
FRAME_TAIL = 0x7B

MODE_SPEED = 0
MODE_MULTI_T = 1
MODE_SINGLE_T = 2
MODE_MULTI_DIRECT = 3
MODE_SINGLE_DIRECT = 4

PID_SPEED_KP = 0x0F
PID_SPEED_KI = 0x10
PID_POSITION_KP = 0x11
PID_POSITION_KI = 0x12

FEEDBACK_SPEED = 0
FEEDBACK_TOTAL_ANGLE = 1
FEEDBACK_MECHANICAL_ANGLE = 2
FEEDBACK_ACCELERATION = 3
FEEDBACK_BUS_VOLTAGE = 4

DEFAULT_Y_MIN_DEG = -100.0
DEFAULT_Y_MAX_DEG = 100.0


class F32CError(Exception):
    """Base exception for F32C communication errors."""


class F32CTimeout(F32CError):
    """Raised when a complete feedback frame is not received in time."""


class F32CProtocolError(F32CError):
    """Raised when an F32C feedback frame is malformed."""


class F32CLimitError(F32CError):
    """Raised before transmission when a target violates a hard limit."""


def bcc(data: bytes | bytearray) -> int:
    """Return the XOR/BCC of all bytes in *data*."""
    value = 0
    for byte in data:
        value ^= byte
    return value


def _validate_motor_id(motor_id: int) -> None:
    if not 1 <= motor_id <= 255:
        raise ValueError("motor_id must be in 1..255")


def _frame(motor_id: int, function: int, payload: bytes = b"") -> bytes:
    _validate_motor_id(motor_id)
    body = bytes((FRAME_HEAD, motor_id, function)) + payload
    return body + bytes((bcc(body), FRAME_TAIL))


def enable_frame(motor_id: int) -> bytes:
    return _frame(motor_id, 0x06)


def disable_frame(motor_id: int) -> bytes:
    return _frame(motor_id, 0x05)


def mode_frame(motor_id: int, mode: int) -> bytes:
    if mode not in range(5):
        raise ValueError("mode must be in 0..4")
    return _frame(motor_id, 0x00, struct.pack(">H", mode))


def speed_frame(motor_id: int, rpm: int) -> bytes:
    if not -1000 <= rpm <= 1000:
        raise ValueError("speed must be in -1000..1000 RPM")
    return _frame(motor_id, 0x01, struct.pack(">h", rpm))


def acceleration_frame(motor_id: int, acceleration: int) -> bytes:
    """Build the T-profile acceleration command (function 0x07)."""
    if not 0 <= acceleration <= 0xFFFF:
        raise ValueError("acceleration must be in 0..65535")
    return _frame(motor_id, 0x07, struct.pack(">H", acceleration))


def pid_frame(motor_id: int, parameter: int, value: int) -> bytes:
    """Build one of the four documented PID parameter commands."""
    if parameter not in (
        PID_SPEED_KP,
        PID_SPEED_KI,
        PID_POSITION_KP,
        PID_POSITION_KI,
    ):
        raise ValueError("unknown PID parameter function code")
    if not 0 <= value <= 256:
        raise ValueError("PID value must be in 0..256")
    return _frame(motor_id, parameter, struct.pack(">H", value))


def save_parameters_frame(motor_id: int) -> bytes:
    """Build the documented non-volatile parameter-save command."""
    return _frame(motor_id, 0x08)


def multi_turn_position_frame(motor_id: int, angle_deg: float) -> bytes:
    deci_deg = int(round(angle_deg * 10.0))
    if not -(2**31) <= deci_deg < 2**31:
        raise ValueError("angle is outside the signed 32-bit protocol range")
    return _frame(motor_id, 0x02, struct.pack(">i", deci_deg))


def single_turn_position_frame(motor_id: int, angle_deg: float) -> bytes:
    if not 0.0 <= angle_deg < 360.0:
        raise ValueError("single-turn angle must be in [0, 360)")
    return _frame(motor_id, 0x03, struct.pack(">H", int(round(angle_deg * 10.0))))


def feedback_request_frame(motor_id: int, feedback_type: int) -> bytes:
    if feedback_type not in range(5):
        raise ValueError("feedback_type must be in 0..4")
    return _frame(motor_id, 0x0E, bytes((feedback_type,)))


def parse_feedback_frame(frame: bytes) -> tuple[int, int, int]:
    """Parse a 9-byte response into ``(motor_id, feedback_type, value)``.

    The returned value is a signed 32-bit integer. Angle values are in 0.1
    degree units, speed values are RPM, and bus voltage is in 0.01 V units.
    """
    if len(frame) != 9:
        raise F32CProtocolError("feedback frame must contain exactly 9 bytes")
    if frame[0] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        raise F32CProtocolError("invalid frame head or tail")
    if frame[7] != bcc(frame[:7]):
        raise F32CProtocolError("feedback BCC mismatch")
    return frame[1], frame[2], struct.unpack(">i", frame[3:7])[0]


class F32CGimbal:
    """Two F32C motors sharing one 3.3 V TTL serial bus.

    ``transport`` must provide ``write(bytes)`` and ``read(size)`` methods.
    Both pyserial objects and K230 ``machine.UART`` objects satisfy this small
    interface. X is motor ID 1 (bottom/yaw); Y is motor ID 2 (top/pitch).
    """

    def __init__(
        self,
        transport,
        *,
        x_id: int = 1,
        y_id: int = 2,
        frame_gap_s: float = 0.002,
        feedback_timeout_s: float = 0.20,
        feedback_retries: int = 3,
        y_min_deg: float = DEFAULT_Y_MIN_DEG,
        y_max_deg: float = DEFAULT_Y_MAX_DEG,
    ) -> None:
        _validate_motor_id(x_id)
        _validate_motor_id(y_id)
        if x_id == y_id:
            raise ValueError("x_id and y_id must be different")
        if frame_gap_s < 0.001:
            raise ValueError("manual requires at least 1 ms between bus frames")
        if not isinstance(feedback_retries, int) or feedback_retries < 1:
            raise ValueError("feedback_retries must be a positive integer")
        if not math.isfinite(feedback_timeout_s) or feedback_timeout_s <= 0:
            raise ValueError("feedback_timeout_s must be positive and finite")
        if not math.isfinite(y_min_deg) or not math.isfinite(y_max_deg):
            raise ValueError("Y hard limits must be finite")
        if y_min_deg >= y_max_deg:
            raise ValueError("y_min_deg must be smaller than y_max_deg")
        self.transport = transport
        self.x_id = x_id
        self.y_id = y_id
        self.frame_gap_s = frame_gap_s
        self.feedback_timeout_s = feedback_timeout_s
        self.feedback_retries = feedback_retries
        self.y_min_deg = float(y_min_deg)
        self.y_max_deg = float(y_max_deg)
        self.zero_deci_deg = {x_id: 0, y_id: 0}
        self.last_relative_deg = {x_id: 0.0, y_id: 0.0}
        self.enabled = False
        self.referenced = False
        self._last_write = 0.0

    def _write(self, frame: bytes) -> None:
        elapsed = time.monotonic() - self._last_write
        if elapsed < self.frame_gap_s:
            time.sleep(self.frame_gap_s - elapsed)
        written = self.transport.write(frame)
        if written is not None and written != len(frame):
            raise F32CError("serial transport accepted only part of a frame")
        self._last_write = time.monotonic()

    def _read_one(self) -> bytes:
        data = self.transport.read(1)
        return bytes(data or b"")

    def _read_feedback(self, timeout_s: float | None = None) -> bytes:
        deadline = time.monotonic() + (
            self.feedback_timeout_s if timeout_s is None else timeout_s
        )
        frame = bytearray()
        observed = bytearray()
        while time.monotonic() < deadline:
            byte = self._read_one()
            if not byte:
                time.sleep(0.001)
                continue
            value = byte[0]
            observed.append(value)
            if len(observed) > 64:
                del observed[:-64]
            if not frame:
                if value == FRAME_HEAD:
                    frame.append(value)
                continue
            frame.append(value)
            if len(frame) == 9:
                try:
                    parse_feedback_frame(bytes(frame))
                    return bytes(frame)
                except F32CProtocolError:
                    # Resynchronize if a new frame head appeared in the bad data.
                    try:
                        next_head = frame[1:].index(FRAME_HEAD) + 1
                        frame = frame[next_head:]
                    except ValueError:
                        frame.clear()
        raw = observed.hex(" ").upper() if observed else "<no bytes received>"
        raise F32CTimeout(
            "timed out waiting for a valid 9-byte feedback frame; RX=%s" % raw
        )

    def request_feedback(self, motor_id: int, feedback_type: int) -> int:
        attempt_errors = []
        unexpected = []
        for _attempt in range(1, self.feedback_retries + 1):
            self._write(feedback_request_frame(motor_id, feedback_type))
            deadline = time.monotonic() + self.feedback_timeout_s
            try:
                while time.monotonic() < deadline:
                    frame = self._read_feedback(
                        max(0.001, deadline - time.monotonic())
                    )
                    response_id, response_type, value = parse_feedback_frame(frame)
                    if response_id == motor_id and response_type == feedback_type:
                        return value
                    unexpected.append((response_id, response_type))
            except F32CTimeout as exc:
                attempt_errors.append(str(exc))

        details = " | ".join(attempt_errors) if attempt_errors else "no matching response"
        if unexpected:
            details += "; unexpected responses=%s" % unexpected
        raise F32CTimeout(
            "motor ID %d feedback type 0x%02X failed after %d attempts: %s"
            % (motor_id, feedback_type, self.feedback_retries, details)
        )
    def start(
        self,
        *,
        speed_rpm: int = 30,
        mode: int = MODE_MULTI_DIRECT,
        acceleration: int | None = None,
        power_on_delay_s: float = 1.5,
        require_feedback: bool = True,
    ) -> None:
        """Check feedback before enabling, then reference both axes in place."""
        if not 1 <= speed_rpm <= 1000:
            raise ValueError("position-mode speed must be in 1..1000 RPM")
        # The vendor examples send one disposable byte, then wait for startup.
        self.transport.write(b"\x00")
        if power_on_delay_s > 0:
            time.sleep(power_on_delay_s)
        # An open COM port alone does not prove the motors are responding.
        # Probe before any torque command, so a silent/wrong port stays disabled.
        try:
            self.capture_software_zero()
        except F32CError:
            self.referenced = False
            if require_feedback:
                raise
        self.configure_position_mode(mode, speed_rpm, acceleration)
        if self.referenced:
            self.enable_at_current_position()
        else:
            # Legacy explicit opt-out: no relative pose is available.
            self.enable()

    def configure_position_mode(
        self, mode: int, speed_rpm: int, acceleration: int | None = None
    ) -> None:
        """Configure both axes while disabled or enabled, without moving them."""
        if not 1 <= speed_rpm <= 1000:
            raise ValueError("position-mode speed must be in 1..1000 RPM")
        for motor_id in (self.x_id, self.y_id):
            self._write(mode_frame(motor_id, mode))
        self.set_position_speed(speed_rpm)
        if acceleration is not None:
            self.set_acceleration(acceleration)

    def enable(self) -> None:
        """Enable both motors without changing the saved software zero."""
        if self.enabled:
            return
        try:
            for motor_id in (self.x_id, self.y_id):
                self._write(enable_frame(motor_id))
        except Exception:
            # The first axis may have accepted its command already.
            for motor_id in (self.x_id, self.y_id):
                try:
                    self._write(disable_frame(motor_id))
                except Exception:
                    pass
            raise
        self.enabled = True

    def enable_at_current_position(self) -> tuple[float, float]:
        """Preload the measured pose before enabling, then hold that pose."""
        current = self.read_relative_angles()
        self.validate_relative_angles(*current)
        x_raw = self.zero_deci_deg[self.x_id] / 10.0 + current[0]
        y_raw = self.zero_deci_deg[self.y_id] / 10.0 + current[1]
        self._write(multi_turn_position_frame(self.x_id, x_raw))
        self._write(multi_turn_position_frame(self.y_id, y_raw))
        try:
            self.enable()
            self.set_relative_angles(*current)
        except Exception:
            self.stop()
            raise
        return current

    def disable(self) -> None:
        """Disable torque while keeping transport and feedback available."""
        if not self.enabled:
            return
        for motor_id in (self.x_id, self.y_id):
            self._write(disable_frame(motor_id))
        self.enabled = False

    def set_position_speed(self, speed_rpm: int) -> None:
        """Set the positive speed limit used by both position-control axes."""
        if not 1 <= speed_rpm <= 1000:
            raise ValueError("position-mode speed must be in 1..1000 RPM")
        for motor_id in (self.x_id, self.y_id):
            self._write(speed_frame(motor_id, speed_rpm))

    def set_acceleration(self, acceleration: int) -> None:
        """Set the documented T-profile acceleration for both axes."""
        for motor_id in (self.x_id, self.y_id):
            self._write(acceleration_frame(motor_id, acceleration))

    def set_speed_pid(self, motor_id: int, kp: int, ki: int) -> None:
        """Set volatile speed-loop KP/KI for one motor."""
        if motor_id not in (self.x_id, self.y_id):
            raise ValueError("motor_id is not an axis of this gimbal")
        self._write(pid_frame(motor_id, PID_SPEED_KP, kp))
        self._write(pid_frame(motor_id, PID_SPEED_KI, ki))

    def save_parameters(self, motor_id: int) -> None:
        """Persist the current controller parameters for one motor."""
        if motor_id not in (self.x_id, self.y_id):
            raise ValueError("motor_id is not an axis of this gimbal")
        self._write(save_parameters_frame(motor_id))

    def validate_relative_angles(self, x_deg: float, y_deg: float) -> None:
        """Reject invalid targets before either position frame is written."""
        if not math.isfinite(x_deg) or not math.isfinite(y_deg):
            raise F32CLimitError("X/Y targets must be finite numbers")
        if not self.y_min_deg <= y_deg <= self.y_max_deg:
            raise F32CLimitError(
                "Y target %.3f deg violates hard limit %.1f..%.1f deg"
                % (y_deg, self.y_min_deg, self.y_max_deg)
            )

    def capture_software_zero(self) -> tuple[float, float]:
        """Read both total-angle registers and save them as software zero."""
        zero_values = {}
        for axis, motor_id in (("X", self.x_id), ("Y", self.y_id)):
            try:
                zero_values[motor_id] = self.request_feedback(
                    motor_id, FEEDBACK_TOTAL_ANGLE
                )
            except F32CTimeout as exc:
                raise F32CTimeout(
                    "%s axis (motor ID %d) zero-reference read failed: %s"
                    % (axis, motor_id, exc)
                ) from exc
        self.zero_deci_deg.update(zero_values)
        self.last_relative_deg[self.x_id] = 0.0
        self.last_relative_deg[self.y_id] = 0.0
        self.referenced = True
        return self.get_software_zero()

    def get_software_zero(self) -> tuple[float, float]:
        """Return the stored X/Y zero references in motor total-angle degrees."""
        if not self.referenced:
            raise F32CError("gimbal has no feedback reference")
        return (
            self.zero_deci_deg[self.x_id] / 10.0,
            self.zero_deci_deg[self.y_id] / 10.0,
        )

    def return_to_zero(self) -> None:
        """Command both axes back to the captured software zero."""
        self.set_relative_angles(0.0, 0.0)

    def set_relative_angles(self, x_deg: float, y_deg: float) -> None:
        """Set X/Y angles relative to the positions captured by :meth:`start`."""
        self.validate_relative_angles(x_deg, y_deg)
        if not self.enabled:
            raise F32CError("gimbal is not enabled")
        if not self.referenced:
            raise F32CError("gimbal has no feedback reference")
        x_raw = self.zero_deci_deg[self.x_id] / 10.0 + x_deg
        y_raw = self.zero_deci_deg[self.y_id] / 10.0 + y_deg
        self._write(multi_turn_position_frame(self.x_id, x_raw))
        self._write(multi_turn_position_frame(self.y_id, y_raw))
        self.last_relative_deg[self.x_id] = x_deg
        self.last_relative_deg[self.y_id] = y_deg

    def read_relative_angles(self) -> tuple[float, float]:
        if not self.referenced:
            raise F32CError("gimbal has no feedback reference")
        values = []
        for motor_id in (self.x_id, self.y_id):
            raw = self.request_feedback(motor_id, FEEDBACK_TOTAL_ANGLE)
            values.append((raw - self.zero_deci_deg[motor_id]) / 10.0)
        return values[0], values[1]

    def hold(self) -> None:
        """Request feedback and command the measured position as the new target."""
        x_deg, y_deg = self.read_relative_angles()
        self.set_relative_angles(x_deg, y_deg)

    def stop(self) -> None:
        """Best-effort disable both motors. Safe to call more than once."""
        if not self.enabled:
            return
        for motor_id in (self.x_id, self.y_id):
            try:
                self._write(disable_frame(motor_id))
            except Exception:
                pass
        self.enabled = False
