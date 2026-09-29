"""WHEELTEC F32C TTL protocol and two-axis gimbal driver.

The implementation follows the manuals shipped with the F32C gimbal:
115200-8-N-1, 0x7A head, 0x7B tail, big-endian payload and XOR BCC.
Angles in protocol frames are signed 0.1 degree units.
"""

from __future__ import annotations

import struct
import time


FRAME_HEAD = 0x7A
FRAME_TAIL = 0x7B

MODE_SPEED = 0
MODE_MULTI_T = 1
MODE_SINGLE_T = 2
MODE_MULTI_DIRECT = 3
MODE_SINGLE_DIRECT = 4

FEEDBACK_SPEED = 0
FEEDBACK_TOTAL_ANGLE = 1
FEEDBACK_MECHANICAL_ANGLE = 2
FEEDBACK_ACCELERATION = 3
FEEDBACK_BUS_VOLTAGE = 4


class F32CError(Exception):
    """Base exception for F32C communication errors."""


class F32CTimeout(F32CError):
    """Raised when a complete feedback frame is not received in time."""


class F32CProtocolError(F32CError):
    """Raised when an F32C feedback frame is malformed."""


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
    ) -> None:
        _validate_motor_id(x_id)
        _validate_motor_id(y_id)
        if x_id == y_id:
            raise ValueError("x_id and y_id must be different")
        if frame_gap_s < 0.001:
            raise ValueError("manual requires at least 1 ms between bus frames")
        self.transport = transport
        self.x_id = x_id
        self.y_id = y_id
        self.frame_gap_s = frame_gap_s
        self.feedback_timeout_s = feedback_timeout_s
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
        while time.monotonic() < deadline:
            byte = self._read_one()
            if not byte:
                time.sleep(0.001)
                continue
            value = byte[0]
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
        raise F32CTimeout("timed out waiting for a valid 9-byte feedback frame")

    def request_feedback(self, motor_id: int, feedback_type: int) -> int:
        self._write(feedback_request_frame(motor_id, feedback_type))
        deadline = time.monotonic() + self.feedback_timeout_s
        while time.monotonic() < deadline:
            frame = self._read_feedback(max(0.001, deadline - time.monotonic()))
            response_id, response_type, value = parse_feedback_frame(frame)
            if response_id == motor_id and response_type == feedback_type:
                return value
        raise F32CTimeout("feedback arrived, but not for the requested motor/type")

    def start(
        self,
        *,
        speed_rpm: int = 30,
        mode: int = MODE_MULTI_DIRECT,
        power_on_delay_s: float = 1.5,
        require_feedback: bool = True,
    ) -> None:
        """Wake, enable and reference both axes at their current positions."""
        if not 1 <= speed_rpm <= 1000:
            raise ValueError("position-mode speed must be in 1..1000 RPM")
        # The vendor examples send one disposable byte, then wait for startup.
        self.transport.write(b"\x00")
        if power_on_delay_s > 0:
            time.sleep(power_on_delay_s)
        for motor_id in (self.x_id, self.y_id):
            self._write(enable_frame(motor_id))
        for motor_id in (self.x_id, self.y_id):
            self._write(mode_frame(motor_id, mode))
        for motor_id in (self.x_id, self.y_id):
            self._write(speed_frame(motor_id, speed_rpm))
        self.enabled = True

        try:
            for motor_id in (self.x_id, self.y_id):
                self.zero_deci_deg[motor_id] = self.request_feedback(
                    motor_id, FEEDBACK_TOTAL_ANGLE
                )
            self.referenced = True
        except F32CError:
            if require_feedback:
                self.stop()
                raise
            self.referenced = False

    def set_relative_angles(self, x_deg: float, y_deg: float) -> None:
        """Set X/Y angles relative to the positions captured by :meth:`start`."""
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
        """Disable both motors. Safe to call more than once."""
        if not self.enabled:
            return
        for motor_id in (self.x_id, self.y_id):
            try:
                self._write(disable_frame(motor_id))
            except Exception:
                pass
        self.enabled = False

