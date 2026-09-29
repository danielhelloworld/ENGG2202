"""Standalone, MicroPython-friendly F32C tracker core for K230/CanMV.

Board pinmux and the detector itself are supplied by the caller because K230
carrier boards expose different UART instances and camera APIs.
"""

import math
import struct
import time


HEAD = 0x7A
TAIL = 0x7B


def _ticks_ms():
    fn = getattr(time, "ticks_ms", None)
    return fn() if fn else int(time.time() * 1000)


def _ticks_diff(new, old):
    fn = getattr(time, "ticks_diff", None)
    return fn(new, old) if fn else new - old


def _sleep_ms(milliseconds):
    fn = getattr(time, "sleep_ms", None)
    if fn:
        fn(int(milliseconds))
    else:
        time.sleep(milliseconds / 1000.0)


def _bcc(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


def _frame(motor_id, function, payload=b""):
    body = bytes((HEAD, motor_id, function)) + payload
    return body + bytes((_bcc(body), TAIL))


def _position_frame(motor_id, angle_deg):
    return _frame(motor_id, 0x02, struct.pack(">i", int(round(angle_deg * 10))))


class K230F32CGimbal:
    def __init__(self, uart, x_id=1, y_id=2, frame_gap_ms=2, timeout_ms=250):
        self.uart = uart
        self.x_id = x_id
        self.y_id = y_id
        self.frame_gap_ms = max(1, frame_gap_ms)
        self.timeout_ms = timeout_ms
        self.last_tx_ms = _ticks_ms() - self.frame_gap_ms
        self.zero = {x_id: 0, y_id: 0}
        self.enabled = False

    def _write(self, data):
        wait_ms = self.frame_gap_ms - _ticks_diff(_ticks_ms(), self.last_tx_ms)
        if wait_ms > 0:
            _sleep_ms(wait_ms)
        self.uart.write(data)
        self.last_tx_ms = _ticks_ms()

    def _read_feedback(self, wanted_id, wanted_type):
        start = _ticks_ms()
        frame = bytearray()
        while _ticks_diff(_ticks_ms(), start) < self.timeout_ms:
            data = self.uart.read(1)
            if not data:
                _sleep_ms(1)
                continue
            byte = data[0]
            if not frame and byte != HEAD:
                continue
            frame.append(byte)
            if len(frame) < 9:
                continue
            valid = frame[8] == TAIL and frame[7] == _bcc(frame[:7])
            if valid and frame[1] == wanted_id and frame[2] == wanted_type:
                return struct.unpack(">i", bytes(frame[3:7]))[0]
            frame = bytearray()
        raise RuntimeError("F32C feedback timeout")

    def _request_angle(self, motor_id):
        self._write(_frame(motor_id, 0x0E, b"\x01"))
        return self._read_feedback(motor_id, 1)

    def start(self, speed_rpm=30):
        self.uart.write(b"\x00")
        _sleep_ms(1500)
        for motor_id in (self.x_id, self.y_id):
            self._write(_frame(motor_id, 0x06))
        for motor_id in (self.x_id, self.y_id):
            self._write(_frame(motor_id, 0x00, b"\x00\x03"))
        for motor_id in (self.x_id, self.y_id):
            self._write(_frame(motor_id, 0x01, struct.pack(">h", speed_rpm)))
        self.enabled = True
        self.zero[self.x_id] = self._request_angle(self.x_id)
        self.zero[self.y_id] = self._request_angle(self.y_id)

    def set_angles(self, x_deg, y_deg):
        x_raw_deg = self.zero[self.x_id] / 10.0 + x_deg
        y_raw_deg = self.zero[self.y_id] / 10.0 + y_deg
        self._write(_position_frame(self.x_id, x_raw_deg))
        self._write(_position_frame(self.y_id, y_raw_deg))

    def stop(self):
        if self.enabled:
            self._write(_frame(self.x_id, 0x05))
            self._write(_frame(self.y_id, 0x05))
            self.enabled = False


class K230Tracker:
    def __init__(
        self,
        hfov_deg=70.0,
        vfov_deg=43.0,
        y_dead_width_deg=120.0,
        gain=0.65,
        max_rate_deg_s=70.0,
        x_sign=1.0,
        y_sign=-1.0,
    ):
        self.hfov = hfov_deg
        self.vfov = vfov_deg
        self.gain = gain
        self.max_rate = max_rate_deg_s
        self.x_sign = x_sign
        self.y_sign = y_sign
        # The dead sector is centered behind logical zero (180 degrees).
        safe_half = (360.0 - y_dead_width_deg) / 2.0 - 2.0
        if safe_half <= 0:
            raise ValueError("Y dead zone/margin leaves no safe motion range")
        self.y_min = -safe_half
        self.y_max = safe_half
        self.x = 0.0
        self.y = 0.0
        self.last_ms = _ticks_ms()

    def update(self, target):
        """Update from ``(cx, cy, width, height)``; None means hold."""
        now = _ticks_ms()
        dt = max(0.001, min(0.25, _ticks_diff(now, self.last_ms) / 1000.0))
        self.last_ms = now
        if target is None:
            return self.x, self.y
        cx, cy, width, height = target
        error_x = cx - width / 2.0
        error_y = cy - height / 2.0
        if abs(error_x) < 6:
            error_x = 0.0
        if abs(error_y) < 6:
            error_y = 0.0
        angle_x = math.degrees(
            math.atan(2.0 * error_x / width * math.tan(math.radians(self.hfov) / 2.0))
        )
        angle_y = math.degrees(
            math.atan(2.0 * error_y / height * math.tan(math.radians(self.vfov) / 2.0))
        )
        desired_x = self.x + self.x_sign * self.gain * angle_x
        desired_y = self.y + self.y_sign * self.gain * angle_y
        max_step = self.max_rate * dt
        self.x += max(-max_step, min(max_step, desired_x - self.x))
        self.y += max(-max_step, min(max_step, desired_y - self.y))
        self.y = max(self.y_min, min(self.y_max, self.y))
        return self.x, self.y


def run_tracking(uart, detect_target, y_dead_zone_deg=120.0):
    """Run at 50 Hz. ``detect_target`` returns a target tuple or ``None``."""
    gimbal = K230F32CGimbal(uart)
    tracker = K230Tracker(y_dead_width_deg=y_dead_zone_deg)
    gimbal.start(speed_rpm=30)
    try:
        while True:
            x_deg, y_deg = tracker.update(detect_target())
            gimbal.set_angles(x_deg, y_deg)
            _sleep_ms(20)
    finally:
        gimbal.stop()


# Board-specific bootstrap example (verify the UART/pinmux in your K230 manual):
# from machine import UART
# uart = UART(UART.UART1, 115200, bits=8, parity=None, stop=1, timeout=20)
# run_tracking(uart, detect_drone_center)

