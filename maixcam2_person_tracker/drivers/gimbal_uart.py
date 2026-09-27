"""Single-owner UART service for two daisy-chained F32C motors."""

from typing import Dict, Optional

from maix import err, pinmap, time, uart

from config import AxisConfig, GimbalConfig
from domain import GimbalCommand, GimbalState
from drivers import f32c_protocol as protocol


class F32CGimbal:
    def __init__(self, config: GimbalConfig) -> None:
        self.config = config
        err.check_raise(
            pinmap.set_pin_function(config.uart_tx_pin, "UART4_TX"),
            "Failed to map the UART4 TX pin",
        )
        err.check_raise(
            pinmap.set_pin_function(config.uart_rx_pin, "UART4_RX"),
            "Failed to map the UART4 RX pin",
        )
        self.serial = uart.UART(config.uart_device, config.baud_rate)
        self.parser = protocol.FeedbackParser()
        self.axis_by_id: Dict[int, AxisConfig] = {
            config.pan.motor_id: config.pan,
            config.tilt.motor_id: config.tilt,
        }
        self.angle_by_id: Dict[int, float] = {}
        self.feedback_timestamp_by_id: Dict[int, int] = {}
        self.last_feedback_request_ms: Optional[int] = None
        self.next_feedback_id = config.pan.motor_id
        self.started = False

    def _write(self, frame: bytes) -> None:
        if self.config.inter_frame_delay_ms > 0:
            time.sleep_ms(self.config.inter_frame_delay_ms)
        written = self.serial.write(frame)
        if written != len(frame):
            raise RuntimeError(
                f"Incomplete UART write: expected {len(frame)}, wrote {written}"
            )

    def _write_for_both(self, frame_builder, value=None) -> None:
        for axis in (self.config.pan, self.config.tilt):
            frame = (
                frame_builder(axis.motor_id)
                if value is None
                else frame_builder(axis.motor_id, value)
            )
            self._write(frame)

    def start(self) -> None:
        self._write_for_both(protocol.enable)
        self._write_for_both(protocol.select_mode, protocol.MODE_MULTI_TURN_DIRECT)
        self._write_for_both(
            protocol.set_acceleration, self.config.motor_acceleration_rpm_s
        )
        self._write_for_both(protocol.set_speed, self.config.motor_speed_limit_rpm)
        self.started = True

    @staticmethod
    def _logical_from_raw(raw_angle_deg: float, axis: AxisConfig) -> float:
        return raw_angle_deg * axis.motor_sign + axis.zero_offset_deg

    @staticmethod
    def _raw_from_logical(logical_angle_deg: float, axis: AxisConfig) -> float:
        return (logical_angle_deg - axis.zero_offset_deg) / axis.motor_sign

    def _handle_feedback(self, now_ms: int, data: bytes) -> None:
        for motor_id, feedback_type, raw_value in self.parser.feed(data):
            if feedback_type != protocol.FEEDBACK_TOTAL_ANGLE:
                continue
            axis = self.axis_by_id.get(motor_id)
            if axis is None:
                continue
            raw_angle_deg = raw_value / 10.0
            self.angle_by_id[motor_id] = self._logical_from_raw(raw_angle_deg, axis)
            self.feedback_timestamp_by_id[motor_id] = now_ms

    def poll(self, now_ms: int) -> None:
        received = self.serial.read()
        if received:
            self._handle_feedback(now_ms, bytes(received))

        if (
            self.last_feedback_request_ms is None
            or now_ms - self.last_feedback_request_ms
            >= self.config.feedback_request_period_ms
        ):
            self._write(
                protocol.request_feedback(
                    self.next_feedback_id, protocol.FEEDBACK_TOTAL_ANGLE
                )
            )
            self.last_feedback_request_ms = now_ms
            self.next_feedback_id = (
                self.config.tilt.motor_id
                if self.next_feedback_id == self.config.pan.motor_id
                else self.config.pan.motor_id
            )

    def state(self, now_ms: int) -> GimbalState:
        pan_id = self.config.pan.motor_id
        tilt_id = self.config.tilt.motor_id
        return GimbalState(
            timestamp_ms=now_ms,
            pan_deg=self.angle_by_id.get(pan_id, 0.0),
            tilt_deg=self.angle_by_id.get(tilt_id, 0.0),
            pan_valid=pan_id in self.angle_by_id,
            tilt_valid=tilt_id in self.angle_by_id,
        )

    def feedback_age_ms(self, now_ms: int) -> int:
        timestamps = [
            self.feedback_timestamp_by_id.get(self.config.pan.motor_id),
            self.feedback_timestamp_by_id.get(self.config.tilt.motor_id),
        ]
        if any(value is None for value in timestamps):
            return 2 ** 31 - 1
        return max(now_ms - int(value) for value in timestamps)

    def command(self, command: GimbalCommand) -> None:
        if not command.enabled:
            return
        pan_raw = self._raw_from_logical(command.pan_target_deg, self.config.pan)
        tilt_raw = self._raw_from_logical(command.tilt_target_deg, self.config.tilt)
        self._write(
            protocol.set_multi_turn_position(self.config.pan.motor_id, pan_raw)
        )
        self._write(
            protocol.set_multi_turn_position(self.config.tilt.motor_id, tilt_raw)
        )

    def hold_current(self, now_ms: int) -> None:
        current = self.state(now_ms)
        if not current.valid:
            return
        self.command(
            GimbalCommand(
                timestamp_ms=now_ms,
                pan_target_deg=current.pan_deg,
                tilt_target_deg=current.tilt_deg,
                enabled=True,
            )
        )

    def disable(self) -> None:
        self._write_for_both(protocol.disable)

    def enter_fault(self, now_ms: int) -> None:
        self.hold_current(now_ms)
        if self.config.disable_on_fault:
            self.disable()

    def shutdown(self, now_ms: int) -> None:
        self.hold_current(now_ms)
        if self.config.disable_on_normal_exit:
            self.disable()


class VisionOnlyGimbal:
    """Stationary adapter used while the physical gimbal remains disabled."""

    def start(self) -> None:
        return None

    def poll(self, now_ms: int) -> None:
        return None

    def state(self, now_ms: int) -> GimbalState:
        return GimbalState(now_ms, 0.0, 0.0, True, True)

    def feedback_age_ms(self, now_ms: int) -> int:
        return 0

    def command(self, command: GimbalCommand) -> None:
        return None

    def enter_fault(self, now_ms: int) -> None:
        return None

    def shutdown(self, now_ms: int) -> None:
        return None
