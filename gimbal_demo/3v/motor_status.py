"""Independent F32C motor-status polling functions.

"parameter address" below means the feedback-type byte in an F32C status
response, not the motor ID used to route a control command.
"""

from __future__ import annotations

import time

from f32c_protocol import (
    FEEDBACK_ACCELERATION,
    FEEDBACK_BUS_VOLTAGE,
    FEEDBACK_MECHANICAL_ANGLE,
    FEEDBACK_SPEED,
    FEEDBACK_TOTAL_ANGLE,
)


STATUS_PARAMETERS = {
    FEEDBACK_SPEED: ("speed", "rpm", 1.0),
    FEEDBACK_TOTAL_ANGLE: ("total_angle", "deg", 0.1),
    FEEDBACK_MECHANICAL_ANGLE: ("mechanical_angle", "deg", 0.1),
    # The supplied manual's acceleration example is internally inconsistent,
    # so preserve the raw value rather than applying an uncertain scale.
    FEEDBACK_ACCELERATION: ("acceleration", "raw", 1.0),
    FEEDBACK_BUS_VOLTAGE: ("bus_voltage", "V", 0.01),
}


def read_motor_status(gimbal, motor_id: int, parameter_addresses=None) -> dict:
    """Read current status parameters from one F32C motor.

    Args:
        gimbal: An initialized :class:`f32c_protocol.F32CGimbal` instance.
        motor_id: Response motor address (normally 1 for X or 2 for Y).
        parameter_addresses: Iterable of feedback parameter addresses. When
            omitted, all addresses 0x00 through 0x04 are queried.

    Returns:
        A dictionary containing ``response_motor_address`` and a ``parameters``
        mapping. Every parameter record contains its status parameter address,
        raw protocol value, converted value, and unit.
    """
    addresses = (
        tuple(STATUS_PARAMETERS)
        if parameter_addresses is None
        else tuple(parameter_addresses)
    )
    unknown = [address for address in addresses if address not in STATUS_PARAMETERS]
    if unknown:
        raise ValueError("unknown F32C parameter address: %s" % unknown)

    parameters = {}
    for address in addresses:
        name, unit, scale = STATUS_PARAMETERS[address]
        raw_value = gimbal.request_feedback(motor_id, address)
        parameters[name] = {
            "parameter_address": address,
            "parameter_address_hex": "0x%02X" % address,
            "raw_value": raw_value,
            "value": raw_value * scale,
            "unit": unit,
        }

    return {
        "response_motor_address": motor_id,
        "response_motor_address_hex": "0x%02X" % motor_id,
        "timestamp_s": time.time(),
        "parameters": parameters,
    }


def read_dual_motor_status(gimbal, parameter_addresses=None) -> dict:
    """Read X and Y status sequentially from the shared TTL bus."""
    return {
        "x": read_motor_status(gimbal, gimbal.x_id, parameter_addresses),
        "y": read_motor_status(gimbal, gimbal.y_id, parameter_addresses),
    }


def format_dual_motor_status(status: dict) -> str:
    """Format :func:`read_dual_motor_status` output as one readable line."""
    axis_text = []
    for axis in ("x", "y"):
        motor = status[axis]
        values = []
        for item in motor["parameters"].values():
            values.append(
                "%s=%g%s(%s)"
                % (
                    item["parameter_address_hex"],
                    item["value"],
                    item["unit"],
                    item["raw_value"],
                )
            )
        axis_text.append(
            "%s[motor=%s %s]"
            % (axis.upper(), motor["response_motor_address_hex"], " ".join(values))
        )
    return "STATUS " + " | ".join(axis_text)
