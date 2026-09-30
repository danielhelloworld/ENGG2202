"""Command-line demo for F32C dual-motor control and live status polling."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque

from f32c_protocol import (
    FEEDBACK_TOTAL_ANGLE,
    F32CGimbal,
    F32CTimeout,
    bcc,
)
from motor_status import format_dual_motor_status, read_dual_motor_status
from tracking_controller import ImageTrackingController, TrackingConfig


class DryRunTransport:
    """Protocol simulator used by ``--dry-run`` and automated checks."""

    def __init__(self) -> None:
        self.rx = deque()
        self.position = {1: 0, 2: 0}

    def write(self, data: bytes) -> int:
        print("TX", data.hex(" ").upper())
        if len(data) == 9 and data[2] == 0x02:
            self.position[data[1]] = int.from_bytes(data[3:7], "big", signed=True)
        if len(data) == 6 and data[2] == 0x0E:
            motor_id, parameter_address = data[1], data[3]
            if motor_id not in self.position:
                return len(data)
            if parameter_address == 0x00:
                value = 0
            elif parameter_address == 0x01:
                value = self.position.get(motor_id, 0)
            elif parameter_address == 0x02:
                value = self.position.get(motor_id, 0) % 3600
            elif parameter_address == 0x03:
                value = 100
            else:
                value = 1200
            body = bytes((0x7A, motor_id, parameter_address)) + value.to_bytes(
                4, "big", signed=True
            )
            self.rx.extend(body + bytes((bcc(body), 0x7B)))
        return len(data)

    def read(self, size: int = 1) -> bytes:
        out = bytearray()
        while self.rx and len(out) < size:
            out.append(self.rx.popleft())
        return bytes(out)

    def close(self) -> None:
        pass


def _build_transport(args):
    if args.dry_run:
        return DryRunTransport()
    if not args.port:
        raise SystemExit("real hardware mode requires --port, for example COM5")
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("install pyserial first: python -m pip install pyserial") from exc
    return serial.Serial(
        port=args.port,
        baudrate=115200,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.01,
        write_timeout=0.2,
    )


def _build_controller(args) -> ImageTrackingController:
    return ImageTrackingController(
        TrackingConfig(
            horizontal_fov_deg=args.hfov,
            vertical_fov_deg=args.vfov,
            x_direction=args.x_sign,
            y_direction=args.y_sign,
            proportional_gain=args.gain,
            pixel_deadband=args.pixel_deadband,
            max_rate_deg_s=args.max_rate,
            lost_timeout_s=args.lost_timeout,
            # Outside -100..+100 is forbidden. In the circular representation,
            # that is a 160 degree dead sector centered at 180 degrees.
            y_dead_zone_width_deg=args.y_dead_width,
            y_dead_zone_center_deg=args.y_dead_center,
            y_dead_zone_margin_deg=args.y_dead_margin,
        )
    )


def _add_tracking_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--hfov", type=float, default=70.0)
    parser.add_argument("--vfov", type=float, default=43.0)
    parser.add_argument("--x-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--y-sign", type=float, choices=(-1.0, 1.0), default=-1.0)
    parser.add_argument("--gain", type=float, default=0.65)
    parser.add_argument("--pixel-deadband", type=float, default=6.0)
    parser.add_argument("--max-rate", type=float, default=70.0)
    parser.add_argument("--lost-timeout", type=float, default=0.35)
    parser.add_argument(
        "--y-dead-width",
        type=float,
        default=160.0,
        help="forbidden sector width; 160 gives the allowed -100..+100 range",
    )
    parser.add_argument("--y-dead-center", type=float, default=180.0)
    parser.add_argument("--y-dead-margin", type=float, default=0.0)


def _add_status_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--status-rate",
        type=float,
        default=2.0,
        help="status output frequency in Hz; 0 disables it",
    )
    parser.add_argument(
        "--status-all",
        action="store_true",
        help="query addresses 0x00..0x04; default queries 0x00, 0x01 and 0x04",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial device, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--speed", type=int, default=30)
    parser.add_argument("--startup-delay", type=float, default=1.5)
    parser.add_argument("--x-id", type=int, default=1)
    parser.add_argument("--y-id", type=int, default=2)
    parser.add_argument("--feedback-timeout", type=float, default=0.5)
    parser.add_argument("--feedback-retries", type=int, default=3)
    sub = parser.add_subparsers(dest="command", required=True)

    manual = sub.add_parser("manual", help="move both axes once")
    manual.add_argument("--x", type=float, required=True)
    manual.add_argument("--y", type=float, required=True)
    manual.add_argument("--hold-seconds", type=float, default=2.0)
    _add_tracking_options(manual)
    _add_status_options(manual)

    track = sub.add_parser("track-stdin", help="track JSON detector boxes from stdin")
    track.add_argument("--rate", type=float, default=50.0)
    _add_tracking_options(track)
    _add_status_options(track)

    simulate = sub.add_parser("simulate", help="run a moving-target simulation")
    simulate.add_argument("--seconds", type=float, default=4.0)
    simulate.add_argument("--rate", type=float, default=50.0)
    _add_tracking_options(simulate)
    _add_status_options(simulate)

    probe = sub.add_parser(
        "probe", help="query motor IDs without enabling or moving the motors"
    )
    probe.add_argument(
        "--ids", type=int, nargs="+", help="motor IDs to query (defaults to X/Y IDs)"
    )
    return parser


def _handle_detection(controller, bus, item, next_send_s, period_s):
    detected = bool(item.get("detected", True))
    if detected:
        x, y, active = controller.update(
            target_cx=float(item["cx"]),
            target_cy=float(item["cy"]),
            frame_width=int(item["width"]),
            frame_height=int(item["height"]),
        )
    else:
        x, y, active = controller.update(
            target_cx=None,
            target_cy=None,
            frame_width=int(item.get("width", 640)),
            frame_height=int(item.get("height", 480)),
        )
    now = time.monotonic()
    if now >= next_send_s:
        bus.set_relative_angles(x, y)
        next_send_s = now + period_s
        print("target=%s x=%7.2f y=%7.2f" % ("ON" if active else "LOST", x, y))
    return next_send_s


def _poll_status_if_due(bus, args, next_status_s):
    """Poll status sequentially on the same TTL bus as the control frames."""
    if args.status_rate <= 0:
        return float("inf")
    now = time.monotonic()
    if now < next_status_s:
        return next_status_s
    parameter_addresses = None if args.status_all else (0x00, 0x01, 0x04)
    status = read_dual_motor_status(bus, parameter_addresses)
    print(format_dual_motor_status(status))
    return time.monotonic() + 1.0 / args.status_rate


def _run_probe(transport, bus, args) -> None:
    """Query total-angle feedback without enabling or moving any motor."""
    transport.write(b"\x00")
    time.sleep(0.05)
    found = []
    print("Probing F32C feedback on %s (motors remain disabled)..." % args.port)
    for motor_id in (args.ids or (args.x_id, args.y_id)):
        try:
            raw = bus.request_feedback(motor_id, FEEDBACK_TOTAL_ANGLE)
        except F32CTimeout as exc:
            print("ID %d: NO VALID REPLY - %s" % (motor_id, exc))
        else:
            found.append(motor_id)
            print("ID %d: OK, total_angle=%+.1f deg (raw=%d)" % (motor_id, raw / 10.0, raw))
    if not found:
        print("No motor replied. Check 8-15 V motor power, common GND, and crossed TX/RX.")
        print("Also confirm the USB-TTL adapter uses 3.3 V logic, not RS-232 levels.")
    elif set(found) != {args.x_id, args.y_id}:
        print("Expected X ID=%d and Y ID=%d; detected IDs=%s" % (args.x_id, args.y_id, found))


def run(args) -> None:
    transport = _build_transport(args)
    bus = F32CGimbal(
        transport,
        x_id=args.x_id,
        y_id=args.y_id,
        feedback_timeout_s=args.feedback_timeout,
        feedback_retries=args.feedback_retries,
    )
    try:
        if args.command == "probe":
            _run_probe(transport, bus, args)
            return
        bus.start(
            speed_rpm=args.speed,
            power_on_delay_s=0.0 if args.dry_run else args.startup_delay,
        )
        controller = _build_controller(args)
        print(
            "Y allowed interval: %.1f..%.1f deg"
            % (controller.y_dead_zone.low, controller.y_dead_zone.high)
        )

        if args.command == "manual":
            y = controller.safe_y(args.y)
            if y != args.y:
                print("requested Y=%.1f clipped to safe Y=%.1f" % (args.y, y))
            bus.set_relative_angles(args.x, y)
            deadline = time.monotonic() + args.hold_seconds
            next_status_s = 0.0
            while time.monotonic() < deadline:
                next_status_s = _poll_status_if_due(bus, args, next_status_s)
                time.sleep(0.01)
            return

        period_s = 1.0 / args.rate
        next_send_s = 0.0
        next_status_s = 0.0
        if args.command == "track-stdin":
            print('JSON example: {"cx":400,"cy":220,"width":640,"height":480}')
            for line in sys.stdin:
                if not line.strip():
                    continue
                next_send_s = _handle_detection(
                    controller, bus, json.loads(line), next_send_s, period_s
                )
                next_status_s = _poll_status_if_due(bus, args, next_status_s)
            return

        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            phase = time.monotonic() - start
            item = {
                "cx": 320.0 + 220.0 * math.sin(phase * 1.2),
                "cy": 240.0 + 100.0 * math.sin(phase * 0.7),
                "width": 640,
                "height": 480,
            }
            next_send_s = _handle_detection(
                controller, bus, item, next_send_s, period_s
            )
            next_status_s = _poll_status_if_due(bus, args, next_status_s)
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("stopped by user")
    except F32CTimeout as exc:
        print("\nF32C feedback timeout: %s" % exc, file=sys.stderr)
        print(
            "Run: python demo.py --port %s probe --ids 1 2"
            % (args.port or "COM4"),
            file=sys.stderr,
        )
        print(
            "RX=<no bytes received> usually means power/wiring/RX-TX; damaged bytes usually mean ID collision or electrical noise.",
            file=sys.stderr,
        )
        raise SystemExit(2)
    finally:
        bus.stop()
        close = getattr(transport, "close", None)
        if close:
            close()


if __name__ == "__main__":
    run(build_parser().parse_args())
