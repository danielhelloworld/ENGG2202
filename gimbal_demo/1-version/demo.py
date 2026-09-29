"""Command-line demo for manual motion and detector-driven tracking."""

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
    bcc,
    parse_feedback_frame,
)
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
            motor_id, feedback_type = data[1], data[3]
            value = self.position.get(motor_id, 0)
            body = bytes((0x7A, motor_id, feedback_type)) + value.to_bytes(
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
            y_dead_zone_width_deg=args.y_dead_width,
            y_dead_zone_center_deg=args.y_dead_center,
            y_dead_zone_margin_deg=args.y_dead_margin,
        )
    )


def _add_tracking_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--hfov", type=float, default=70.0, help="camera horizontal FOV")
    parser.add_argument("--vfov", type=float, default=43.0, help="camera vertical FOV")
    parser.add_argument("--x-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--y-sign", type=float, choices=(-1.0, 1.0), default=-1.0)
    parser.add_argument("--gain", type=float, default=0.65)
    parser.add_argument("--pixel-deadband", type=float, default=6.0)
    parser.add_argument("--max-rate", type=float, default=70.0, help="deg/s command slew")
    parser.add_argument("--lost-timeout", type=float, default=0.35)
    parser.add_argument("--y-dead-width", type=float, default=120.0)
    parser.add_argument("--y-dead-center", type=float, default=180.0)
    parser.add_argument("--y-dead-margin", type=float, default=2.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial device, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--dry-run", action="store_true", help="print frames, no hardware")
    parser.add_argument("--speed", type=int, default=30, help="position speed, RPM")
    parser.add_argument("--startup-delay", type=float, default=1.5)
    sub = parser.add_subparsers(dest="command", required=True)

    manual = sub.add_parser("manual", help="move both axes once")
    manual.add_argument("--x", type=float, required=True)
    manual.add_argument("--y", type=float, required=True)
    manual.add_argument("--hold-seconds", type=float, default=2.0)
    _add_tracking_options(manual)

    track = sub.add_parser("track-stdin", help="track JSON detector boxes from stdin")
    track.add_argument("--rate", type=float, default=50.0, help="maximum command rate")
    _add_tracking_options(track)

    simulate = sub.add_parser("simulate", help="run a moving-target tracking simulation")
    simulate.add_argument("--seconds", type=float, default=4.0)
    simulate.add_argument("--rate", type=float, default=50.0)
    _add_tracking_options(simulate)
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
        print(f"target={'ON' if active else 'LOST'} x={x:7.2f} y={y:7.2f}")
    return next_send_s


def run(args) -> None:
    transport = _build_transport(args)
    bus = F32CGimbal(transport)
    try:
        bus.start(
            speed_rpm=args.speed,
            power_on_delay_s=0.0 if args.dry_run else args.startup_delay,
        )
        controller = _build_controller(args)
        print(
            "Y safe interval: "
            f"{controller.y_dead_zone.low:.1f}..{controller.y_dead_zone.high:.1f} deg"
        )

        if args.command == "manual":
            y = controller.y_dead_zone.clamp(args.y)
            if y != args.y:
                print(f"requested Y={args.y:.1f} clipped to safe Y={y:.1f}")
            bus.set_relative_angles(args.x, y)
            time.sleep(args.hold_seconds)
            return

        period_s = 1.0 / args.rate
        next_send_s = 0.0
        if args.command == "track-stdin":
            print('JSON line example: {"cx":400,"cy":220,"width":640,"height":480}')
            for line in sys.stdin:
                if not line.strip():
                    continue
                next_send_s = _handle_detection(
                    controller, bus, json.loads(line), next_send_s, period_s
                )
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
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        bus.stop()
        close = getattr(transport, "close", None)
        if close:
            close()


if __name__ == "__main__":
    run(build_parser().parse_args())

