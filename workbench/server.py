#!/usr/bin/env python3
"""Local MaixCAM 2 workbench gateway and static dashboard server."""

from __future__ import annotations

import argparse
import ipaddress
import json
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parent
WEB_ROOT = ROOT / "web"
MAX_BODY_BYTES = 64 * 1024


class WorkbenchState:
    def __init__(self, device_ip: str, video_port: int, bind_host: str, bind_port: int) -> None:
        self.lock = threading.Lock()
        self.device_ip = str(ipaddress.ip_address(device_ip))
        self.video_port = video_port
        self.ingest_url = f"http://{bind_host}:{bind_port}/api/telemetry"
        self.latest: dict = {}
        self.received_at = 0.0
        self.revision = 0
        self.changed = threading.Condition(self.lock)

    def snapshot(self) -> dict:
        with self.lock:
            age = time.time() - self.received_at if self.received_at else None
            connected = age is not None and age < 3.0
            return {
                "schema_version": 1,
                "connected": connected,
                "age_seconds": round(age, 2) if age is not None else None,
                "received_at": self.received_at or None,
                "revision": self.revision,
                "device": {"name": "MaixCAM 2", "ip": self.device_ip},
                "ingest_url": self.ingest_url,
                "camera": {
                    **self.latest.get("camera", {}),
                    "stream_url": "/api/camera/stream",
                    "available": connected and self.latest.get("camera", {}).get("available", False),
                },
                "runtime": self.latest.get("runtime", {"running": False}),
                "vision": self.latest.get("vision", {}),
                "tracking": self.latest.get("tracking", {}),
                "gimbal": self.latest.get("gimbal", {}),
                "system": self.latest.get("system", {}),
                "extensions": self.latest.get("extensions", {}),
            }

    def update(self, payload: dict) -> int:
        with self.changed:
            self.latest = payload
            self.received_at = time.time()
            self.revision += 1
            self.changed.notify_all()
            return self.revision

    def update_connection(self, device_ip: str | None, video_port: int | None) -> None:
        validated_ip = str(ipaddress.ip_address(device_ip)) if device_ip else None
        validated_port = int(video_port) if video_port is not None else None
        if validated_port is not None and not 1 <= validated_port <= 65535:
            raise ValueError("video_port must be between 1 and 65535")
        with self.lock:
            if validated_ip:
                self.device_ip = validated_ip
            if validated_port is not None:
                self.video_port = validated_port
            self.revision += 1


def create_handler(state: WorkbenchState):
    class Handler(BaseHTTPRequestHandler):
        server_version = "MaixWorkbench/1.0"

        def _send_json(self, body: dict, status: int = 200) -> None:
            payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def _read_json(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > MAX_BODY_BYTES:
                raise ValueError("Request body must be between 1 byte and 64 KiB")
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("JSON root must be an object")
            return body

        def do_GET(self) -> None:
            if self.path == "/api/health":
                self._send_json({"ok": True, "service": "maix-workbench"})
                return
            if self.path == "/api/state":
                self._send_json(state.snapshot())
                return
            if self.path == "/api/config":
                with state.lock:
                    config = {
                        "device_ip": state.device_ip,
                        "video_port": state.video_port,
                        "ingest_url": state.ingest_url,
                    }
                self._send_json(config)
                return
            if self.path == "/api/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                revision = -1
                try:
                    while True:
                        with state.changed:
                            state.changed.wait_for(
                                lambda: state.revision != revision, timeout=1.0
                            )
                            revision = state.revision
                        data = json.dumps(state.snapshot(), ensure_ascii=False)
                        self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
            if self.path == "/api/camera/stream":
                self._proxy_camera_stream()
                return
            if self.path == "/" or self.path == "/index.html":
                self._serve_static("index.html")
                return
            if self.path.startswith("/web/"):
                self._serve_static(self.path.removeprefix("/web/"))
                return
            self._send_json({"error": "Not found"}, 404)

        def _serve_static(self, relative: str) -> None:
            candidate = (WEB_ROOT / relative).resolve()
            if WEB_ROOT.resolve() not in candidate.parents and candidate != WEB_ROOT.resolve():
                self._send_json({"error": "Not found"}, 404)
                return
            if not candidate.is_file():
                self._send_json({"error": "Not found"}, 404)
                return
            content = candidate.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", mimetypes.guess_type(candidate.name)[0] or "application/octet-stream")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(content)

        def _proxy_camera_stream(self) -> None:
            url = f"http://{state.device_ip}:{state.video_port}/stream.mjpg"
            try:
                upstream = urlopen(url, timeout=2.0)
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    upstream.headers.get("Content-Type", "multipart/x-mixed-replace; boundary=frame"),
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                while True:
                    chunk = upstream.read(16 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError, URLError):
                return

        def do_POST(self) -> None:
            if self.path == "/api/telemetry":
                try:
                    payload = self._read_json()
                    state.update(payload)
                    self._send_json({"ok": True, "revision": state.revision}, 202)
                except (ValueError, json.JSONDecodeError) as exc:
                    self._send_json({"error": str(exc)}, 400)
                return
            if self.path == "/api/config":
                try:
                    payload = self._read_json()
                    state.update_connection(
                        payload.get("device_ip"), payload.get("video_port")
                    )
                    self._send_json({"ok": True, "state": state.snapshot()})
                except (ValueError, TypeError) as exc:
                    self._send_json({"error": str(exc)}, 400)
                return
            self._send_json({"error": "Not found"}, 404)

        def log_message(self, format: str, *args) -> None:
            print("[workbench] " + format % args)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the MaixCAM 2 developer workbench")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address; use the Mac USB-network IP for device telemetry ingress")
    parser.add_argument("--port", type=int, default=8760)
    parser.add_argument("--device-ip", default="10.177.5.1")
    parser.add_argument("--video-port", type=int, default=8080)
    args = parser.parse_args()
    state = WorkbenchState(args.device_ip, args.video_port, args.host, args.port)
    server = ThreadingHTTPServer((args.host, args.port), create_handler(state))
    server.daemon_threads = True
    print(f"MaixCAM 2 Workbench: http://{args.host}:{args.port}/")
    print(f"Device stream source: http://{state.device_ip}:{state.video_port}/stream.mjpg")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nWorkbench stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
