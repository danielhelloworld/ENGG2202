"""Serve the latest annotated tracker frame to a browser over HTTP MJPEG."""

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class _ThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class MjpegStreamer:
    def __init__(self, host: str = "0.0.0.0", port: int = 8080) -> None:
        self.host = host
        self.port = port
        self._frame = None
        self._lock = threading.Lock()

        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path == "/" or self.path == "/index.html":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(
                        b'<html><meta name="viewport" content="width=device-width">'
                        b'<title>MaixCAM 2 Person Tracker</title>'
                        b'<body style="background:#111;color:#eee;font-family:sans-serif">'
                        b'<h3>MaixCAM 2 Person Tracker</h3>'
                        b'<img src="/stream.mjpg" style="max-width:100%;height:auto">'
                        b'</body></html>'
                    )
                    return

                if self.path != "/stream.mjpg":
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.end_headers()
                try:
                    while True:
                        with owner._lock:
                            jpeg = owner._frame
                        if jpeg is None:
                            threading.Event().wait(0.05)
                            continue
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n"
                            + b"Content-Length: "
                            + str(len(jpeg)).encode("ascii")
                            + b"\r\n\r\n"
                            + jpeg
                            + b"\r\n"
                        )
                        self.wfile.flush()
                        threading.Event().wait(0.05)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass

            def log_message(self, format: str, *args) -> None:
                pass

        self._server = _ThreadingHTTPServer((host, port), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="mjpeg-http",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()
        print(f"Camera preview: http://<MaixCAM-2-IP>:{self.port}/")

    def update(self, frame) -> None:
        jpeg_image = frame.to_jpeg(quality=75)
        jpeg = bytes(jpeg_image.to_bytes())
        with self._lock:
            self._frame = jpeg

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
