"""Build one MaixVision entry, reusing the tested mechanical-only motor sources."""
import ast
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MOTOR = ROOT.parent / "motor_joystick"


def build():
    chunks, hashes = [], {}

    def add(path, transform=None):
        code = path.read_text(encoding="utf-8")
        hashes[str(path.relative_to(ROOT.parent))] = hashlib.sha256(path.read_bytes()).hexdigest()
        if transform:
            code = transform(code)
        ast.parse(code)
        chunks.append("\n# Source: " + str(path.relative_to(ROOT.parent)) + "\n" + code)

    def visual_motor_config(code):
        # This app's faster profile only; retain the standalone motor demo caps.
        for old, new in (('not 0 < v <= 100', 'not 0 < v <= 2000'),
                         ('not 1 <= c["motor_rpm"] <= 5', 'not 1 <= c["motor_rpm"] <= 80')):
            assert code.count(old) == 1, "Shared motor validator changed: " + old
            code = code.replace(old, new)
        return code

    for name in ("src/config.py", "vendor/protocol.py", "vendor/motor_base.py",
                 "src/service.py", "src/mechanical_only.py"):
        add(MOTOR / name, visual_motor_config if name == "src/config.py" else None)
        if name == "src/config.py":
            add(ROOT / "src/config.py")  # Keep all user settings near the top.

    def web_core(code):
        code = code.split("WEB_HTML =", 1)[0]
        code = code.replace('content_type="application/json; charset=utf-8"):',
                            'content_type="application/json; charset=utf-8", headers=None):')
        code = code.replace('data = body.encode("utf-8") if isinstance(body, str) else',
                            'data = body if isinstance(body, bytes) else body.encode("utf-8") if isinstance(body, str) else')
        code = code.replace('            self.end_headers()',
                            '            for key, value in (headers or {}).items():\n                self.send_header(key, value)\n            self.end_headers()')
        marker = '            elif self.path == "/api/state":'
        assert code.count(marker) == 1
        route = '''            elif self.path == "/api/telemetry":
                self.reply(200, control.telemetry())
            elif self.path == "/api/frame":
                frame = control.media()
                if frame is None:
                    self.reply(503, {"error": "No fresh video"})
                else:
                    self.reply(200, frame["jpeg"], "image/jpeg", {"X-Frame-Id": str(frame["id"])})
'''
        code = code.replace(marker, route + marker)
        marker = '                elif self.path == "/api/stop":'
        assert code.count(marker) == 1
        return code.replace(marker, '''                elif self.path == "/api/background":
                    value = control.background(body.get("token"))
''' + marker)

    add(MOTOR / "src/web.py", web_core)
    for name in ("vision.py", "response.py", "tracking_motor.py", "prediction.py", "tracking.py", "hub.py", "runtime.py"):
        add(ROOT / "src" / name)
    html = ROOT / "src/web.html"
    hashes[str(html.relative_to(ROOT.parent))] = hashlib.sha256(html.read_bytes()).hexdigest()
    chunks.insert(-1, "\nWEB_HTML = " + repr(html.read_text(encoding="utf-8")) + "\n")
    source = ("# Generated standalone YOLO click tracker. Edit CONFIG near the top.\n"
              "# Default startup automatically enables and homes from the current pose.\n") + "\n".join(chunks)
    ast.parse(source)
    (ROOT / "main.py").write_text(source, encoding="utf-8")
    hashes["yolo_click_tracker/build.py"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (ROOT / "source_hashes.json").write_text(json.dumps(hashes, indent=2)+"\n", encoding="utf-8")
    print("Built", ROOT / "main.py")


if __name__ == "__main__":
    build()
