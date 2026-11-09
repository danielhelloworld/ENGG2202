"""Keep a real control heartbeat while a person/model reviews feedback.

Run on the PC, not MaixCAM2. JSON lines on stdin are explicit control requests.
No model/API client is embedded: the current Codex chat can read the state file
and send commands to this process, then evaluate the next measured recording.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


def validate_bridge_command(body):
    if not isinstance(body, dict):
        raise ValueError("Expected a JSON object")
    action = body.get("action")
    if action not in ("status", "zero", "arm", "home", "pi", "record_start", "record_stop", "pulse", "stop"):
        raise ValueError("Unknown action")
    if action == "pulse":
        x, y, seconds = (body.get(k, 0) for k in ("x", "y", "seconds"))
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y, seconds))
                or math.hypot(x, y) > 1 or not 0 < seconds <= 1):
            raise ValueError("pulse requires a unit joystick vector and 0 < seconds <= 1")
    if action == "pi":
        if type(body.get("axis")) is not int or body["axis"] not in (0, 1):
            raise ValueError("axis is 0=YAW or 1=PITCH")
        pair = [body.get("kp"), body.get("ki")]
        if all(v is None for v in pair) or any(v is not None and (type(v) is not int or not 0 <= v <= 256) for v in pair):
            raise ValueError("At least one integer kp/ki in 0..256 is required")
    if action == "record_start" and (not isinstance(body.get("note", ""), str) or len(body.get("note", "")) > 200):
        raise ValueError("note must be a string with at most 200 characters")
    return body


class FeedbackBridge:
    def __init__(self, url, output):
        parsed = urlsplit(url)
        if parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.path not in ("", "/") or parsed.query or parsed.fragment or parsed.username:
            raise ValueError("Use the camera origin, e.g. http://CAMERA_IP:8080")
        self.url = url.rstrip("/")
        self.output = Path(output).resolve()
        self.token = None
        self.seq = 0
        self.vector = (0.0, 0.0)
        self.pulse_end = 0.0
        self.commands = queue.Queue(maxsize=16)
        self.state = None
        self.last_published = 0.0
        self.capture_number = 0
        self.work = queue.Queue(maxsize=8)

    def request(self, path, body=None):
        headers = {"Origin": self.url}
        data = None
        if body is not None:
            headers["Content-Type"] = "application/json"
            data = json.dumps(body, allow_nan=False).encode("utf-8")
        with urlopen(Request(self.url+path, data=data, headers=headers), timeout=.4) as response:
            return json.load(response)

    def read_stdin(self):
        for line in sys.stdin:
            try:
                body = validate_bridge_command(json.loads(line))
                self.commands.put_nowait(body)
            except (ValueError, queue.Full) as exc:
                print("COMMAND_REJECTED " + str(exc), flush=True)
        # EOF is an explicit end to this controller, not an indefinite lease.
        self.commands.put({"action": "stop"})

    def publish(self, force=False):
        if force or time.monotonic()-self.last_published >= 1:
            try:
                self.work.put_nowait(("state", self.state))
            except queue.Full:
                pass  # Disk/stdout must never delay a real control heartbeat.
            self.last_published = time.monotonic()

    def write_feedback(self):
        while True:
            item = self.work.get()
            try:
                kind, value = item
                self.output.mkdir(parents=True, exist_ok=True)
                if kind == "state":
                    destination = self.output/"latest_state.json"
                    temporary = self.output/"latest_state.tmp"
                    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
                    temporary.replace(destination)
                    print("FEEDBACK " + json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)
                else:
                    trace = self.request("/api/tuning/export")
                    self.capture_number += 1
                    path = self.output/("trace-%d-%03d.json" % (int(time.time()), self.capture_number))
                    path.write_text(json.dumps(trace, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
                    print("TRACE_SAVED " + str(path), flush=True)
            except Exception as exc:
                print("FEEDBACK_SAVE_FAILED " + str(exc), file=sys.stderr, flush=True)
            finally:
                self.work.task_done()

    def run(self):
        threading.Thread(target=self.write_feedback, daemon=True).start()
        # Wait for autonomous startup to finish without taking ownership mid-HOME.
        deadline = time.monotonic()+35
        while True:
            self.state = self.request("/api/state")
            self.publish()
            if self.state.get("fault"):
                raise RuntimeError(self.state["fault"])
            if self.state.get("ready") and (not self.state.get("armed") and not self.state.get("arm_requested") or self.state.get("startup_takeover")):
                break
            if time.monotonic() > deadline:
                raise RuntimeError("Camera is busy; release its existing browser controller")
            time.sleep(.1)
        connected = self.request("/api/connect", {})
        self.token, self.state = connected["token"], connected["state"]
        self.publish(True)
        threading.Thread(target=self.read_stdin, daemon=True).start()
        try:
            while True:
                started = time.monotonic()
                if started >= self.pulse_end:
                    self.vector = (0.0, 0.0)
                try:
                    command = self.commands.get_nowait()
                except queue.Empty:
                    command = {"action": ""}
                action = command["action"]
                if action == "stop":
                    break
                if action == "status":
                    self.publish(True)
                    action = ""
                if action == "pulse":
                    self.vector = (command.get("x", 0), command.get("y", 0))
                    self.pulse_end = started+command["seconds"]
                    action = ""
                elif action:
                    self.vector, self.pulse_end = (0.0, 0.0), 0.0
                self.seq += 1
                self.state = self.request("/api/control", dict(command, action=action, token=self.token,
                                                              seq=self.seq, x=self.vector[0], y=self.vector[1]))
                if action == "record_stop":
                    try:
                        self.work.put_nowait(("trace", None))
                    except queue.Full:
                        self.state["feedback_export_error"] = "Writer busy; /api/tuning/export remains available"
                self.publish()
                time.sleep(max(0, .08-(time.monotonic()-started)))
        finally:
            if self.token:
                try:
                    self.state = self.request("/api/stop", {"token": self.token})
                    self.publish(True)
                except Exception:
                    pass  # The camera's existing lease/watchdog remains active.
                self.token = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True, help="Camera origin from main_web.py")
    parser.add_argument("--output", default=str(Path(__file__).resolve().parent/"output"/"feedback"))
    args = parser.parse_args()
    try:
        FeedbackBridge(args.url, args.output).run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print("BRIDGE_STOPPED " + str(exc), file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
