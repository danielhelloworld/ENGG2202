"""Read-only board performance sampler. Does not ARM, select, or move motors."""
import argparse
import http.client
import json
from pathlib import Path
import statistics
import time
from urllib.parse import urlsplit


def run(url, duration, output, frames=False):
    parsed = urlsplit(url)
    conn = http.client.HTTPConnection(parsed.hostname, parsed.port or 80, timeout=3)
    rows, started, next_frame = [], time.monotonic(), 0.
    try:
        while time.monotonic()-started < duration:
            before = time.monotonic()
            conn.request("GET", "/api/state")
            response = conn.getresponse()
            if response.status != 200:
                raise RuntimeError("State HTTP %d" % response.status)
            state = json.loads(response.read())
            row = dict(host_s=time.monotonic()-started, state_rtt_ms=(time.monotonic()-before)*1000, state=state)
            if frames and time.monotonic() >= next_frame:
                before = time.monotonic()
                conn.request("GET", "/api/frame")
                response = conn.getresponse()
                data = response.read()
                row["web_frame"] = dict(status=response.status, bytes=len(data),
                    id=response.getheader("X-Frame-Id"), rtt_ms=(time.monotonic()-before)*1000)
                next_frame = before+.25
            rows.append(row)
            time.sleep(.15)
    finally:
        conn.close()
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(dict(synthetic=False, readonly=True, web_frames=frames, rows=rows), indent=2), encoding="utf-8")
    tail = rows[len(rows)//2:]
    keys = ("inference_fps_actual", "local_preview_fps_actual", "capture_to_control_p50_ms", "capture_to_control_p95_ms")
    result = {key: round(statistics.median([r["state"]["performance"][key] for r in tail if r["state"]["performance"].get(key) is not None]), 2) for key in keys}
    stages = tail[-1]["state"]["performance"].get("stages", {})
    result.update(stage_p50_ms={k:v["p50_ms"] for k,v in stages.items()},
                  faults=list({r["state"]["fault"] for r in rows if r["state"]["fault"]}),
                  final_angles=rows[-1]["state"]["angles"], version=rows[-1]["state"]["app_version"])
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://192.168.100.210:8080")
    parser.add_argument("--duration", type=float, default=15.)
    parser.add_argument("--output", required=True)
    parser.add_argument("--frames", action="store_true")
    args = parser.parse_args()
    run(args.url, args.duration, args.output, args.frames)
