"""Read-only PC capture: GET telemetry/state, never sends motor controls."""
import argparse
import json
import math
from pathlib import Path
import time
import urllib.error
import urllib.request


def summarize(data):
    state = data.get("state", {})
    limit = state.get("feedback_loss_grace_s", .1)
    rows = data.get("samples", [])
    usable = [row for row in rows if row.get("armed") and row.get("commanded_deg") is not None
              and not row.get("feedback_degraded") and not row.get("motion_paused") and not row.get("motion_warning")
              and row.get("feedback_age_s") is not None and 0 <= row["feedback_age_s"] < limit]
    errors = [row["response"]["error_deg"] for row in usable
              if all(math.isfinite(v) for v in row.get("response", {}).get("error_deg", (float("nan"),)*2))]
    rms = None if not errors else [math.sqrt(sum(e[axis]**2 for e in errors)/len(errors)) for axis in (0, 1)]
    return dict(total_samples=len(rows), eligible_samples=len(errors),
                moving_samples=sum(any(abs(v) >= 1 for v in row.get("requested_rate_deg_s", (0,0))) for row in usable),
                rms_command_feedback_error_deg=rms,
                note="Disabled/stale targets are excluded. No torque/current measurement or motor command is issued by this tool.")


def capture(base, seconds=3., interval=.1):
    base = base.rstrip("/")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def read(path):
        with opener.open(base+path, timeout=2) as response:
            return json.load(response)

    try:
        data = read("/api/telemetry")
    except urllib.error.HTTPError as exc:
        if exc.code != 404:
            raise
        # Old board versions have no history API. Do not treat their disabled
        # stale target field as a meaningful position-control error.
        data = dict(schema=0, samples=[], compatibility="old board state snapshots")
        until = time.monotonic()+seconds
        while True:
            state = read("/api/state")
            data["state"] = state
            target, actual = state.get("target"), state.get("angles")
            armed = bool(state.get("armed"))
            data["samples"].append(dict(t=time.monotonic(), armed=armed,
                actual_deg=actual, commanded_deg=target if armed else None,
                feedback_age_s=state.get("age"), feedback_degraded=state.get("feedback_degraded", False),
                response=dict(error_deg=[q-a for q,a in zip(target, actual)] if armed and target is not None else (0., 0.))))
            if time.monotonic() >= until or len(data["samples"]) >= 600:
                break
            time.sleep(interval)
    data.update(source=base, read_only=True)
    data["summary"] = summarize(data)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://192.168.100.210:8080")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]/"validation/latest_feedback.json")
    args = parser.parse_args()
    data = capture(args.url)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(data["summary"], ensure_ascii=False))
    print("Saved", args.output)


if __name__ == "__main__":
    main()
