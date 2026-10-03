"""Run the complete control sequence on a PC with synthetic sensors only."""
import copy
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("sound_yolo_simulation", ROOT / "main.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def run():
    c = copy.deepcopy(m.CONFIG)
    c.update(sound_calibrated=True, sound_samples=[(-.7, .2, -35, 0), (0, .2, 0, 0), (.7, .2, 35, 0)])
    ctl = m.SoundVisualController(c)
    angles, previous = (0.0, 0.0), None
    events = []
    for frame in range(120):
        now = frame * .1
        sound = {"valid": now < 1, "quality": .9, "age_seconds": 0, "frames": frame, "vector_xy": (.7, .2)}
        tracks = [{"key": (0, 7), "cx": .58, "cy": .5, "score": .9}] if 2 < now < 6 else []
        target = ctl.step(now, sound, tracks, angles)
        if target is not None:
            angles = tuple(old + m.clip(new-old, -rate*.1, rate*.1) for old, new, rate in zip(angles, target, c["max_rate"]))
        if ctl.state != previous:
            event = {"synthetic": True, "time_s": round(now, 1), "state": ctl.state,
                     "selected": ctl.selected, "angles": [round(v, 2) for v in angles]}
            events.append(event)
            print(json.dumps(event))
            previous = ctl.state
    expected = ["WAIT_SOUND", "SLEW", "SEARCH", "TRACK", "LOST", "WAIT_SOUND"]
    assert [e["state"] for e in events] == expected, events
    return events


if __name__ == "__main__":
    run()
