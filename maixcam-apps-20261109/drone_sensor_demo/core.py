"""Evidence fusion, not a calibrated probability or a drone classifier."""
import json
import math
import os
from collections import deque

DEFAULT = {
    "weights": {"vision": 0.70, "sound": 0.15, "thermal": 0.15},
    "max_age": {"vision": 0.35, "sound": 0.60, "thermal": 0.60},
    "pair_delta": 0.25, "association_radius": 0.12,
    "visual_confirm": 0.80, "confirm_hits": 3, "hold_seconds": 0.8,
    "model_path": "/root/models/yolo11n.mud", "model_type": "YOLO11",
    "drone_labels": ["drone", "uav", "quadcopter"],
    "thermal_to_rgb": None, "sound_to_rgb": None,
    "calibration_resolution": None, "sound_calibration_bounds": None,
    "camera_fps": 60,
    "alignment_path": "/root/thermal_yolo_fusion/alignment.json",
    "thermal_alignment": {},  # Optional original overlay CONFIG overrides/profiles.
    "thermal_detector": {},  # Bad-pixel mask etc.; raw 160x120 coordinates.
    "fixed_camera": True,  # Declared operating mode; no pose compensation.
}


def validate(config):
    w = config["weights"]
    if set(w) != {"vision", "sound", "thermal"}:
        raise ValueError("Exactly three sensor weights required")
    if not all(isinstance(v, (float, int)) and math.isfinite(v) and 0 <= v <= 1 for v in w.values()):
        raise ValueError("Weights must be finite in [0,1]")
    if abs(sum(w.values()) - 1) > 1e-6 or w["vision"] < .60:
        raise ValueError("Weights must sum to 1; vision must be >= 0.60")
    for v in config["max_age"].values():
        if not math.isfinite(v) or v <= 0:
            raise ValueError("Invalid sensor timeout")
    if not .5 <= config["visual_confirm"] <= 1 or not 2 <= config["confirm_hits"] <= 10:
        raise ValueError("Invalid confirmation settings")
    for name in ("association_radius", "pair_delta", "hold_seconds"):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError("Invalid " + name)
    return config


def load_config(path):
    config = json.loads(json.dumps(DEFAULT))
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            config.update(json.load(f))
    return validate(config)


def save_config(path, config):
    validate(config)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    temp = path + ".tmp"
    with open(temp, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)


def observation(kind, x, y, score, timestamp, token, label="", calibrated=True):
    return dict(kind=kind, x=float(x), y=float(y), score=float(score),
                timestamp=float(timestamp), token=str(token), label=label, calibrated=calibrated)


class Fusion:
    """Bounded tracks, one-to-one association; every hit requires a new source frame."""
    def __init__(self, config):
        self.config = validate(config)
        self.tracks = []
        self.serial = 0
        self.seen = set()
        self.seen_order = deque()

    def update(self, observations, now):
        c = self.config
        self.tracks = [t for t in self.tracks if now - t["last"] <= c["hold_seconds"]]
        used = set()
        rejected = []
        for o in sorted(observations, key=lambda a: a.get("kind") != "vision"):
            kind = o.get("kind")
            if kind not in c["weights"]:
                continue
            if (not all(math.isfinite(o[k]) for k in ("x", "y", "score", "timestamp"))
                    or not 0 <= o["score"] <= 1 or not 0 <= o["x"] <= 1 or not 0 <= o["y"] <= 1
                    or not 0 <= now - o["timestamp"] <= c["max_age"][kind]
                    or not o.get("calibrated", False)):
                rejected.append(kind + ": stale/invalid/uncalibrated")
                continue
            key = (kind, o["token"])
            if key in self.seen:
                continue
            self.seen.add(key)
            self.seen_order.append(key)
            while len(self.seen_order) > 4096:
                self.seen.discard(self.seen_order.popleft())
            choices = []
            for t in self.tracks:
                if (t["id"], kind) in used:
                    continue
                distance = math.hypot(t["x"] - o["x"], t["y"] - o["y"])
                time_gate = c["hold_seconds"] if kind in t["evidence"] else c["pair_delta"]
                if distance <= c["association_radius"] and abs(t["last"] - o["timestamp"]) <= time_gate:
                    choices.append((distance, t["id"], t))
            choices.sort(key=lambda item: item[0])
            # Ambiguous candidates never add auxiliary evidence to either target.
            if len(choices) > 1 and choices[1][0] - choices[0][0] < .025:
                rejected.append(kind + ": ambiguous association")
                continue
            if choices:
                t = choices[0][2]
            else:
                self.serial += 1
                t = dict(id=self.serial, x=o["x"], y=o["y"], last=o["timestamp"],
                         evidence={}, visual_hits=deque(maxlen=5), confirmed_at=None)
                self.tracks.append(t)
            used.add((t["id"], kind))
            previous = t["evidence"].get(kind)
            if previous and o["timestamp"] <= previous["timestamp"]:
                continue
            t["evidence"][kind] = o
            t["last"] = max(t["last"], o["timestamp"])
            # Auxiliary directions cannot drag a fresh visual position away.
            v = t["evidence"].get("vision")
            if kind == "vision" or not v or now - v["timestamp"] > c["max_age"]["vision"]:
                t["x"], t["y"] = o["x"], o["y"]
            if kind == "vision":
                good = o["label"].lower() in c["drone_labels"] and o["score"] >= c["visual_confirm"]
                t["visual_hits"].append((o["timestamp"], good))
        result = []
        for t in self.tracks:
            values = {}
            for k in c["weights"]:
                o = t["evidence"].get(k)
                valid = o and 0 <= now - o["timestamp"] <= c["max_age"][k]
                # An auxiliary observation must also be synchronized to the latest evidence.
                valid = valid and abs(t["last"] - o["timestamp"]) <= c["pair_delta"]
                if k == "vision" and o and o["label"].lower() not in c["drone_labels"]:
                    valid = False
                values[k] = o["score"] if valid else 0.0
            hits = sum(good for ts, good in t["visual_hits"] if 0 <= now - ts <= .7)
            confirmed = values["vision"] >= c["visual_confirm"] and hits >= c["confirm_hits"]
            if confirmed:
                t["confirmed_at"] = now
            held = t["confirmed_at"] is not None and now - t["confirmed_at"] <= c["hold_seconds"]
            state = "VISUAL_CONFIRMED" if confirmed else "HOLD_UNCONFIRMED" if held else "CANDIDATE"
            contributions = {k: values[k] * c["weights"][k] for k in values}
            result.append(dict(id=t["id"], x=t["x"], y=t["y"], state=state,
                               evidence=values, contributions=contributions,
                               score=sum(contributions.values()), hits=hits))
        return sorted(result, key=lambda t: (t["state"] == "VISUAL_CONFIRMED", t["score"]), reverse=True)[:24], rejected


def thermal_candidates(pixels):
    """Multi-scale center-surround on original 160x120 gray pixels; no temperature claim."""
    import cv2
    import numpy as np
    raw = np.asarray(pixels, dtype=np.float32)
    if raw.shape != (120, 160) or not np.isfinite(raw).all():
        raise ValueError("Expected finite original Thermal160 120x160 frame")
    response = np.zeros_like(raw)
    for size in (5, 11, 21):
        mean = cv2.boxFilter(raw, -1, (size, size))
        variance = np.maximum(cv2.boxFilter(raw * raw, -1, (size, size)) - mean * mean, 0)
        response = np.maximum(response, (raw - mean) / np.maximum(np.sqrt(variance), 2.0))
    mask = (response > 3.0).astype(np.uint8)
    mask[:2] = mask[-2:] = 0
    mask[:, :2] = mask[:, -2:] = 0
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    output = []
    for i in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[i])
        if area > 600 or max(w, h) / max(1, min(w, h)) > 6:
            continue
        peak = float(response[labels == i].max())
        output.append(dict(x=float(centers[i][0]), y=float(centers[i][1]), area=area,
                           box=[x, y, w, h], score=min(.95, peak / 10)))
    return sorted(output, key=lambda o: o["score"], reverse=True)[:24]
