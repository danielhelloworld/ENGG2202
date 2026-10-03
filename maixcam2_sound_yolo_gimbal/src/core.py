"""Pure sound -> slew -> settle -> search -> locked-ID visual controller."""
import math


def clip(value, low, high):
    return max(low, min(high, value))


def sound_to_angles(sound, c):
    if not c["sound_calibrated"] or not sound.get("valid") or sound.get("quality", 0) < c["sound_quality"]:
        return None
    age = sound.get("age_seconds")
    vector = sound.get("vector_xy")
    if age is None or not 0 <= age <= c["sound_fresh_s"] or vector is None:
        return None
    if not all(math.isfinite(v) for v in vector):
        return None
    neighbours = sorted((math.hypot(vector[0] - s[0], vector[1] - s[1]), s) for s in c["sound_samples"])
    neighbours = [(d, s) for d, s in neighbours[:3] if d <= c["sound_radius"]]
    if not neighbours:
        return None  # No extrapolation outside measured neighbourhoods.
    # Reject overlapping calibration regions with incompatible directions.
    if any(max(s[i] for _, s in neighbours) - min(s[i] for _, s in neighbours) > 35 for i in (2, 3)):
        return None
    if neighbours[0][0] < 1e-6:
        return tuple(neighbours[0][1][2:])
    weights = [1 / max(d * d, 1e-8) for d, _ in neighbours]
    return tuple(sum(w * s[i] for w, (_, s) in zip(weights, neighbours)) / sum(weights) for i in (2, 3))


class SoundVisualController:
    def __init__(self, c):
        self.c = c
        self.state = "WAIT_SOUND"
        self.selected = None
        self.last_seen = None
        self.target = None
        self.anchor = None
        self.since = 0.0
        self.settled_at = None
        self.sound_start = None
        self.sound_anchor = None
        self.last_sound_frame = -1
        self.candidate = None
        self.hits = 0
        self.last_tick = None
        self.last_visual = None
        self.filtered = [0.0, 0.0]
        self.reason = "waiting for calibrated sound"

    def reset(self, now, angles, sound_frame=-1):
        self.state, self.selected = "WAIT_SOUND", None
        self.target = tuple(angles)
        self.since = now
        self.sound_start = self.sound_anchor = None
        self.last_sound_frame = sound_frame
        self.candidate, self.hits = None, 0
        self.filtered = [0.0, 0.0]
        self.last_visual = None
        self.reason = "waiting for new sound"

    def select(self, key, tracks, now, angles):
        if self.state == "FAULT" or not any(t["key"] == key for t in tracks):
            return False
        self.selected, self.last_seen, self.state = key, now, "TRACK"
        self.target = tuple(angles)
        self.filtered = [0.0, 0.0]
        self.last_visual = now
        return True

    def fail(self, reason):
        self.state, self.reason, self.selected = "FAULT", reason, None

    def _bounded(self, target):
        return tuple(clip(v, *limits) for v, limits in zip(target, self.c["limits"]))

    def step(self, now, sound, tracks, angles, vision_fresh=True, new_vision=True):
        dt = min(.15, max(0.0, now - self.last_tick)) if self.last_tick is not None else .1
        self.last_tick = now
        if self.target is None:
            self.target = tuple(angles)
        if self.state == "FAULT":
            return None
        frame = sound.get("frames", -1)
        if self.state == "WAIT_SOUND":
            direction = sound_to_angles(sound, self.c)
            if direction is None:
                self.sound_start = self.sound_anchor = None
                self.reason = "sound invalid, stale or outside calibration"
            elif frame != self.last_sound_frame:
                self.last_sound_frame = frame
                if self.sound_anchor is None or max(abs(a-b) for a, b in zip(direction, self.sound_anchor)) > self.c["sound_stable_deg"]:
                    self.sound_start, self.sound_anchor = now, direction
                elif now - self.sound_start >= self.c["sound_stable_s"]:
                    # FIXED BASE: calibrated absolute direction, never add present yaw.
                    self.target = self.anchor = self._bounded(direction)
                    self.state, self.since, self.settled_at = "SLEW", now, None
                    self.reason = "turning to sound direction"
        elif self.state == "SLEW":
            if now - self.since > self.c["turn_timeout"]:
                self.fail("motor did not reach sound direction")
                return None
            if max(abs(a-b) for a, b in zip(angles, self.target)) <= self.c["settle_tolerance"]:
                if self.settled_at is None:
                    self.settled_at = now
                if now - self.settled_at >= self.c["settle_s"]:
                    self.state, self.since = "SEARCH", now
                    self.reason = "looking near sound direction"
            else:
                self.settled_at = None
        elif self.state == "SEARCH":
            if now - self.since > self.c["search_timeout"]:
                self.reset(now, angles, frame)
            else:
                radius = min(self.c["search_amplitude"], self.c["search_rate"] * self.c["search_timeout"] / (2 * math.pi))
                # A continuous bounded sweep beginning at the sound direction.
                offset = radius * math.sin((now - self.since) * 2 * math.pi / self.c["search_timeout"])
                self.target = self._bounded((self.anchor[0] + offset, self.anchor[1]))
                if new_vision:
                    ranked = sorted((math.hypot((t["cx"]-.5)*2, (t["cy"]-.5)*2), t["key"], t) for t in tracks) if vision_fresh else []
                    valid = bool(ranked and ranked[0][0] <= self.c["acquire_radius"])
                    if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < self.c["ambiguity_margin"]:
                        valid = False
                        self.reason = "ambiguous targets: tap one"
                    key = ranked[0][1] if valid else None
                    self.hits = self.hits + 1 if key is not None and key == self.candidate else (1 if key is not None else 0)
                    self.candidate = key
                    if self.hits >= self.c["confirmation_frames"]:
                        self.select(key, tracks, now, angles)
        elif self.state in ("TRACK", "LOST"):
            found = next((t for t in tracks if t["key"] == self.selected), None) if vision_fresh else None
            if found is None:
                if self.state != "LOST":
                    self.target = tuple(angles)  # Cancel the previous moving target.
                    self.filtered = [0.0, 0.0]
                self.state, self.reason = "LOST", "holding for same ID"
                if now - self.last_seen >= self.c["lost_timeout"]:
                    self.reset(now, angles, frame)
            elif new_vision:
                dt = min(.2, max(0.0, now - self.last_visual)) if self.last_visual is not None else .1
                self.last_visual = now
                self.last_seen, self.state, self.reason = now, "TRACK", "tracking selected ID"
                errors = (found["cx"] - .5, found["cy"] - .5)
                result = []
                for axis in range(2):
                    err = errors[axis]
                    if abs(err) < self.c["deadzone"][axis]:
                        self.filtered[axis] = 0.0
                        result.append(angles[axis])
                        continue
                    angle_error = math.degrees(math.atan(2 * err * math.tan(math.radians(self.c["camera_fov"][axis] / 2))))
                    correction = angle_error * self.c["pixel_sign"][axis] * self.c["visual_gain"][axis]
                    alpha = 1 - (1 - self.c["ema"][axis]) ** (dt * self.c["inference_fps"])
                    self.filtered[axis] += alpha * (correction - self.filtered[axis])
                    cap = min(self.c["step_cap"][axis], self.c["max_rate"][axis] * dt)
                    result.append(angles[axis] + clip(self.filtered[axis], -cap, cap))
                self.target = self._bounded(result)
        return self.target
