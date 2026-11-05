"""Thermal-only connected hot regions and one manually selected target.

Candidate coordinates always refer to original, unflipped Thermal160 pixels.
The display uses Alignment.matrix, the same map used by overlay and ROI heat.
"""


def hot_candidates(frame, config):
    if frame is None or not frame["valid_temperature"]:
        return []
    pixels = frame["pixels"]
    if pixels.shape != (THERMAL_H, THERMAL_W):
        return []
    lo, hi = frame["lo"] / 10.0, frame["hi"] / 10.0
    temps = lo + (hi - lo) * np.minimum(pixels.astype(np.float32) / 254.0, 1.0)
    threshold = max(config["hot_min_c"], float(np.median(temps)) + config["hot_delta_c"])
    binary = (temps >= threshold).astype(np.uint8)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)
    candidates = []
    for label in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[label])
        if area < config["hot_min_area_px"]:
            continue
        values = temps[y:y + h, x:x + w][labels[y:y + h, x:x + w] == label]
        candidates.append({"center": tuple(float(v) for v in centroids[label]),
                           "box": (x, y, w, h), "area": area,
                           "max_c": float(values.max()), "mean_c": float(values.mean()),
                           "label": label})
    candidates.sort(key=lambda item: (-item["max_c"], -item["area"]))
    return candidates[:config["hot_max_candidates"]]


class HotTracker:
    def __init__(self, config):
        self.config = config
        self.mode = "OFF"
        self.candidates = []
        self.selected = None
        self.selected_number = 0
        self.misses = 0
        self.last_seen_at = None
        self.frame = None
        self.thermal_ok = False

    def toggle(self):
        if self.mode == "OFF":
            self.mode = "PICK"
        else:
            self.clear()

    def clear(self):
        self.mode = "OFF"
        self.candidates = []
        self.selected = None
        self.frame = None
        self.last_seen_at = None
        self.misses = 0

    def update(self, frame, now):
        if self.mode == "OFF":
            return
        self.thermal_ok = bool(frame is not None and frame["valid_temperature"] and
                               0 <= now - frame["received_at"] <= self.config["max_thermal_age"])
        if self.thermal_ok and frame is not self.frame:
            self.frame = frame
            self.candidates = hot_candidates(frame, self.config)
            if self.mode == "TRACK":
                self._associate(frame["received_at"])
        elif not self.thermal_ok:
            self.candidates = []
        if (self.mode == "TRACK" and self.last_seen_at is not None and
                now - self.last_seen_at > self.config["hot_track_timeout_s"]):
            self.mode, self.selected = "LOST", None

    def _associate(self, received_at):
        if self.selected is None:
            return
        x, y = self.selected["center"]
        prior_area = self.selected["area"]
        nearest = None
        best = float("inf")
        for item in self.candidates:
            dx, dy = item["center"][0] - x, item["center"][1] - y
            distance = (dx * dx + dy * dy) ** .5
            area_ratio = item["area"] / prior_area
            if distance > self.config["hot_track_gate_px"] or not .25 <= area_ratio <= 4:
                continue
            cost = distance + 2 * abs(np.log(area_ratio))
            if cost < best:
                nearest, best = item, cost
        if nearest is None:
            self.misses += 1
            self.selected = None if self.misses > self.config["hot_track_misses"] else self.selected
            if self.selected is None:
                self.mode = "LOST"
        else:
            self.selected = nearest
            self.misses = 0
            self.last_seen_at = received_at

    def select_canvas(self, x, y, alignment):
        """Pick only inside the calibrated thermal footprint on the visible canvas."""
        if self.mode == "OFF" or not self.thermal_ok or not self.candidates:
            return False
        ix, iy = int(x), int(y)
        if not (0 <= ix < alignment.config["width"] and 0 <= iy < alignment.config["height"]):
            return False
        if not alignment.mask[iy, ix]:
            return False
        try:
            inverse = np.linalg.inv(alignment.matrix)
            point = cv2.perspectiveTransform(np.array([[[x, y]]], np.float64), inverse).reshape(2)
        except (ValueError, np.linalg.LinAlgError, cv2.error):
            return False
        px, py = point
        if not (0 <= px < THERMAL_W and 0 <= py < THERMAL_H):
            return False
        nearest, best = None, float("inf")
        for item in self.candidates:
            cx, cy = item["center"]
            distance = ((cx - px) ** 2 + (cy - py) ** 2) ** .5
            bx, by, bw, bh = item["box"]
            inside = bx - 2 <= px < bx + bw + 2 and by - 2 <= py < by + bh + 2
            if (inside or distance <= max(6, .6 * max(bw, bh))) and distance < best:
                nearest, best = item, distance
        if nearest is None:
            return False
        self.selected = nearest
        self.selected_number += 1
        self.mode, self.misses = "TRACK", 0
        self.last_seen_at = self.frame["received_at"]
        return True

    def status(self, show_temperature=True):
        if self.mode == "OFF":
            return ""
        if not self.thermal_ok:
            return "HOT WAIT THERMAL"
        if self.mode == "PICK":
            return "HOT PICK %d" % len(self.candidates)
        if self.mode == "LOST":
            return "HOT LOST - tap another"
        if self.misses:
            return "HOT HOLD %d/%d" % (self.misses, self.config["hot_track_misses"])
        if show_temperature:
            return "HOT #%d max~%.1fC" % (self.selected_number, self.selected["max_c"])
        return "HOT #%d" % self.selected_number

    def draw(self, canvas, alignment, thermal_visible, show_temperature=True):
        if not thermal_visible or not self.thermal_ok or self.mode == "OFF":
            return
        for index, item in enumerate(self.candidates, 1):
            point = cv2.perspectiveTransform(
                np.array([[[item["center"][0], item["center"][1]]]], np.float64),
                alignment.matrix).reshape(2)
            x, y = (int(round(v)) for v in point)
            if not (0 <= x < canvas.shape[1] and 64 <= y < canvas.shape[0] - 28 and
                    alignment.mask[y, x]):
                continue
            is_selected = self.mode == "TRACK" and self.misses == 0 and item is self.selected
            color = (255, 80, 40) if is_selected else (255, 210, 70)
            cv2.drawMarker(canvas, (x, y), color, cv2.MARKER_CROSS, 17 if is_selected else 10, 2)
            label = "%d %.1fC" % (index, item["max_c"]) if show_temperature else str(index)
            draw_text_rgb(canvas, label, x + 7, y - 6, color, .42)
            if is_selected:
                bx, by, bw, bh = item["box"]
                corners = np.array([[[bx, by]], [[bx + bw, by]],
                                    [[bx + bw, by + bh]], [[bx, by + bh]]], np.float64)
                quad = cv2.perspectiveTransform(corners, alignment.matrix).reshape(-1, 2)
                cv2.polylines(canvas, [np.rint(quad).astype(np.int32)], True, color, 2, cv2.LINE_AA)
