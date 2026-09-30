"""Geometry, raw-pixel ROI thermometry and shared screen/RTSP composition."""
import json


def letterbox_geometry(source_w, source_h, output_w, output_h):
    scale = min(output_w / source_w, output_h / source_h)
    width, height = round(source_w * scale), round(source_h * scale)
    return ((output_w - width) // 2, (output_h - height) // 2,
            width, height, width / source_w, height / source_h)


class Alignment:
    def __init__(self, config, source_size, path=None):
        self.config, self.source_size, self.path = config, tuple(source_size), path
        self.rect = list(config["thermal_rect"])
        self.manual = False
        self.homography = None
        self.registration_stats = {}
        self.distances = [float(d) for d in config["calibration_distances_m"]]
        if not self.distances or len(set(self.distances)) != len(self.distances):
            raise ValueError("Calibration distances must be nonempty and unique")
        for d in self.distances:
            distance_key(d)
        self.distance = float(config["calibration_distance_m"])
        if self.distance not in self.distances:
            raise ValueError("Active calibration distance must be a configured slot")
        self.profiles = {}
        self.adjustments = {}
        self.tuning = [1.0, 1.0, 0.0, 0.0]  # scale X/Y, shift as RGB viewport fractions
        self.load_profiles(config.get("calibration_profiles", {}))
        self.load_adjustments(config.get("manual_adjustments", {}))
        if path and os.path.isfile(path):
            try:
                with open(path, encoding="utf-8") as handle:
                    saved = json.load(handle)
                rect = saved["rect"]
                if saved.get("source_size") != list(source_size):
                    raise ValueError("Camera/model resolution changed; realign")
                if saved.get("camera_fps") != config["camera_fps"]:
                    raise ValueError("Camera FPS/mode changed or unknown; recalibrate")
                if saved.get("flips") != [config["thermal_flip_x"], config["thermal_flip_y"]]:
                    raise ValueError("Thermal orientation changed; realign")
                if (len(rect) != 4 or not all(np.isfinite(rect)) or
                        not 0.1 <= rect[2] <= 2.0 or not 0.1 <= rect[3] <= 2.0 or
                        not -1.0 <= rect[0] <= 1.0 or not -1.0 <= rect[1] <= 1.0):
                    raise ValueError("Invalid alignment rectangle")
                self.rect = [float(v) for v in rect]
                self.manual = True
                self.load_profiles(saved.get("profiles", {}))
                self.load_adjustments(saved.get("manual_adjustments", {}))
                if saved.get("active_distance_m") in self.distances:
                    self.distance = float(saved["active_distance_m"])
            except Exception as exc:
                LOG.warning("Alignment ignored: %s", exc)
        self.select_distance(self.distance)

    def load_adjustments(self, adjustments):
        for key, record in adjustments.items():
            try:
                values = record["values"]
                if (record.get("source_size") != list(self.source_size) or
                        record.get("camera_fps") != self.config["camera_fps"] or
                        record.get("flips") != [self.config["thermal_flip_x"], self.config["thermal_flip_y"]] or
                        len(values) != 4 or not np.isfinite(values).all() or
                        not all(.1 <= v <= 4 for v in values[:2]) or
                        not all(-2 <= v <= 2 for v in values[2:])):
                    raise ValueError("Invalid manual scale/shift or changed camera geometry")
                self.adjustments[key] = dict(record, values=[float(v) for v in values])
            except Exception as exc:
                LOG.warning("Ignoring manual adjustment %s: %s", key, exc)

    @property
    def manually_tuned(self):
        return not np.allclose(self.tuning, [1, 1, 0, 0], rtol=0, atol=1e-10)

    def load_profiles(self, profiles):
        for key, profile in profiles.items():
            try:
                if (profile.get("source_size") != list(self.source_size) or
                        profile.get("camera_fps") != self.config["camera_fps"] or
                        profile.get("thermal_size") != [THERMAL_W, THERMAL_H] or
                        profile.get("flips") != [self.config["thermal_flip_x"], self.config["thermal_flip_y"]] or
                        distance_key(profile["distance_m"]) != key):
                    raise ValueError("Profile geometry/distance changed")
                validate_homography(profile["matrix"], self.source_size)
                # Recheck stored independent measurements, not just a claimed PASS flag.
                stats = check_registration(profile["matrix"], profile["check_thermal_points"],
                    profile["check_visible_points"], self.source_size, profile,
                    self.config["calibration_max_error_px"])
                self.profiles[key] = dict(profile, **stats)
            except Exception as exc:
                LOG.warning("Ignoring profile %s: %s", key, exc)

    def select_distance(self, distance):
        if float(distance) not in self.distances:
            raise ValueError("Unknown calibration distance")
        self.distance = float(distance)
        profile = self.profiles.get(distance_key(self.distance))
        self.homography = validate_homography(profile["matrix"], self.source_size) if profile else None
        self.registration_stats = profile or {}
        self.tuning = list(self.adjustments.get(distance_key(self.distance), {}).get("values", [1., 1., 0., 0.]))
        self.rebuild()

    def next_distance(self):
        self.select_distance(self.distances[(self.distances.index(self.distance) + 1) % len(self.distances)])

    def set_registration(self, matrix, stats):
        matrix = validate_homography(matrix, self.source_size)
        checks = check_registration(matrix, stats["check_thermal_points"], stats["check_visible_points"],
                                    self.source_size, stats, self.config["calibration_max_error_px"])
        profile = dict(stats, **checks)
        profile.update(matrix=matrix.tolist(), source_size=list(self.source_size),
                       camera_fps=self.config["camera_fps"],
                       thermal_size=[THERMAL_W, THERMAL_H],
                       flips=[self.config["thermal_flip_x"], self.config["thermal_flip_y"]],
                       distance_m=self.distance)
        self.profiles[distance_key(self.distance)] = profile
        # New measured geometry replaces manual edits of the old geometry.
        self.adjustments.pop(distance_key(self.distance), None)
        self.select_distance(self.distance)

    def rebuild(self):
        self._heat_frame = self._heat_image = None
        c = self.config
        x, y, w, h, sx, sy = letterbox_geometry(*self.source_size, c["width"], c["height"])
        self.viewport = (x, y, w, h, sx, sy)
        rx, ry, rw, rh = self.rect
        flip = np.eye(3, dtype=np.float32)
        if c["thermal_flip_x"]:
            flip[0, 0], flip[0, 2] = -1, THERMAL_W - 1
        if c["thermal_flip_y"]:
            flip[1, 1], flip[1, 2] = -1, THERMAL_H - 1
        target = np.array([[rw * (w - 1) / (THERMAL_W - 1), 0, x + rx * (w - 1)],
                           [0, rh * (h - 1) / (THERMAL_H - 1), y + ry * (h - 1)],
                           [0, 0, 1]], np.float32)
        self.matrix = target @ flip
        if self.homography is not None:
            # H already includes orientation; never apply the thermal flip twice.
            # Match cv2.resize's pixel-centre convention used to draw the RGB image.
            to_canvas = np.array([[sx, 0, x + (sx - 1) / 2],
                                  [0, sy, y + (sy - 1) / 2], [0, 0, 1]], np.float64)
            self.matrix = to_canvas @ self.homography
        self.base_matrix = self.matrix.copy()
        centre = np.array([[[ (THERMAL_W - 1) / 2, (THERMAL_H - 1) / 2 ]]], np.float64)
        cx, cy = cv2.perspectiveTransform(centre, self.base_matrix).reshape(2)
        zx, zy, dx, dy = self.tuning
        post = np.array([[zx, 0, cx * (1 - zx) + dx * w],
                         [0, zy, cy * (1 - zy) + dy * h], [0, 0, 1]], np.float64)
        self.matrix = post @ self.base_matrix
        self.overlay_centre = (cx + dx * w, cy + dy * h)
        yy, xx = np.indices((THERMAL_H, THERMAL_W), dtype=np.float32)
        points = np.dstack((xx, yy)).reshape(-1, 1, 2)
        mapped = cv2.perspectiveTransform(points, self.matrix).reshape(THERMAL_H, THERMAL_W, 2)
        self.mx, self.my = mapped[:, :, 0], mapped[:, :, 1]
        self.visible_samples = ((self.mx >= x) & (self.mx < x + w) &
                                (self.my >= y) & (self.my < y + h))
        mask = cv2.warpPerspective(np.full((THERMAL_H, THERMAL_W), 255, np.uint8),
                                   self.matrix, (c["width"], c["height"]), flags=cv2.INTER_NEAREST)
        viewport_mask = np.zeros_like(mask)
        viewport_mask[y:y + h, x:x + w] = 255
        self.mask = (mask > 0) & (viewport_mask > 0)
        self.outlines = cv2.findContours(self.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    def heat_image(self, frame):
        if frame is not self._heat_frame:
            colored = cv2.applyColorMap(frame["pixels"], getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET))
            colored = cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)
            self._heat_image = cv2.warpPerspective(colored, self.matrix,
                (self.config["width"], self.config["height"]), flags=cv2.INTER_LINEAR)
            self._heat_frame = frame
        return self._heat_image

    def display_box(self, box):
        x, y, w, h, sx, sy = self.viewport
        bx, by, bw, bh = box
        left, top = max(x, x + bx * sx), max(y, y + by * sy)
        right, bottom = min(x + w, x + (bx + bw) * sx), min(y + h, y + (by + bh) * sy)
        return left, top, max(0.0, right - left), max(0.0, bottom - top)

    def adjust(self, action, fine=False):
        self.manual = False
        zoom, step = (1.01, .002) if fine else (1.05, .01)
        values = list(self.tuning)
        if action == "RESET":
            values = [1., 1., 0., 0.]
        elif action in ("NEAR", "FAR"):
            factor = zoom if action == "NEAR" else 1 / zoom
            # Joint limit preserves aspect ratio even when either axis reaches a bound.
            factor = float(np.clip(factor, max(.1 / v for v in values[:2]), min(4 / v for v in values[:2])))
            values[0] *= factor
            values[1] *= factor
        elif action in ("W-", "W+", "H-", "H+"):
            index = 0 if action[0] == "W" else 1
            values[index] = float(np.clip(values[index] * (zoom if action[1] == "+" else 1 / zoom), .1, 4))
        else:
            index, direction = {"LEFT": (2, -1), "RIGHT": (2, 1), "UP": (3, -1), "DOWN": (3, 1)}[action]
            values[index] = float(np.clip(values[index] + step * direction, -2, 2))
        self.tuning = values
        self.adjustments[distance_key(self.distance)] = {
            "values": values, "source_size": list(self.source_size),
            "camera_fps": self.config["camera_fps"],
            "flips": [self.config["thermal_flip_x"], self.config["thermal_flip_y"]]}
        self.rebuild()

    def save(self):
        if not self.path:
            raise RuntimeError("No persistent alignment path")
        saved = {"schema": 4, "rect": self.rect, "source_size": list(self.source_size),
                 "camera_fps": self.config["camera_fps"],
                 "manual_adjustments": self.adjustments,
                 "profiles": self.profiles, "active_distance_m": self.distance,
                 "flips": [self.config["thermal_flip_x"], self.config["thermal_flip_y"]]}
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        with open(self.path + ".tmp", "w", encoding="utf-8") as handle:
            json.dump(saved, handle, indent=2)
        os.replace(self.path + ".tmp", self.path)
        self.manual = True
        LOG.info("Saved alignment at %s; CONFIG thermal_rect = %s", self.path, self.rect)
        if self.profiles:
            print("CONFIG['calibration_profiles'] = " + repr(self.profiles), flush=True)
        if self.adjustments:
            print("CONFIG['manual_adjustments'] = " + repr(self.adjustments), flush=True)


def thermal_state(frame, visible_at, now, config):
    if frame is None:
        return "NO THERMAL", False, False
    age = now - frame["received_at"]
    if age < 0 or age > config["max_thermal_age"]:
        return "THERMAL STALE", False, False
    if abs(visible_at - frame["received_at"]) > config["max_pair_delta"]:
        return "THERMAL DESYNC", False, False
    if not frame["valid_temperature"]:
        return "UNCALIBRATED", True, False
    return "THERMAL OK", True, True


def roi_temperature(frame, box, alignment):
    """Sample original thermal pixels, never interpolated/colorized pixels."""
    if frame is None or not frame["valid_temperature"]:
        return None
    x, y, w, h = box
    if w <= 0 or h <= 0:
        return None
    mask = (alignment.visible_samples & (alignment.mx >= x) & (alignment.mx < x + w) &
            (alignment.my >= y) & (alignment.my < y + h))
    values = frame["pixels"][mask]
    if values.size < 1:
        return None
    temps = frame["lo"] / 10.0 + (frame["hi"] - frame["lo"]) / 10.0 * np.minimum(values.astype(np.float32) / 254, 1.0)
    return {"max": float(temps.max()), "mean": float(temps.mean()), "samples": int(values.size)}


def draw_text_rgb(canvas, text, x, y, color=(255, 255, 255), scale=.5):
    # ASCII UI avoids an external font dependency; array channels are RGB.
    x, y = int(x), int(y)
    cv2.putText(canvas, str(text), (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(canvas, str(text), (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def compose(rgb, detections, thermal, alignment, now, visible_at, mode="FUSION", alpha=.4,
            show_temperature=True, show_overlay=True):
    c = alignment.config
    out = np.zeros((c["height"], c["width"], 3), np.uint8)
    x, y, w, h, _, _ = alignment.viewport
    out[y:y + h, x:x + w] = cv2.resize(rgb, (w, h))
    state, show_heat, use_temp = thermal_state(thermal, visible_at, now, c)
    effective_mode = mode if show_overlay else "VISIBLE"
    if effective_mode == "THERMAL":
        out[:] = 15
    if show_heat and effective_mode != "VISIBLE":
        heat = alignment.heat_image(thermal)
        blend = heat if effective_mode == "THERMAL" else cv2.addWeighted(out, 1 - alpha, heat, alpha, 0)
        out[alignment.mask] = blend[alignment.mask]
    results = []
    for det in detections[:c["max_objects"]]:
        box = alignment.display_box(det["box"])
        bx, by, bw, bh = box
        if bw <= 0 or bh <= 0:
            continue
        # Async boxes belong to an older RGB frame. Do not assign temperatures when timestamps diverge.
        pair_ok = thermal is not None and abs(det.get("captured_at", visible_at) - thermal["received_at"]) <= c["max_detection_thermal_delta"]
        result = roi_temperature(thermal, box, alignment) if show_temperature and use_temp and pair_ok else None
        results.append(result)
        cv2.rectangle(out, (int(bx), int(by)), (int(bx + bw - 1), int(by + bh - 1)), (80, 255, 80), 2)
        label = "%s %.0f%%" % (det["label"], det["score"] * 100)
        if not show_temperature:
            pass
        elif result:
            label += " ROI max ~%.1fC" % result["max"]
        else:
            label += " T:--"
        tx = int(np.clip(bx, 0, max(0, c["width"] - len(label) * 8)))
        ty = int(np.clip(by - 6, 66, c["height"] - 38))
        draw_text_rgb(out, label, tx, ty, (130, 255, 130), .45)
    global_stats = roi_temperature(thermal, (x, y, w, h), alignment) if show_temperature and use_temp else None
    detail = ("max ~%.1fC" % global_stats["max"] if global_stats else "T:--") if show_temperature else "TEMP OFF"
    if alignment.manually_tuned:
        align_text = distance_key(alignment.distance) + " MANUAL TUNE"
    elif alignment.homography is not None:
        align_text = "%s check %.1fpx" % (distance_key(alignment.distance), alignment.registration_stats["check_max_px"])
    else:
        align_text = distance_key(alignment.distance) + " NO POINT CAL"
    draw_text_rgb(out, "%s | %s | %s" % (state, detail, align_text), 8, 83, (255, 220, 80), .48)
    return out, results, state


class TouchUI:
    def __init__(self, alignment, hot_tracker=None):
        self.alignment = alignment
        self.hot_tracker = hot_tracker
        self.mode, self.alpha = "FUSION", alignment.config["alpha"]
        self.show_temperatures, self.show_heat = True, True
        self.calibrating, self.was_pressed = False, False
        self.message, self.message_until = "", 0.0
        self.registration = CalibrationSession(alignment)
        self.fine = False

    def buttons(self):
        width, height = self.alignment.config["width"], self.alignment.config["height"]
        labels = ["EXIT", "VIEW", "ALPHA", "DIST", "CAL", "ALIGN", "SAVE", "TEMP", "HEAT"]
        if self.hot_tracker is not None:
            labels.append("HOT")
        # Two generous rows keep all actions usable on the 480px-wide portrait display.
        columns = 5
        buttons = [(name, ((i % columns) * width // columns, (i // columns) * 32,
                           width // columns, 31)) for i, name in enumerate(labels)]
        if self.calibrating:
            labels = ["FAR", "NEAR", "LEFT", "RIGHT", "UP", "DOWN", "W-", "W+", "H-", "H+", "FINE", "RESET"]
            for i, name in enumerate(labels):
                buttons.append((name, ((i % 6) * width // 6, height - 106 + (i // 6) * 38, width // 6, 36)))
        return buttons

    def touch(self, x, y, pressed):
        clicked = pressed and not self.was_pressed
        self.was_pressed = pressed
        if not clicked:
            return False
        if self.registration.active:
            self.registration.click(x, y)
            if not self.registration.active:
                self.message = self.registration.message
                self.message_until = time.monotonic() + 5
            return False
        for name, (bx, by, bw, bh) in self.buttons():
            if not (bx <= x < bx + bw and by <= y < by + bh):
                continue
            if name == "EXIT":
                return True
            if name == "VIEW":
                modes = ["FUSION", "VISIBLE", "THERMAL"]
                self.mode = modes[(modes.index(self.mode) + 1) % len(modes)]
            elif name == "ALPHA":
                self.alpha = round((self.alpha + .2) % 1.01, 2)
            elif name == "TEMP":
                self.show_temperatures = not self.show_temperatures
            elif name == "HEAT":
                self.show_heat = not self.show_heat
            elif name == "ALIGN":
                self.calibrating = not self.calibrating
            elif name == "DIST":
                self.alignment.next_distance()
            elif name == "CAL":
                self.calibrating = False
                if self.hot_tracker is not None:
                    self.hot_tracker.clear()
                self.registration.begin()
            elif name == "HOT":
                self.calibrating = False
                self.hot_tracker.toggle()
                self.message = "Tap one numbered hot region" if self.hot_tracker.mode == "PICK" else "Hot tracking off"
                self.message_until = time.monotonic() + 2
            elif name == "SAVE":
                try:
                    self.alignment.save()
                    self.message = "Saved distance profiles and manual scale/shift"
                except Exception as exc:
                    LOG.warning("Save alignment: %s", exc)
                    self.message = "Save failed; see MaixVision console"
                self.message_until = time.monotonic() + 3
            elif name == "FINE":
                self.fine = not self.fine
            else:
                try:
                    self.alignment.adjust(name, self.fine)
                except ValueError as exc:
                    self.message = str(exc)
                    self.message_until = time.monotonic() + 4
            break
        else:
            if (not self.calibrating and self.hot_tracker is not None and
                    self.hot_tracker.select_canvas(x, y, self.alignment)):
                self.message = "Tracking selected hot region"
                self.message_until = time.monotonic() + 2
        return False

    def draw(self, canvas, status):
        if self.hot_tracker is not None and self.hot_tracker.mode != "OFF" and not self.calibrating:
            draw_text_rgb(canvas, self.hot_tracker.status(self.show_temperatures), 8, 105, (255, 155, 75), .48)
        if self.calibrating:
            a = self.alignment
            cv2.drawContours(canvas, a.outlines, -1, (60, 235, 255), 1, cv2.LINE_AA)
            cx, cy = (int(round(v)) for v in a.overlay_centre)
            x, y, w, h, _, _ = a.viewport
            if x <= cx < x + w and y <= cy < y + h:
                cv2.drawMarker(canvas, (cx, cy), (60, 235, 255), cv2.MARKER_CROSS, 17, 1)
            zx, zy, dx, dy = a.tuning
            info = "FOV scale %.2fx / %.2fx | %s | FAR shrink / NEAR enlarge" % (zx, zy, "FINE" if self.fine else "COARSE")
            draw_text_rgb(canvas, info, 8, 105, (60, 235, 255), .43)
        for name, (x, y, w, h) in self.buttons():
            active = ((name == "TEMP" and self.show_temperatures) or
                      (name == "HEAT" and self.show_heat) or
                      (name == "HOT" and self.hot_tracker is not None and self.hot_tracker.mode != "OFF"))
            color = (35, 88, 65) if active else (24, 30, 40)
            cv2.rectangle(canvas, (x, y), (x + w - 2, y + h - 2), color, -1)
            draw_text_rgb(canvas, name, x + 8, y + 22, scale=.48)
        h = canvas.shape[0]
        cv2.rectangle(canvas, (0, h - 28), (canvas.shape[1], h), (20, 24, 30), -1)
        msg = self.message if time.monotonic() < self.message_until else status
        draw_text_rgb(canvas, msg, 8, h - 9, (180, 225, 245), .44)
