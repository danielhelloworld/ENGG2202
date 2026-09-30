"""Planar cross-spectrum registration and full-screen paired-point calibration.

Matrices map RAW (unflipped) thermal pixels to RGB camera-input pixels.
Each distance is measured independently. No depth estimation or matrix interpolation.
"""


def distance_key(distance):
    value = float(distance)
    if not np.isfinite(value) or value <= 0:
        raise ValueError("Calibration distance must be positive metres")
    return "%gm" % value


def validate_homography(matrix, source_size):
    h = np.asarray(matrix, np.float64)
    if h.shape != (3, 3) or not np.isfinite(h).all() or abs(h[2, 2]) < 1e-9:
        raise ValueError("Invalid registration matrix")
    h = h / h[2, 2]
    if abs(np.linalg.det(h)) < 1e-9 or np.linalg.cond(h) > 1e9:
        raise ValueError("Singular registration matrix")
    corners = np.array([[0, 0], [159, 0], [159, 119], [0, 119]], np.float64)
    denom = np.c_[corners, np.ones(4)] @ h[2]
    if np.min(denom) <= 1e-5:
        raise ValueError("Perspective horizon crosses the thermal image")
    mapped = cv2.perspectiveTransform(corners.reshape(-1, 1, 2), h).reshape(-1, 2)
    if np.any(np.abs(mapped) > max(source_size) * 20):
        raise ValueError("Registration extrapolates too far")
    area = abs(cv2.contourArea(mapped.astype(np.float32)))
    if area < 4 or area > np.prod(source_size) * 100:
        raise ValueError("Invalid projected thermal area")
    return h


def calibration_points(points, size, minimum):
    pts = np.asarray(points, np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < minimum or not np.isfinite(pts).all():
        raise ValueError("Need at least %d complete finite point pairs" % minimum)
    if np.any(pts < 0) or np.any(pts > np.asarray(size) - 1):
        raise ValueError("Calibration point outside its source image")
    if len(np.unique(pts, axis=0)) != len(pts):
        raise ValueError("Repeated point: choose distinct markers")
    return pts


def point_errors(matrix, thermal_points, visible_points):
    predicted = cv2.perspectiveTransform(np.asarray(thermal_points, np.float64).reshape(-1, 1, 2), matrix).reshape(-1, 2)
    return np.linalg.norm(predicted - np.asarray(visible_points), axis=1)


def fit_registration(thermal_points, visible_points, source_size, threshold=4.0):
    src = calibration_points(thermal_points, (THERMAL_W, THERMAL_H), 6)
    dst = calibration_points(visible_points, source_size, 6)
    if len(src) != len(dst):
        raise ValueError("Point-pair counts differ")
    def coverage(points, size):
        return abs(cv2.contourArea(cv2.convexHull(points.astype(np.float32)))) / np.prod(size)
    if coverage(src, (THERMAL_W, THERMAL_H)) < .03 or coverage(dst, source_size) < .02:
        raise ValueError("Spread markers across the shared field; points too narrow/collinear")
    matrix, mask = cv2.findHomography(src, dst, cv2.RANSAC, float(threshold))
    if matrix is None or mask is None:
        raise ValueError("Cannot solve registration")
    inliers = mask.ravel().astype(bool)
    if inliers.sum() < max(6, int(np.ceil(len(src) * .75))):
        raise ValueError("Too many mismatched markers; undo and repick")
    if coverage(src[inliers], (THERMAL_W, THERMAL_H)) < .03 or coverage(dst[inliers], source_size) < .02:
        raise ValueError("Valid pairs do not cover enough of the shared field")
    matrix = validate_homography(matrix, source_size)
    errors = point_errors(matrix, src, dst)
    return matrix, {"pairs": len(src), "inliers": int(inliers.sum()),
                    "fit_rmse_px": float(np.sqrt(np.mean(errors[inliers] ** 2))),
                    "rejected_indices": np.flatnonzero(~inliers).tolist(),
                    "thermal_points": src.tolist(), "visible_points": dst.tolist()}


def check_registration(matrix, thermal_points, visible_points, source_size, fit, maximum=6.0):
    src = calibration_points(thermal_points, (THERMAL_W, THERMAL_H), 2)
    dst = calibration_points(visible_points, source_size, 2)
    if len(src) != len(dst):
        raise ValueError("Check-pair counts differ")
    for new, old, separation in ((src, fit["thermal_points"], 2), (dst, fit["visible_points"], 3)):
        distances = np.linalg.norm(new[:, None, :] - np.asarray(old)[None, :, :], axis=2)
        if distances.min() < separation:
            raise ValueError("Check on NEW markers, not the fitting markers")
    if np.max(np.linalg.norm(src - src[0], axis=1)) < 10:
        raise ValueError("Spread the independent check markers apart")
    errors = point_errors(validate_homography(matrix, source_size), src, dst)
    if errors.max() > maximum:
        raise ValueError("Check error %.1fpx > %.1fpx; repick/recalibrate" % (errors.max(), maximum))
    return {"check_pairs": len(src), "check_rmse_px": float(np.sqrt(np.mean(errors ** 2))),
            "check_max_px": float(errors.max()), "check_thermal_points": src.tolist(),
            "check_visible_points": dst.tolist()}


class CalibrationSession:
    def __init__(self, alignment):
        self.alignment = alignment
        self.active = False
        self.latest = None
        self.reset()

    def reset(self):
        self.frozen = None
        self.pending = None
        self.thermal_points, self.visible_points = [], []
        self.check_thermal, self.check_visible = [], []
        self.candidate, self.fit = None, None
        self.phase = "FIT"
        self.message = "Keep a flat target still at selected distance; FREEZE"

    def begin(self):
        self.reset()
        self.active = True

    def observe(self, rgb, thermal, visible_at, now):
        self.latest = (rgb, thermal, visible_at)

    def freeze(self):
        if self.latest is None:
            raise ValueError("Wait for both cameras")
        rgb, thermal, visible_at = self.latest
        state, show, _ = thermal_state(thermal, visible_at, time.monotonic(), self.alignment.config)
        if not show:
            raise ValueError("Cannot freeze: " + state)
        frozen = (rgb.copy(), dict(thermal, pixels=thermal["pixels"].copy()))
        self.reset()
        self.frozen = frozen
        self.message = "Tap RGB marker, then SAME marker in thermal; repeat 6-12 pairs"

    def controls(self):
        width = self.alignment.config["width"]
        names = ["CANCEL", "FREEZE", "UNDO", "FIT", "APPLY"]
        return [(name, (i * width // 5, 0, width // 5, 34)) for i, name in enumerate(names)]

    def panel(self, thermal_view):
        c = self.alignment.config
        sw, sh = (THERMAL_W, THERMAL_H) if thermal_view else self.alignment.source_size
        x, y, w, h, _, _ = letterbox_geometry(sw, sh, c["width"], c["height"] - 140)
        return x, y + 84, w, h, sw, sh

    def panel_point(self, x, y, thermal_view):
        bx, by, w, h, sw, sh = self.panel(thermal_view)
        if not (bx <= x < bx + w and by <= y < by + h):
            return None
        px = float(np.clip((x - bx + .5) * sw / w - .5, 0, sw - 1))
        py = float(np.clip((y - by + .5) * sh / h - .5, 0, sh - 1))
        c = self.alignment.config
        if thermal_view:
            px = sw - 1 - px if c["thermal_flip_x"] else px
            py = sh - 1 - py if c["thermal_flip_y"] else py
        return [float(px), float(py)]

    def screen_point(self, point, thermal_view):
        bx, by, w, h, sw, sh = self.panel(thermal_view)
        px, py = point
        c = self.alignment.config
        if thermal_view:
            px = sw - 1 - px if c["thermal_flip_x"] else px
            py = sh - 1 - py if c["thermal_flip_y"] else py
        return int(round(bx + (px + .5) * w / sw - .5)), int(round(by + (py + .5) * h / sh - .5))

    def action(self, name):
        if name == "CANCEL":
            self.active = False
        elif name == "FREEZE":
            self.freeze()
        elif name == "UNDO":
            src, dst = self.current_points()
            if self.pending is not None:
                self.pending = None
            elif src:
                src.pop()
                dst.pop()
            self.message = "Last selection undone"
        elif name == "FIT":
            if self.pending is not None:
                raise ValueError("Finish or UNDO the current pair")
            self.candidate, self.fit = fit_registration(self.thermal_points, self.visible_points,
                self.alignment.source_size, self.alignment.config["calibration_ransac_px"])
            self.phase = "CHECK"
            self.check_thermal, self.check_visible = [], []
            self.message = "Fit %.1fpx; select 2+ NEW check pairs, then APPLY" % self.fit["fit_rmse_px"]
        elif name == "APPLY":
            if self.candidate is None or self.pending is not None:
                raise ValueError("FIT first, then finish 2+ independent check pairs")
            stats = check_registration(self.candidate, self.check_thermal, self.check_visible,
                self.alignment.source_size, self.fit, self.alignment.config["calibration_max_error_px"])
            self.alignment.set_registration(self.candidate, dict(self.fit, **stats))
            self.active = False
            # Applying is live; SAVE persists all distance profiles separately.
            self.message = "Applied at %s; press SAVE to persist" % distance_key(self.alignment.distance)

    def current_points(self):
        return (self.thermal_points, self.visible_points) if self.phase == "FIT" else (self.check_thermal, self.check_visible)

    def click(self, x, y):
        try:
            for name, (bx, by, w, h) in self.controls():
                if bx <= x < bx + w and by <= y < by + h:
                    self.action(name)
                    return
            if self.frozen is None:
                return
            thermal_view = self.pending is not None
            point = self.panel_point(x, y, thermal_view)
            if point is None:
                return
            if not thermal_view:
                self.pending = point
            else:
                src, dst = self.current_points()
                if len(src) >= 24:
                    raise ValueError("Maximum 24 pairs; FIT or APPLY")
                src.append(point)
                dst.append(self.pending)
                self.pending = None
            count = len(self.current_points()[0])
            self.message = "%s pairs: %d; tap %s" % (self.phase, count, "THERMAL" if self.pending else "RGB")
        except Exception as exc:
            self.message = str(exc)
            LOG.warning("Calibration: %s", exc)

    def render(self):
        c = self.alignment.config
        canvas = np.full((c["height"], c["width"], 3), 18, np.uint8)
        thermal_view = self.pending is not None
        data = self.frozen or (self.latest[:2] if self.latest else None)
        if data is not None:
            rgb, thermal = data
            if thermal_view and thermal is not None:
                pixels = thermal["pixels"]
                if c["thermal_flip_x"]:
                    pixels = pixels[:, ::-1]
                if c["thermal_flip_y"]:
                    pixels = pixels[::-1, :]
                pixels = cv2.cvtColor(cv2.applyColorMap(np.ascontiguousarray(pixels), cv2.COLORMAP_JET), cv2.COLOR_BGR2RGB)
            else:
                pixels = rgb
            x, y, w, h, _, _ = self.panel(thermal_view)
            canvas[y:y + h, x:x + w] = cv2.resize(pixels, (w, h))
            src, dst = self.current_points()
            for i, point in enumerate(src if thermal_view else dst):
                px, py = self.screen_point(point, thermal_view)
                cv2.drawMarker(canvas, (px, py), (80, 255, 80), cv2.MARKER_CROSS, 10, 1)
                draw_text_rgb(canvas, str(i + 1), px + 5, py - 5, scale=.4)
        for name, (x, y, w, h) in self.controls():
            cv2.rectangle(canvas, (x, y), (x + w - 2, y + h - 2), (35, 40, 55), -1)
            draw_text_rgb(canvas, name, x + 6, y + 23, scale=.48)
        title = "%s | %s | %s | %s" % (distance_key(self.alignment.distance),
            "THERMAL" if thermal_view else "RGB", self.phase, "FROZEN" if self.frozen else "LIVE")
        draw_text_rgb(canvas, title, 8, 57, (255, 220, 80), .55)
        # Split long messages so failures remain readable at the default 640px output.
        draw_text_rgb(canvas, self.message[:78], 6, c["height"] - 30, scale=.42)
        draw_text_rgb(canvas, self.message[78:156], 6, c["height"] - 10, scale=.42)
        return canvas
