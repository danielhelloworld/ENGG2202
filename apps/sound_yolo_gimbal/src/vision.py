"""Native Maix YOLO + per-class ByteTrack (mainweb behavioral port)."""


class NativeVision:
    def __init__(self, c):
        from maix import nn, tracker
        self.c, self.api = c, tracker
        if c["model_kind"] not in ("YOLO11", "YOLOv8"):
            raise ValueError("Choose YOLO11 or YOLOv8 to match the .mud model")
        self.model = getattr(nn, c["model_kind"])(model=c["model_path"], dual_buff=False)
        self.trackers = {}

    def detect(self, img):
        objects = self.model.detect(img, conf_th=self.c["confidence"], iou_th=self.c["iou"])
        grouped = {}
        for obj in objects:
            cid = int(obj.class_id)
            if self.c["class_ids"] is not None and cid not in self.c["class_ids"]:
                continue
            grouped.setdefault(cid, []).append(self.api.Object(obj.x, obj.y, obj.w, obj.h, cid, obj.score))
        for cid in grouped:
            if cid not in self.trackers:
                # Lost buffer is frames; actual control timeout below is wall-clock 2s.
                self.trackers[cid] = self.api.ByteTracker(max(1, int(self.c["inference_fps"] * self.c["lost_timeout"])), .5, .5, .8, 60)
        output = []
        width, height = img.width(), img.height()
        for cid, tracker_instance in self.trackers.items():
            for track in tracker_instance.update(grouped.get(cid, [])):
                if track.lost or not track.history:
                    continue
                obj = track.history[-1]
                output.append({"key": (cid, int(track.id)), "box": (obj.x, obj.y, obj.w, obj.h),
                               "cx": (obj.x + obj.w/2) / width, "cy": (obj.y + obj.h/2) / height,
                               "score": float(track.score), "label": self.model.labels[cid],
                               "trail": [(o.x + o.w/2, o.y + o.h/2) for o in track.history]})
        return output


def touch_to_image(x, y, iw, ih, dw, dh):
    scale = min(dw / iw, dh / ih)
    return ((x - (dw - iw*scale)/2) / scale, (y - (dh - ih*scale)/2) / scale)


def draw_tracking(img, tracks, controller, sound, motor, fps):
    from maix import image
    green = image.Color.from_rgb(60, 240, 110)
    yellow = image.Color.from_rgb(255, 210, 50)
    white = image.Color.from_rgb(235, 240, 250)
    for tr in tracks:
        selected = tr["key"] == controller.selected
        color = yellow if selected else green
        x, y, w, h = map(int, tr["box"])
        img.draw_rect(x, y, w, h, color, 2)
        img.draw_string(x, max(42, y-18), "%s %d:%d %.2f" % (tr["label"], *tr["key"], tr["score"]), color)
        if selected:
            for px, py in tr["trail"]:
                img.draw_circle(int(px), int(py), 2, yellow, -1)
            img.draw_line(int(tr["cx"]*img.width()), int(tr["cy"]*img.height()), img.width()//2, img.height()//2, yellow, 2)
    cx, cy = img.width()//2, img.height()//2
    img.draw_line(cx-12, cy, cx+12, cy, white, 1)
    img.draw_line(cx, cy-12, cx, cy+12, white, 1)
    img.draw_rect(0, 0, img.width(), 38, image.COLOR_BLACK, -1)
    img.draw_string(4, 3, "STOP   CLEAR   %s   %s" % ("DISARM" if motor["armed"] else "ARM", controller.state), yellow)
    img.draw_string(4, 20, "%s Y%.1f P%.1f %.1ffps" % ("SIM" if motor["simulated"] else "MOTOR", *motor["angles"], fps), white)
    img.draw_rect(0, img.height()-36, img.width(), 36, image.COLOR_BLACK, -1)
    vector = sound.get("vector_xy")
    mic_text = "MIC %s Q%.2f" % (str(vector and [round(v, 2) for v in vector]), sound.get("quality", 0))
    img.draw_string(4, img.height()-34, mic_text, white)
    img.draw_string(4, img.height()-17, controller.reason[:80], yellow)
