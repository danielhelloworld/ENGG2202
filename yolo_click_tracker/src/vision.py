"""Built-in NPU YOLO, same-ID tracking, and timestamped double buffering."""
from collections import deque


class NativeVision:
    def __init__(self, c):
        from maix import nn, tracker
        self.c, self.api = c, tracker
        if c["model_kind"] not in ("YOLO11", "YOLOv8"):
            raise ValueError("Choose YOLO11 or YOLOv8 to match the .mud model")
        self.model = getattr(nn, c["model_kind"])(model=c["model_path"], dual_buff=c["dual_buff"])
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


class DetectionPipeline:
    """dual_buff results belong to the previous submitted image, not this one."""
    def __init__(self, vision, c, clock=time.monotonic):
        self.vision, self.c, self.clock = vision, c, clock
        self.pending = None
        self.completed = deque(maxlen=30)
        self.api_ms = self.latency_ms = None

    def push(self, img, stamp, pose):
        submitted = (img, stamp, pose)
        started = self.clock()
        tracks = self.vision.detect(img)
        finished = self.clock()
        self.api_ms = (finished-started)*1000
        if self.c["dual_buff"]:
            result = self.pending
            self.pending = submitted
        else:
            result = submitted
        if result is None:
            return None  # Warm-up is not a valid detection or control heartbeat.
        frame, captured, capture_pose = result
        self.completed.append(finished)
        self.latency_ms = max(0., finished-captured)*1000
        return frame, tracks, captured, capture_pose

    def stats(self):
        fps = 0.
        if len(self.completed) > 1 and self.completed[-1] > self.completed[0]:
            fps = (len(self.completed)-1)/(self.completed[-1]-self.completed[0])
        return dict(inference_fps_actual=round(fps, 1), detect_api_ms=self.api_ms,
                    capture_to_result_ms=self.latency_ms, dual_buff=self.c["dual_buff"],
                    model_path=self.c["model_path"], model_kind=self.c["model_kind"])

    def close(self):
        self.pending = None


def configure_camera_rotation(cam, c):
    """Use sensor/ISP mirroring if supported; otherwise restore and rotate in CPU."""
    if not c["rotate_180"] or not c["camera_rotate"]:
        return False
    try:
        horizontal, vertical = cam.hmirror(), cam.vflip()
        if horizontal not in (0, 1) or vertical not in (0, 1):
            raise RuntimeError("Camera flip state is unavailable")
    except Exception as exc:
        print("CAMERA_ROTATION CPU fallback:", str(exc), flush=True)
        return False
    try:
        cam.hmirror(1-horizontal)
        cam.vflip(1-vertical)
        if (cam.hmirror(), cam.vflip()) != (1-horizontal, 1-vertical):
            raise RuntimeError("Camera flip setting not acknowledged")
    except Exception as exc:
        # Never combine a half-applied camera flip with another 180-degree rotation.
        try:
            cam.hmirror(horizontal)
            cam.vflip(vertical)
            if (cam.hmirror(), cam.vflip()) != (horizontal, vertical):
                raise RuntimeError("Camera flip restoration failed")
        except Exception as restore:
            raise RuntimeError("Camera orientation is uncertain; motor startup blocked") from restore
        print("CAMERA_ROTATION CPU fallback:", str(exc), flush=True)
        return False
    print("CAMERA_ROTATION camera hmirror+vflip (no CPU image rotation)", flush=True)
    return True


