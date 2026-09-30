"""YOLO process isolation: bounded latest-frame mailbox, private socket IPC.

Avoid relying on native inference to release the Python GIL. No network port,
pickle deserialization, input FIFO, or shared camera/model object between processes.
"""
import subprocess
import sys


def ipc_send(sock, header, payload=b""):
    header = dict(header, payload_bytes=len(payload))
    encoded = json.dumps(header).encode("utf-8")
    if len(encoded) > 65536 or len(payload) > 32 * 1024 * 1024:
        raise ValueError("IPC message exceeds bound")
    sock.sendall(struct.pack(">I", len(encoded)) + encoded)
    if payload:
        sock.sendall(payload)


def ipc_exact(sock, length):
    data = bytearray()
    while len(data) < length:
        chunk = sock.recv(min(length - len(data), 1024 * 1024))
        if not chunk:
            raise EOFError("YOLO worker connection closed")
        data.extend(chunk)
    return bytes(data)


def ipc_receive(sock):
    length = struct.unpack(">I", ipc_exact(sock, 4))[0]
    if not 1 <= length <= 65536:
        raise ValueError("Invalid IPC header size")
    header = json.loads(ipc_exact(sock, length))
    size = header.get("payload_bytes", 0)
    if not isinstance(size, int) or not 0 <= size <= 32 * 1024 * 1024:
        raise ValueError("Invalid IPC payload size")
    return header, ipc_exact(sock, size)


def run_yolo_worker(fd, model_path, kind):
    """Only this process initializes/uses the NPU model. RGB capture stays in parent."""
    channel = socket.socket(fileno=fd)
    detector = None
    try:
        from maix import image, nn
        detector = getattr(nn, kind)(model=model_path, dual_buff=False)
        width, height = detector.input_width(), detector.input_height()
        ipc_send(channel, {"event": "ready", "width": width, "height": height})
        while True:
            header, data = ipc_receive(channel)
            if header.get("event") == "stop":
                break
            if header.get("event") != "frame" or len(data) != width * height * 3:
                raise ValueError("Invalid inference frame")
            rgb = np.frombuffer(data, np.uint8).reshape(height, width, 3)
            img = image.cv2image(rgb, bgr=False, copy=True)
            if img.format() != detector.input_format():
                img = img.to_format(detector.input_format())
            started = time.monotonic()
            objects = detector.detect(img, conf_th=header["confidence"], iou_th=header["iou"])
            detections = []
            for obj in sorted(objects, key=lambda item: item.score, reverse=True)[:header["max_objects"]]:
                label = str(detector.labels[obj.class_id]).encode("ascii", "replace").decode("ascii")[:24]
                detections.append({"box": [obj.x, obj.y, obj.w, obj.h], "score": float(obj.score),
                                   "label": label, "captured_at": header["captured_at"]})
            ipc_send(channel, {"event": "result", "captured_at": header["captured_at"],
                               "inference_ms": (time.monotonic() - started) * 1000,
                               "detections": detections})
    except EOFError:
        pass
    except Exception as exc:
        try:
            ipc_send(channel, {"event": "error", "error": str(exc)})
        except OSError:
            pass
        raise
    finally:
        detector = None
        channel.close()


class InlineDetector:
    """One Maix hardware process; scheduled inference still blocks the calling loop."""
    def __init__(self, config, model, image_module):
        self.config, self.model, self.image = config, model, image_module
        self.enabled, self.latest, self.error = True, None, ""
        self.completed = self.replaced = 0
        self.last_started = None

    @classmethod
    def launch(cls, config, model_path, kind):
        from maix import image, nn
        model = getattr(nn, kind)(model=model_path, dual_buff=False)
        return cls(config, model, image), (model.input_width(), model.input_height())

    def set_enabled(self, enabled):
        if self.enabled != enabled:
            self.enabled = enabled
            self.latest = None
            self.last_started = None

    def submit(self, rgb, captured_at):
        if not self.enabled or self.error:
            return
        now = time.monotonic()
        cap = self.config["inference_fps"]
        if cap and self.last_started is not None and now - self.last_started < 1 / cap:
            return
        self.last_started = now
        try:
            img = self.image.cv2image(rgb, bgr=False, copy=True)
            if img.format() != self.model.input_format():
                img = img.to_format(self.model.input_format())
            started = time.monotonic()
            objects = self.model.detect(img, conf_th=self.config["confidence"], iou_th=self.config["iou"])
            detections = []
            for obj in sorted(objects, key=lambda item: item.score, reverse=True)[:self.config["max_objects"]]:
                label = str(self.model.labels[obj.class_id]).encode("ascii", "replace").decode("ascii")[:24]
                detections.append({"box": [obj.x, obj.y, obj.w, obj.h], "score": float(obj.score),
                                   "label": label, "captured_at": captured_at})
            self.latest = {"captured_at": captured_at, "detections": detections,
                           "inference_ms": (time.monotonic() - started) * 1000}
            self.completed += 1
        except Exception as exc:
            self.error, self.latest = str(exc), None
            LOG.exception("Inline YOLO failed; RGB/thermal preview continues")

    def snapshot(self, now):
        result = self.latest
        stats = {"completed": self.completed, "replaced": 0, "error": self.error,
                 "age": None, "inference_ms": None}
        if result is None or not self.enabled or self.error:
            return [], stats
        age = now - result["captured_at"]
        stats.update(age=age, inference_ms=result["inference_ms"])
        return (result["detections"] if 0 <= age <= self.config["max_detection_age"] else []), stats

    def close(self):
        self.enabled, self.latest, self.model = False, None, None


class AsyncDetector:
    def __init__(self, config, channel, process=None):
        self.config, self.channel, self.process = config, channel, process
        self.condition = threading.Condition()
        self.stopped = False
        self.pending = None
        self.generation = 0
        self.enabled = True
        self.latest = None
        self.completed = self.replaced = 0
        self.error = ""
        self.thread = threading.Thread(target=self._work, name="yolo-ipc", daemon=True)
        self.thread.start()

    @classmethod
    def launch(cls, config, entry_path, model_path, kind):
        if os.name != "posix":
            raise RuntimeError("NPU process mode runs on the Linux board")
        parent, child = socket.socketpair()
        process = None
        try:
            # pass_fds creates a private inherited channel, not an exposed server.
            process = subprocess.Popen([sys.executable, "-u", entry_path, "--yolo-worker",
                str(child.fileno()), model_path, kind], pass_fds=(child.fileno(),), close_fds=True)
            child.close()
            parent.settimeout(120)
            ready, _ = ipc_receive(parent)
            if ready.get("event") != "ready":
                raise RuntimeError("YOLO startup failed: " + str(ready))
            width, height = int(ready["width"]), int(ready["height"])
            if not 1 <= width <= 4096 or not 1 <= height <= 4096 or width * height * 3 > 32 * 1024 * 1024:
                raise ValueError("Invalid model input size")
            parent.settimeout(30)
            return cls(config, parent, process), (width, height)
        except Exception:
            child.close()
            parent.close()
            if process is not None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            raise

    def submit(self, rgb, captured_at):
        # Ownership: producer must not modify this new RGB array after submission.
        with self.condition:
            if self.stopped or not self.enabled or self.error:
                return
            if self.pending is not None:
                self.replaced += 1
            self.pending = (rgb, captured_at, self.generation)
            self.condition.notify()

    def set_enabled(self, enabled):
        with self.condition:
            if self.enabled != enabled:
                self.enabled = enabled
                self.generation += 1
                self.latest = self.pending = None

    def snapshot(self, now):
        with self.condition:
            result = self.latest
            stats = {"completed": self.completed, "replaced": self.replaced, "error": self.error,
                     "age": None, "inference_ms": None}
            if result is None or not self.enabled or self.error:
                return [], stats
            age = now - result["captured_at"]
            stats.update(age=age, inference_ms=result["inference_ms"])
            if age < 0 or age > self.config["max_detection_age"]:
                return [], stats
            return result["detections"], stats

    def _work(self):
        last_started = 0.
        try:
            while True:
                with self.condition:
                    while self.pending is None and not self.stopped:
                        self.condition.wait(timeout=.5)
                    if self.stopped:
                        break
                    cap = self.config["inference_fps"]
                    delay = max(0, 1 / cap - (time.monotonic() - last_started)) if cap else 0
                    if delay:
                        self.condition.wait(timeout=delay)
                        continue
                    rgb, captured_at, generation = self.pending
                    self.pending = None
                last_started = time.monotonic()
                ipc_send(self.channel, {"event": "frame", "captured_at": captured_at,
                    "confidence": self.config["confidence"], "iou": self.config["iou"],
                    "max_objects": self.config["max_objects"]}, np.ascontiguousarray(rgb).tobytes())
                # Drop our image reference while waiting; only the single pending frame is retained.
                del rgb
                result, _ = ipc_receive(self.channel)
                if result.get("event") != "result":
                    raise RuntimeError("YOLO worker failed: " + str(result))
                if result.get("captured_at") != captured_at:
                    raise ValueError("Inference response timestamp does not match its input")
                with self.condition:
                    self.completed += 1
                    if self.enabled and generation == self.generation:
                        self.latest = result
        except Exception as exc:
            with self.condition:
                if not self.stopped:
                    self.error = str(exc)
                    self.latest = self.pending = None
                    LOG.exception("YOLO process unavailable; RGB/thermal preview continues")

    def close(self):
        with self.condition:
            self.stopped = True
            self.pending = self.latest = None
            self.condition.notify_all()
        try:
            self.channel.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.channel.close()
        self.thread.join(timeout=2)
        if self.process is not None:
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)


class RateMeter:
    """Recent count deltas, not model latency or a lifetime-average FPS."""
    def __init__(self):
        self.at, self.count = time.monotonic(), 0
        self.fps = 0.

    def update(self, count, now):
        if now - self.at >= 1:
            self.fps = max(0, count - self.count) / (now - self.at)
            self.at, self.count = now, count
        return self.fps
