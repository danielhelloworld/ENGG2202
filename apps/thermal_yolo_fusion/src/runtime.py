"""Hardware entry point and an explicitly synthetic PC demo for streaming tests."""
import sys


def camera_image_to_rgb(frame, image_module):
    """Return an owned RGB array; image2cv cannot ensure BGR with copy=False."""
    if frame.format() != image_module.Format.FMT_RGB888:
        frame = frame.to_format(image_module.Format.FMT_RGB888)
    return image_module.image2cv(frame, ensure_bgr=False, copy=True)


class BoardTrace:
    """Native faulthandler timer can report Python stacks even during a held GIL."""
    def __init__(self, enabled):
        self.enabled = enabled
        self.handler = None
        if enabled:
            try:
                import faulthandler
                faulthandler.dump_traceback_later(15, repeat=True)
                self.handler = faulthandler
            except (RuntimeError, OSError, ValueError) as exc:
                LOG.warning("Stack watchdog unavailable: %s", exc)

    def step(self, label, attempt=0):
        if self.enabled and attempt < 3:
            LOG.info("STEP %s [attempt %d]", label, attempt + 1)

    def progress(self):
        if self.handler:
            self.handler.cancel_dump_traceback_later()
            self.handler.dump_traceback_later(15, repeat=True)

    def close(self):
        if self.handler:
            self.handler.cancel_dump_traceback_later()


def validate_config(config):
    for key in ("width", "height"):
        value = config[key]
        if not isinstance(value, int) or value % 8 or not 160 <= value <= 2040:
            raise ValueError(key + " must be a multiple of 8 in [160,2040]")
    if not 0 < config["stream_fps"] <= 60 or not 1 <= config["jpeg_quality"] <= 95:
        raise ValueError("Invalid stream_fps/jpeg_quality")
    if not 0 <= config["alpha"] <= 1 or config["loop_fps"] < 0:
        raise ValueError("Invalid alpha/loop_fps")
    if config["camera_fps"] <= 0 or config["inference_fps"] < 0:
        raise ValueError("Invalid camera/inference FPS")
    if config["max_detection_age"] <= 0 or config["max_detection_thermal_delta"] <= 0:
        raise ValueError("Invalid detection time limits")
    if config["max_thermal_age"] <= 0 or config["max_pair_delta"] <= 0:
        raise ValueError("Thermal time limits must be positive")
    if config["detector_mode"] not in ("inline", "process"):
        raise ValueError("detector_mode must be inline or process")
    if not 1 <= config["camera_read_timeout_ms"] <= 1000 or config["camera_stall_seconds"] <= 0:
        raise ValueError("Invalid camera timeout")
    if (not -100 <= config["hot_min_c"] <= 320 or config["hot_delta_c"] <= 0 or
            not 1 <= config["hot_min_area_px"] <= PIXELS or
            not 1 <= config["hot_max_candidates"] <= 100 or
            config["hot_track_gate_px"] <= 0 or
            not 0 <= config["hot_track_misses"] <= 20 or
            config["hot_track_timeout_s"] <= 0):
        raise ValueError("Invalid hot-region tracking configuration")


def select_model(config):
    if config["model_path"]:
        candidates = [(config["model_path"], config["model_type"])]
    else:
        candidates = [("/root/models/yolo11n.mud", "YOLO11"),
                      ("/root/models/yolo11s.mud", "YOLO11"),
                      ("/root/models/yolov8n.mud", "YOLOv8")]
    for path, model_type in candidates:
        if os.path.isfile(path):
            return path, model_type
    raise FileNotFoundError(
        "No supported MaixCAM2 YOLO model found. Upload its .mud AND referenced .axmodel files "
        "to /root/models, then set CONFIG['model_path'] and CONFIG['model_type']. Tried: "
        + ", ".join(path for path, _ in candidates))


def touch_to_canvas(x, y, disp_w, disp_h, width, height):
    scale = min(disp_w / width, disp_h / height)
    ox, oy = (disp_w - width * scale) / 2, (disp_h - height * scale) / 2
    return (x - ox) / scale, (y - oy) / scale


def rtsp_addresses(port):
    addresses = set()
    try:
        for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = item[4][0]
            if not ip.startswith("127."):
                addresses.add(ip)
    except OSError:
        pass
    # Local read-only enumeration also covers USB gadget interfaces.
    try:
        import subprocess
        info = subprocess.check_output(["ip", "-j", "-4", "addr"], timeout=2)
        for iface in json.loads(info):
            for address in iface.get("addr_info", []):
                ip = address.get("local", "")
                if ip and not ip.startswith("127."):
                    addresses.add(ip)
    except Exception:
        pass
    return ["rtsp://%s:%d/live" % (ip, port) for ip in sorted(addresses)] or ["rtsp://<DEVICE_IP>:%d/live" % port]


def run_board():
    from maix import app, camera, display, image, touchscreen
    from maix import sys as maix_sys
    import maix

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    c = dict(CONFIG)
    validate_config(c)
    if maix_sys.device_name() != "MaixCAM2":
        raise RuntimeError("This app targets MaixCAM2 + Thermal160 UART telemetry30")
    LOG.info("MaixPy %s, Python %s", getattr(maix, "__version__", "unknown"), sys.version.split()[0])
    LOG.info("RTSP/JPEG output contains the composed thermal + YOLO image; use TCP transport")
    cam = disp = thermal = stream = detector = None
    trace = BoardTrace(c["diagnostic_trace"])
    try:
        trace.step("display.init")
        disp = display.Display()
        path, kind = select_model(c)
        if kind not in ("YOLO11", "YOLOv8", "YOLOv5"):
            raise ValueError("model_type must be YOLO11, YOLOv8 or YOLOv5")
        LOG.info("Loading %s: %s; detector_mode=%s", kind, path, c["detector_mode"])
        trace.step("detector.init")
        if c["detector_mode"] == "process":
            LOG.warning("Experimental cross-process Maix hardware mode; not validated on this board")
            detector, (iw, ih) = AsyncDetector.launch(c, os.path.abspath(__file__), path, kind)
        else:
            detector, (iw, ih) = InlineDetector.launch(c, path, kind)
        trace.step("camera.init")
        cam = camera.Camera(iw, ih, image.Format.FMT_RGB888, fps=c["camera_fps"])
        LOG.info("Requested RGB %dfps; composition cap=%s; independent thermal UART", c["camera_fps"], c["loop_fps"] or "none")
        try:
            ts = touchscreen.TouchScreen()
        except Exception as exc:
            LOG.warning("Touch unavailable; use MaixVision Stop / USER key: %s", exc)
            ts = None
        # Both single-file and project execution work; no imported sibling modules.
        app_dir = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
        alignment = Alignment(c, (iw, ih), c.get("alignment_path") or os.path.join(app_dir, "alignment.json"))
        hot_tracker = HotTracker(c)
        ui = TouchUI(alignment, hot_tracker)
        if c["rtsp_enabled"]:
            stream = JpegRtspServer(c["rtsp_host"], c["rtsp_port"], c["stream_fps"], c["jpeg_quality"], c["max_clients"])
            for address in rtsp_addresses(stream.port):
                LOG.info("PLAY: %s", address)
                LOG.info('ffplay -rtsp_transport tcp -i "%s"', address)
        thermal = ThermalReader(c)
        thermal.start()
        last_stats = time.monotonic()
        frames = 0
        attempts = 0
        last_camera_frame = time.monotonic()
        last_camera_warning = last_camera_frame
        visual_rate, thermal_rate, ai_rate, stream_rate = RateMeter(), RateMeter(), RateMeter(), RateMeter()
        while not app.need_exit():
            started = time.monotonic()
            trace.step("touch.read", attempts)
            if ts is not None:
                # Latest accepted thermal frame is used for touch picking; its age is checked.
                pick_frame, _, _ = thermal.snapshot()
                hot_tracker.update(pick_frame, started)
                tx, ty, pressed = ts.read()
                tx, ty = touch_to_canvas(tx, ty, disp.width(), disp.height(), c["width"], c["height"])
                if ui.touch(tx, ty, pressed):
                    app.set_exit_flag(True)
                    break
            trace.step("camera.read", attempts)
            camera_img = cam.read(block=True, block_ms=c["camera_read_timeout_ms"])
            visible_at = time.monotonic()
            attempts += 1
            if camera_img is None:
                if visible_at - last_camera_frame >= c["camera_stall_seconds"]:
                    raise RuntimeError("Camera produced no frames for %.1fs; check sensor/FPS mode and restart the board" % (visible_at - last_camera_frame))
                if visible_at - last_camera_warning >= 2:
                    LOG.warning("CAMERA WAIT: no frame for %.1fs", visible_at - last_camera_frame)
                    last_camera_warning = visible_at
                time.sleep(.005)
                continue
            if frames == 0:
                LOG.info("Camera frame format %s; preserving RGB channel order", camera_img.format())
            last_camera_frame = visible_at
            trace.step("image.to_rgb", attempts - 1)
            thermal_frame, reader_status, thermal_count = thermal.snapshot()
            hot_tracker.update(thermal_frame, visible_at)
            rgb = camera_image_to_rgb(camera_img, image)
            # rgb owns its pixels; release the camera view before acquiring another frame.
            del camera_img
            trace.step("detector.submit", attempts - 1)
            detector.set_enabled(not ui.registration.active)
            detector.submit(rgb, visible_at)
            now = time.monotonic()
            detections, ai_stats = detector.snapshot(now)
            trace.step("compose", attempts - 1)
            ui.registration.observe(rgb, thermal_frame, visible_at, now)
            canvas, _, thermal_status = compose(rgb, detections, thermal_frame, alignment, now,
                                                visible_at, ui.mode, ui.alpha,
                                                ui.show_temperatures, ui.show_heat)
            _, thermal_visible, _ = thermal_state(thermal_frame, visible_at, now, c)
            hot_tracker.draw(canvas, alignment, thermal_visible, ui.show_temperatures)
            if stream is None:
                stream_status = "RTSP OFF"
            elif stream.error:
                stream_status = "RTSP ENCODE ERROR (console)"
            else:
                stream_status = "RTSP:%d clients:%d" % (stream.port, stream.client_count())
            status = "V%.1f T%.1f AI%.1f NET%.1f | %s" % (visual_rate.fps, thermal_rate.fps, ai_rate.fps, stream_rate.fps, stream_status)
            if ai_stats["error"]:
                status = "YOLO ERROR (console) | " + status
            elif ai_stats["age"] is not None:
                status += " | AI age %.0fms" % (ai_stats["age"] * 1000)
            if hot_tracker.mode != "OFF":
                status += " | " + hot_tracker.status(ui.show_temperatures)
            ui.draw(canvas, status)
            if ui.registration.active:
                canvas = ui.registration.render()
            # Exactly this rendered RGB image goes to BOTH outputs.
            shown = image.cv2image(canvas, bgr=False, copy=True)
            trace.step("display.show", attempts - 1)
            disp.show(shown, fit=image.Fit.FIT_CONTAIN)
            del shown
            trace.step("stream.submit", attempts - 1)
            if stream is not None:
                stream.submit(canvas, now)
            finished = time.monotonic()
            frames += 1
            trace.step("frame.done", attempts - 1)
            trace.progress()
            visual_rate.update(frames, finished)
            thermal_rate.update(thermal_count, finished)
            ai_rate.update(ai_stats["completed"], finished)
            stream_rate.update(stream.encoded if stream else 0, finished)
            if finished - last_stats >= 5:
                LOG.info("V=%.1f T=%.1f AI=%.1f NET=%.1f fps; boxes=%d; AI age=%s; inference_ms=%s; replaced=%d; %s/%s",
                         visual_rate.fps, thermal_rate.fps, ai_rate.fps, stream_rate.fps,
                         len(detections), ai_stats["age"], ai_stats["inference_ms"], ai_stats["replaced"],
                         thermal_status, reader_status)
                last_stats = finished
            remaining = 1.0 / c["loop_fps"] - (finished - started) if c["loop_fps"] else 0
            if remaining > 0:
                time.sleep(remaining)
    except Exception:
        LOG.exception("Application stopped; see the error above")
        raise
    finally:
        for resource in (detector, thermal, stream, cam, disp):
            if resource is not None:
                try:
                    resource.close()
                except Exception:
                    LOG.exception("Resource cleanup failed")
        # Native display/model resources are released with their Python objects.
        detector = cam = disp = None
        trace.close()


def demo_frame(width=640, height=480, seconds=0):
    """Explicit test fixture. Never used by run_board or substituted for missing hardware."""
    yy, xx = np.indices((height, width))
    rgb = np.zeros((height, width, 3), np.uint8)
    rgb[:, :, 0] = (xx * 180 // width + 20).astype(np.uint8)
    rgb[:, :, 1] = (yy * 140 // height + 20).astype(np.uint8)
    rgb[:, :, 2] = 55
    cv2.rectangle(rgb, (width // 3, height // 4), (width * 2 // 3, height * 3 // 4), (185, 190, 200), -1)
    ty, tx = np.indices((THERMAL_H, THERMAL_W), dtype=np.float32)
    pixels = np.clip(40 + 190 * np.exp(-((tx - 80) ** 2 + (ty - 60) ** 2) / 500), 0, 254).astype(np.uint8)
    frame = {"pixels": pixels, "lo": 200, "hi": 450, "valid_temperature": True, "received_at": time.monotonic()}
    detections = [{"box": (width // 3, height // 4, width // 3, height // 2), "label": "DEMO target", "score": .94}]
    return rgb, frame, detections


def run_demo():
    import argparse
    parser = argparse.ArgumentParser(description="Synthetic fusion/RTSP test; no hardware inference")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18554)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--snapshot", default="")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    c = dict(CONFIG)
    alignment = Alignment(c, (c["width"], c["height"]))
    ui = TouchUI(alignment)
    server = JpegRtspServer(args.host, args.port, c["stream_fps"], c["jpeg_quality"])
    print("SYNTHETIC DEMO: rtsp://%s:%d/live" % (args.host, server.port), flush=True)
    start = time.monotonic()
    saved = False
    try:
        while time.monotonic() - start < args.seconds:
            rgb, frame, detections = demo_frame(c["width"], c["height"], time.monotonic() - start)
            now = time.monotonic()
            canvas, _, _ = compose(rgb, detections, frame, alignment, now, now)
            ui.draw(canvas, "SYNTHETIC DEMO | no real camera, YOLO or temperatures")
            draw_text_rgb(canvas, "SYNTHETIC TEST", 350, 88, (255, 255, 255), .65)
            server.submit(canvas, now)
            if args.snapshot and not saved:
                if not cv2.imwrite(args.snapshot, cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError("Could not write demo snapshot")
                saved = True
            time.sleep(1.0 / (c["loop_fps"] or 30))
    finally:
        server.close()


if __name__ == "__main__":
    if len(sys.argv) >= 5 and sys.argv[1] == "--yolo-worker":
        run_yolo_worker(int(sys.argv[2]), sys.argv[3], sys.argv[4])
    elif "--demo" in sys.argv:
        run_demo()
    else:
        run_board()
