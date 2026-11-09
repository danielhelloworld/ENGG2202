"""MaixCAM2 runtime. No microphone or sound-direction resources are opened."""
PANEL_BUTTONS = {name: (490, 42+i*36, 145, 32) for i, name in enumerate(
    ("arm", "stop", "zero", "recheck", "c", "home"))}


class LocalPanel:
    def __init__(self, c):
        self.c = c
        self.pressed = self.captured = False
        self.started = 0.

    def touch(self, hub, x, y, pressed, frame, now):
        edge = pressed and not self.pressed
        self.pressed = bool(pressed)
        if not pressed:
            if self.captured:
                self.captured = False
                hub.local_action(vector=(0, 0))
            return
        sx, sy, sw, sh = PANEL_BUTTONS["stop"]
        if sx <= x < sx+sw and sy <= y < sy+sh:
            self.captured = False
            hub.local_action("stop")
            return
        if edge:
            for name, (bx, by, bw, bh) in PANEL_BUTTONS.items():
                if bx <= x < bx+bw and by <= y < by+bh:
                    hub.local_action(name)
                    return
            if math.hypot(x-560, y-330) <= 57:
                self.captured, self.started = True, now
            elif frame:
                point = screen_to_normalized(x, y, contain_rect(frame["width"], frame["height"]))
                if point:
                    hub.local_action(click=(frame["id"], *point))
        if self.captured:
            dx, dy = (x-560)/50, (330-y)/50
            length = math.hypot(dx, dy)
            if now-self.started >= self.c["max_touch_hold_s"] or length < self.c["joystick_deadzone"]:
                vector = (0., 0.)
            else:
                strength = min(1., (length-self.c["joystick_deadzone"])/(1-self.c["joystick_deadzone"]))
                vector = (dx/length*strength, dy/length*strength)
            hub.local_action(vector=vector)


def prepare_image(img, c, model, camera_rotated=False):
    from maix import image
    if c["rotate_180"] and not camera_rotated:
        import cv2
        if img.format() != image.Format.FMT_RGB888:
            img = img.to_format(image.Format.FMT_RGB888)
        rgb = image.image2cv(img, ensure_bgr=False, copy=True)
        img = image.cv2image(cv2.rotate(rgb, cv2.ROTATE_180), bgr=False, copy=True)
    if img.format() != model.input_format():
        img = img.to_format(model.input_format())
    return img


def draw_visual(img, frame, selected, prediction=None):
    from maix import image
    green, yellow = image.Color.from_rgb(40, 230, 130), image.Color.from_rgb(255, 210, 60)
    for tr in frame["tracks"]:
        chosen = tuple(tr["key"]) == selected
        color = yellow if chosen else green
        x, y, w, h = map(int, tr["box"])
        img.draw_rect(x, y, w, h, color, 2)
        img.draw_string(max(0, x), max(0, y-18), "%s %d:%d %.2f" % (tr["label"], *tr["key"], tr["score"]), color)
        if chosen:
            for px, py in tr.get("trail", [])[-40:]:
                img.draw_circle(int(px), int(py), 2, yellow, -1)
            img.draw_line(int(tr["cx"]*img.width()), int(tr["cy"]*img.height()), img.width()//2, img.height()//2, yellow, 1)
    cx, cy = img.width()//2, img.height()//2
    img.draw_line(cx-10, cy, cx+10, cy, yellow, 1)
    img.draw_line(cx, cy-10, cx, cy+10, yellow, 1)
    if prediction is not None:
        px = int(clip(prediction[0], 0., 1.)*(img.width()-1))
        py = int(clip(prediction[1], 0., 1.)*(img.height()-1))
        color = image.Color.from_rgb(45, 195, 255)
        img.draw_circle(px, py, 5, color, 2)
        img.draw_string(max(0, px-12), max(0, py-18), "PRED", color)


def draw_panel(disp, img, frame, hub, timings=None, cache=None):
    from maix import image
    started = time.monotonic()
    state = hub.state()
    if timings is not None:
        timings.add("panel_state", (time.monotonic()-started)*1000)
    rgb = image.Color.from_rgb
    cache = {} if cache is None else cache
    if "canvas" not in cache:
        cache["canvas"] = image.Image(640, 480, image.Format.FMT_RGB888, bg=rgb(10, 16, 26))
    canvas = cache["canvas"]
    mode = "HOMING" if state.get("arm_homing") else state["vision_state"]
    header = "YOLO CLICK | %s | %s" % (mode, state["phase"])
    if cache.get("header") != header:
        canvas.draw_rect(0, 0, 640, 38, rgb(10, 16, 26), -1)
        canvas.draw_string(8, 10, header, rgb(240, 240, 240), scale=1.3)
        cache["header"] = header
    x, y, w, h = contain_rect(frame["width"], frame["height"])
    video_started = time.monotonic()
    if cache.get("frame_id") != frame["id"]:
        resized = img.resize(w, h)
        if timings is not None:
            timings.add("panel_resize", (time.monotonic()-video_started)*1000)
        canvas.draw_image(x, y, resized)
        cache["frame_id"] = frame["id"]
    if state["vision_age"] is None or state["vision_age"] > hub.c["vision_max_age"]:
        canvas.draw_string(10, 46, "VISION STALE - CLICK DISABLED", rgb(255, 70, 80), scale=1.3)
    labels = {"arm": "ARM / HOLD", "stop": "DISARM", "zero": "ZERO OK", "recheck": "RECHECK",
              "c": "C: HOME" if state["c_next_home"] else "C: CLEAR", "home": "HOME"}
    if cache.get("labels") != labels:
        for name, (bx, by, bw, bh) in PANEL_BUTTONS.items():
            if cache.get("labels", {}).get(name) == labels[name]:
                continue
            canvas.draw_rect(bx, by, bw, bh, rgb(155, 40, 50) if name == "stop" else rgb(35, 65, 85), -1)
            canvas.draw_string(bx+10, by+8, labels[name], rgb(240, 240, 240))
        cache["labels"] = labels
    vector = tuple(hub.manual)
    if cache.get("vector") != vector:
        canvas.draw_rect(490, 270, 150, 120, rgb(10, 16, 26), -1)
        canvas.draw_circle(560, 330, 55, rgb(70, 110, 140), 2)
        canvas.draw_circle(int(560+vector[0]*36), int(330-vector[1]*36), 12, rgb(40, 215, 205), -1)
        cache["vector"] = vector
    if cache.get("owner") != state["control_owner"]:
        canvas.draw_rect(490, 390, 150, 20, rgb(10, 16, 26), -1)
        canvas.draw_string(492, 395, state["control_owner"], rgb(120, 190, 210))
        cache["owner"] = state["control_owner"]
    important = (state["fault"], str(state["selected"]), hub.notice)
    if started-cache.get("footer_time", float("-inf")) >= .2 or cache.get("important") != important:
        canvas.draw_rect(0, 410, 640, 70, rgb(10, 16, 26), -1)
        canvas.draw_string(8, 412, "Y %.1f P %.1f | selected %s" % (*state["angles"], str(state["selected"])), rgb(230, 235, 240))
        canvas.draw_string(8, 438, str(state["fault"] or hub.notice)[:83], rgb(255, 195, 70))
        perf = state.get("performance", {})
        canvas.draw_string(8, 461, "YOLO %.1ffps | result age %.0fms | C clear, C again HOME" % (
            perf.get("inference_fps_actual", 0.), perf.get("capture_to_result_ms") or 0.), rgb(140, 160, 180))
        cache.update(footer_time=started, important=important)
    composed = time.monotonic()
    if timings is not None:
        timings.add("panel_compose", (composed-started)*1000)
    disp.show(canvas)
    if timings is not None:
        timings.add("display_show", (time.monotonic()-composed)*1000)


def tracker_main():
    from maix import app, camera, display, touchscreen, sys as maix_sys
    if str(maix_sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART3 mapping requires MaixCAM2")
    validate_visual_config(CONFIG)
    print("YOLO_CLICK VERSION", CONFIG["app_version"], flush=True)
    driver = VelocityMotorService(dict(CONFIG))
    hub = TrackingHub(driver, CONFIG)
    try:
        # Old position-stream tuning profiles do not describe the new servo.
        if not isinstance(driver,VelocityMotorService):hub.software_autotune.load()
    except (ValueError, OSError, TypeError, KeyError) as exc:
        print("SOFTWARE_AUTOTUNE profile load skipped", str(exc), flush=True)
    cam = disp = touch = server = server_thread = vision = pipeline = renderer = None
    frame = preview = None
    panel = LocalPanel(CONFIG)
    timings = StageTimings()
    try:
        print("YOLO_CLICK STEP model.load", CONFIG["model_path"], flush=True)
        vision = NativeVision(CONFIG)
        pipeline = DetectionPipeline(vision, CONFIG)
        print("YOLO_CLICK STEP camera.open", flush=True)
        cam = camera.Camera(vision.model.input_width(), vision.model.input_height(), vision.model.input_format(),
                            fps=CONFIG["camera_fps"], buff_num=CONFIG["camera_buffers"])
        camera_rotated = configure_camera_rotation(cam, CONFIG)
        print("NPU_MODEL %s %s %dx%d dual_buff=%s" % (
            CONFIG["model_kind"], CONFIG["model_path"], vision.model.input_width(), vision.model.input_height(), CONFIG["dual_buff"]), flush=True)
        disp, touch = display.Display(), touchscreen.TouchScreen()
        # Display.show also transports JPEG to the IDE when its preview is enabled.
        # Physical display remains full resolution; reduce only IDE JPEG quality.
        display.set_trans_image_quality(70)
        # Validate camera/model before starting the motor thread.
        result = None
        for _ in range(2 if CONFIG["dual_buff"] else 1):
            img = cam.read(block=True, block_ms=200)
            if img is None:
                raise RuntimeError("No initial camera frame")
            stamp = time.monotonic()
            img = prepare_image(img, CONFIG, vision.model, camera_rotated)
            result = pipeline.push(img, stamp, None)
        if result is None:
            raise RuntimeError("YOLO pipeline did not produce an initial frame")
        img, tracks, stamp, capture_pose = result
        result = None
        frame = hub.offer_frame(tracks, img.width(), img.height(), stamp, capture_pose)
        hub.performance = dict(pipeline.stats(), camera_rotation=camera_rotated)
        draw_visual(img, frame, None)
        panel_cache = {}
        draw_panel(disp, img, frame, hub, timings, panel_cache)
        preview = img
        if CONFIG["web_enabled"]:
            hub.offer_preview_image(frame, img)
        last_visual_id = frame["id"]
        def render(current, picture, selected, prediction):
            nonlocal last_visual_id
            if current["id"] != last_visual_id:
                started = time.monotonic()
                draw_visual(picture, current, selected, prediction)
                timings.add("overlay", (time.monotonic()-started)*1000)
                last_visual_id = current["id"]
                # The finished source is immutable from this point, including idle redraws.
                if server is not None:
                    hub.offer_preview_image(current, picture)
            draw_panel(disp, picture, current, hub, timings, panel_cache)
        renderer = LatestPreview(render, CONFIG["preview_fps"], timings)
        renderer.displayed = frame
        renderer.offer(frame, img, None, None)
        del img
        if CONFIG["web_enabled"]:
            server = make_http_server(hub, (WEB_CONFIG["host"], WEB_CONFIG["port"]))
        driver.start()
        renderer.thread.start()
        if server is not None:
            server_thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .05}, daemon=True)
            server_thread.start()
        print("YOLO_CLICK READY | no microphone | UART3 B2/B3 | " + (
            "AUTOMATIC STARTUP HOME: motor may move immediately" if CONFIG["startup_auto_home"] else "START DISABLED"), flush=True)
        if server is not None:
            try:
                import psutil
                ips = {a.address for values in psutil.net_if_addrs().values() for a in values
                       if a.family == socket.AF_INET and not a.address.startswith("127.")}
            except Exception:
                ips = {"CAMERA_IP"}
            for ip in sorted(ips):
                print("COMPUTER OPEN: http://%s:%d" % (ip, WEB_CONFIG["port"]), flush=True)
        last_infer = last_log = last_metrics = last_control_tick = 0.
        last_loop = time.monotonic()
        control_latencies = deque(maxlen=120)
        empty_camera_polls = 0
        while not app.need_exit():
            now = time.monotonic()
            timings.add("main_loop_interval", (now-last_loop)*1000)
            last_loop = now
            if renderer.error is not None:
                raise RuntimeError("Local preview failed") from renderer.error
            x, y, pressed = touch.read()
            try:
                panel.touch(hub, x*640/disp.width(), y*480/disp.height(), pressed, renderer.displayed, now)
            except (RuntimeError, ControlError) as exc:
                hub.notice = str(exc)
            control_ticked = False
            now = time.monotonic()
            if now-last_infer >= 1/CONFIG["inference_fps"]:
                # Empty polls leave the inference slot open for the next available frame.
                img = cam.read(block=False)
                capture_read_ms = (time.monotonic()-now)*1000
                timings.add("camera_read", capture_read_ms)
                if img is not None:
                    last_infer = now
                    stamp = time.monotonic()
                    capture_pose = driver.capture_pose()
                    vision.preview_target = hub.tracking.selected
                    prepared = time.monotonic()
                    img = prepare_image(img, CONFIG, vision.model, camera_rotated)
                    timings.add("prepare_image", (time.monotonic()-prepared)*1000)
                    result = pipeline.push(img, stamp, capture_pose)
                    timings.add("detect_and_track", pipeline.api_ms)
                    if result is None:
                        hub.tick()
                        continue
                    img, tracks, stamp, capture_pose = result
                    offered = time.monotonic()
                    current = hub.offer_frame(tracks, img.width(), img.height(), stamp, capture_pose)
                    timings.add("offer_frame", (time.monotonic()-offered)*1000)
                    hub.performance.update(pipeline.stats(), camera_rotation=camera_rotated,
                                           camera_read_ms=capture_read_ms)
                    tick_started = time.monotonic()
                    hub.tick()  # Submit control before handing the image to the display worker.
                    last_control_tick = time.monotonic()
                    timings.add("control_tick", (time.monotonic()-tick_started)*1000)
                    control_ticked = True
                    control_ms = (time.monotonic()-stamp)*1000
                    control_latencies.append(control_ms)
                    hub.performance["capture_to_control_tick_ms"] = control_ms
                    renderer.offer(current, img, hub.tracking.selected, hub.tracking.prediction_point)
                    del img
                else:
                    empty_camera_polls += 1
            if not control_ticked and time.monotonic()-last_control_tick >= .01:
                tick_started = time.monotonic()
                hub.tick()
                last_control_tick = time.monotonic()
                timings.add("control_tick", (last_control_tick-tick_started)*1000)
            if now-last_metrics >= 1:
                ordered = sorted(control_latencies)
                hub.performance.update(
                    camera_empty_polls=empty_camera_polls,
                    capture_to_control_samples=len(ordered),
                    capture_to_control_p50_ms=ordered[math.ceil(.5*len(ordered))-1] if ordered else None,
                    capture_to_control_p95_ms=ordered[math.ceil(.95*len(ordered))-1] if ordered else None,
                )
                hub.performance.update(renderer.stats(), stages=dict(timings.snapshot(), **hub.timings.snapshot(), **driver.timings.snapshot()),
                                       timestamp_basis="camera.read return; excludes sensor/ISP queue and LCD scanout")
                last_metrics = now
            if now-last_log >= 5:
                print("YOLO_CLICK " + json.dumps(dict(version=CONFIG["app_version"], phase=driver.phase,
                    fps=hub.performance.get("inference_fps_actual"), display_fps=hub.performance.get("local_preview_fps_actual"),
                    control_p95_ms=hub.performance.get("capture_to_control_p95_ms"), fault=driver.fault)), flush=True)
                last_log = now
            time.sleep(.003)
    finally:
        driver.request_arm(False,reason="SCRIPT_EXIT：脚本退出／异常清理")
        driver.close()
        if renderer is not None:
            renderer.close()
        preview = result = img = None
        if pipeline is not None:
            pipeline.close()
        if server is not None:
            if server_thread is not None:
                server.shutdown()
                server_thread.join(1)
            server.server_close()
        hub.close_preview()
        for resource in (touch, cam, disp):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    print("YOLO_CLICK CLOSE_FAILED", str(exc))
        vision = None
        print("YOLO_CLICK STOP: disable requested", flush=True)


if __name__ == "__main__":
    tracker_main()
