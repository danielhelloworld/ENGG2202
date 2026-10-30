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


def draw_panel(disp, img, frame, hub):
    from maix import image
    state = hub.state()
    rgb = image.Color.from_rgb
    canvas = image.Image(640, 480, image.Format.FMT_RGB888, bg=rgb(10, 16, 26))
    mode = "HOMING" if state.get("arm_homing") else state["vision_state"]
    canvas.draw_string(8, 10, "YOLO CLICK | %s | %s" % (mode, state["phase"]), rgb(240, 240, 240), scale=1.3)
    x, y, w, h = contain_rect(frame["width"], frame["height"])
    canvas.draw_image(x, y, img.resize(w, h))
    if state["vision_age"] is None or state["vision_age"] > hub.c["vision_max_age"]:
        canvas.draw_string(10, 46, "VISION STALE - CLICK DISABLED", rgb(255, 70, 80), scale=1.3)
    labels = {"arm": "ARM / HOLD", "stop": "DISARM", "zero": "ZERO OK", "recheck": "RECHECK",
              "c": "C: HOME" if state["c_next_home"] else "C: CLEAR", "home": "HOME"}
    for name, (bx, by, bw, bh) in PANEL_BUTTONS.items():
        canvas.draw_rect(bx, by, bw, bh, rgb(155, 40, 50) if name == "stop" else rgb(35, 65, 85), -1)
        canvas.draw_string(bx+10, by+8, labels[name], rgb(240, 240, 240))
    canvas.draw_circle(560, 330, 55, rgb(70, 110, 140), 2)
    canvas.draw_circle(int(560+hub.manual[0]*36), int(330-hub.manual[1]*36), 12, rgb(40, 215, 205), -1)
    canvas.draw_string(492, 395, state["control_owner"], rgb(120, 190, 210))
    canvas.draw_string(8, 412, "Y %.1f P %.1f | selected %s" % (*state["angles"], str(state["selected"])), rgb(230, 235, 240))
    canvas.draw_string(8, 438, str(state["fault"] or hub.notice)[:83], rgb(255, 195, 70))
    perf = state.get("performance", {})
    canvas.draw_string(8, 461, "YOLO %.1ffps | result age %.0fms | C clear, C again HOME" % (
        perf.get("inference_fps_actual", 0.), perf.get("capture_to_result_ms") or 0.), rgb(140, 160, 180))
    disp.show(canvas)


def tracker_main():
    from maix import app, camera, display, touchscreen, sys as maix_sys
    if str(maix_sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART3 mapping requires MaixCAM2")
    validate_visual_config(CONFIG)
    print("YOLO_CLICK VERSION", CONFIG["app_version"], flush=True)
    driver = VisualMotorService(dict(CONFIG))
    hub = TrackingHub(driver, CONFIG)
    cam = disp = touch = server = server_thread = vision = pipeline = None
    frame = preview = None
    panel = LocalPanel(CONFIG)
    try:
        print("YOLO_CLICK STEP model.load", CONFIG["model_path"], flush=True)
        vision = NativeVision(CONFIG)
        pipeline = DetectionPipeline(vision, CONFIG)
        print("YOLO_CLICK STEP camera.open", flush=True)
        cam = camera.Camera(vision.model.input_width(), vision.model.input_height(), vision.model.input_format(), fps=CONFIG["camera_fps"])
        camera_rotated = configure_camera_rotation(cam, CONFIG)
        print("NPU_MODEL %s %s %dx%d dual_buff=%s" % (
            CONFIG["model_kind"], CONFIG["model_path"], vision.model.input_width(), vision.model.input_height(), CONFIG["dual_buff"]), flush=True)
        disp, touch = display.Display(), touchscreen.TouchScreen()
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
        frame = hub.offer_frame(tracks, img.width(), img.height(), stamp, capture_pose)
        hub.performance = dict(pipeline.stats(), camera_rotation=camera_rotated)
        draw_visual(img, frame, None)
        hub.publish_media(frame, img.to_jpeg(CONFIG["jpeg_quality"]).to_bytes())
        draw_panel(disp, img, frame, hub)
        preview = img
        del img
        server = make_http_server(hub, (WEB_CONFIG["host"], WEB_CONFIG["port"]))
        driver.start()
        server_thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .05}, daemon=True)
        server_thread.start()
        print("YOLO_CLICK READY | no microphone | UART3 B2/B3 | " + (
            "AUTOMATIC STARTUP HOME: motor may move immediately" if CONFIG["startup_auto_home"] else "START DISABLED"), flush=True)
        try:
            import psutil
            ips = {a.address for values in psutil.net_if_addrs().values() for a in values
                   if a.family == socket.AF_INET and not a.address.startswith("127.")}
        except Exception:
            ips = {"CAMERA_IP"}
        for ip in sorted(ips):
            print("COMPUTER OPEN: http://%s:%d" % (ip, WEB_CONFIG["port"]), flush=True)
        last_infer = last_log = last_preview = 0.
        while not app.need_exit():
            now = time.monotonic()
            x, y, pressed = touch.read()
            try:
                panel.touch(hub, x*640/disp.width(), y*480/disp.height(), pressed, frame, now)
            except (RuntimeError, ControlError) as exc:
                hub.notice = str(exc)
            if now-last_infer >= 1/CONFIG["inference_fps"]:
                last_infer = now
                img = cam.read(block=True, block_ms=200)
                if img is not None:
                    stamp = time.monotonic()
                    pose_state = driver.snapshot()
                    capture_pose = tuple(pose_state["angles"]) if pose_state["reference_valid"] and not pose_state.get("feedback_degraded") and pose_state["age"] is not None and pose_state["age"] <= CONFIG["pose_max_age_s"] else None
                    img = prepare_image(img, CONFIG, vision.model, camera_rotated)
                    result = pipeline.push(img, stamp, capture_pose)
                    if result is None:
                        hub.tick()
                        continue
                    img, tracks, stamp, capture_pose = result
                    current = hub.offer_frame(tracks, img.width(), img.height(), stamp, capture_pose)
                    hub.performance = dict(pipeline.stats(), camera_rotation=camera_rotated)
                    hub.tick()  # Submit control before JPEG encoding / screen drawing.
                    if time.monotonic()-last_preview >= 1/CONFIG["preview_fps"]:
                        draw_visual(img, current, hub.tracking.selected, hub.tracking.prediction_point)
                        hub.publish_media(current, img.to_jpeg(CONFIG["jpeg_quality"]).to_bytes())
                        draw_panel(disp, img, current, hub)
                        frame = current  # Local click always references the displayed frame.
                        preview = img
                        last_preview = time.monotonic()
                    del img
                elif preview is not None:
                    draw_panel(disp, preview, frame, hub)
            hub.tick()
            if now-last_log >= 1:
                print("YOLO_CLICK " + json.dumps(hub.state()), flush=True)
                last_log = now
            time.sleep(.003)
    finally:
        driver.request_arm(False)
        driver.close()
        preview = result = img = None
        if pipeline is not None:
            pipeline.close()
        if server is not None:
            if server_thread is not None:
                server.shutdown()
                server_thread.join(1)
            server.server_close()
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
