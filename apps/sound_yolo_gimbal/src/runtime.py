"""MaixVision entry; all hardware construction is inside run_board()."""
import json


def run_board(c=CONFIG):
    validate_config(c)
    from maix import app, camera, display, touchscreen, image, sys as maix_sys
    if str(maix_sys.device_name()).lower() != "maixcam2":
        raise RuntimeError("This application and pin assignment require MaixCAM2")
    sensor = SoundDirectionSensor(c["mic_rotation"], c["mic_mirror"], c["sound_fresh_s"])
    motor = MotorService(c)
    controller = SoundVisualController(c)
    cam = disp = touch = vision = None
    try:
        print("STEP model.load", c["model_path"])
        vision = NativeVision(c)
        print("STEP camera.open")
        cam = camera.Camera(vision.model.input_width(), vision.model.input_height(), vision.model.input_format())
        disp, touch = display.Display(), touchscreen.TouchScreen()
        print("STEP microphone.open", c["mic_port"])
        sensor.open_uart(c["mic_port"], c["mic_baud"])
        # Prove camera and inference return before starting the motor service.
        print("STEP camera.first_frame")
        first = cam.read(block=True, block_ms=200)
        if first is None:
            raise RuntimeError("No first camera frame")
        print("STEP detector.first_frame")
        vision.detect(first)
        del first
        print("STEP motor.start", "REAL" if c["motor_enabled"] else "SIMULATION")
        motor.start()
        deadline = time.monotonic() + 3
        while not motor.snapshot()["ready"]:
            state = motor.snapshot()
            if state["fault"] or time.monotonic() > deadline or app.need_exit():
                raise RuntimeError(state["fault"] or "Motor initialization interrupted/timed out")
            time.sleep(.01)
        pressed_before = False
        last_log = last_frame = time.monotonic()
        last_inference = 0.0
        last_result_time = 0.0
        tracks, fps = [], 0.0
        previously_armed = False
        while not app.need_exit():
            sensor.poll()
            img = cam.read(block=True, block_ms=200)
            now = time.monotonic()
            if img is None:
                if now - last_frame > c["vision_max_age"]:
                    raise RuntimeError("Camera stream stale")
                continue  # Never refresh motor heartbeat without a current camera frame.
            last_frame = now
            new_vision = now - last_inference >= 1 / c["inference_fps"]
            if new_vision:
                captured = now
                tracks = vision.detect(img)
                now = time.monotonic()
                fps = 1 / max(.001, now - last_inference) if last_inference else 0.0
                last_inference, last_result_time = now, captured
            fresh = now - last_result_time <= c["vision_max_age"]
            if not fresh:
                tracks = []
                raise RuntimeError("YOLO result stale; motor stopped")
            state = motor.snapshot()
            if state["fault"]:
                controller.fail(state["fault"])
            elif not state["ready"] or state["age"] > c["watchdog_s"]:
                raise RuntimeError("Motor feedback stale")
            sound = sensor.snapshot(now)
            if not state["fault"] and state["armed"] != previously_armed:
                controller.reset(now, state["angles"], sound.get("frames", -1))
            previously_armed = state["armed"]
            tx, ty, pressed = touch.read()
            if pressed and not pressed_before:
                x, y = touch_to_image(tx, ty, img.width(), img.height(), disp.width(), disp.height())
                if 0 <= y < 38 and 0 <= x < 62:
                    break
                elif 0 <= y < 38 and 62 <= x < 132 and not state["fault"]:
                    controller.reset(now, state["angles"], sound.get("frames", -1))
                elif 0 <= y < 38 and 132 <= x < 228 and not state["fault"]:
                    motor.request_arm(not state["armed"])
                    controller.reset(now, state["angles"], sound.get("frames", -1))
                elif y >= 38:
                    hits = [t for t in tracks if t["box"][0] <= x <= t["box"][0]+t["box"][2] and t["box"][1] <= y <= t["box"][1]+t["box"][3]]
                    if hits and state["armed"] and not state["fault"]:
                        chosen = min(hits, key=lambda t: t["box"][2]*t["box"][3])
                        controller.select(chosen["key"], tracks, now, state["angles"])
            pressed_before = pressed
            target = None
            if not state["fault"]:
                if state["armed"]:
                    target = controller.step(now, sound, tracks, state["angles"], fresh, new_vision)
                    if target is None:
                        raise RuntimeError(controller.reason)
                    motor.submit(target)
                else:
                    controller.reset(now, state["angles"], sound.get("frames", -1))
                    controller.state, controller.reason = "DISARMED", "feedback only; tap ARM to start"
            draw_tracking(img, tracks, controller, sound, state, fps)
            disp.show(img, fit=image.Fit.FIT_CONTAIN)
            if now - last_log >= 1:
                print("SOUND_YOLO", json.dumps({"state": controller.state, "selected": controller.selected,
                      "sound": sound, "motor": state, "target": target, "fps": fps}))
                last_log = now
            del img
    except Exception as exc:
        controller.fail(str(exc))
        print("SYSTEM_FAULT", str(exc))
        raise
    finally:
        # Stop motors before closing peripherals, including partial-init failures.
        motor.close()
        for resource in (sensor, cam, touch, disp):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    print("CLOSE_FAILED", str(exc))
        vision = None
        print("SOUND_YOLO_STOP")


if __name__ == "__main__":
    run_board()
