"""640x480 logical touch UI; callbacks enqueue requests, never touch UART."""
import json

BUTTONS = {
    "arm": (16, 390, 120, 50), "stop": (146, 390, 120, 50),
    "zero": (276, 390, 140, 50), "home": (426, 390, 198, 50),
    "recheck": (352, 335, 270, 44),
}


class JoystickUI:
    def __init__(self, c):
        self.c = c
        self.pressed = False
        self.captured = False
        self.started = None
        self.vector = (0.0, 0.0)
        self.blocked = False

    def touch(self, x, y, pressed, now):
        action = None
        new_press = pressed and not self.pressed
        if not pressed:
            self.captured = self.blocked = False
            self.started = None
            self.vector = (0.0, 0.0)
        else:
            # DISARM also works if a captured finger slides onto its button.
            bx, by, bw, bh = BUTTONS["stop"]
            if bx <= x < bx+bw and by <= y < by+bh:
                action = "stop"
                self.captured = False
                self.vector = (0.0, 0.0)
            elif new_press:
                for name, (bx, by, bw, bh) in BUTTONS.items():
                    if bx <= x < bx+bw and by <= y < by+bh:
                        action = name
                        break
                if action is None and math.hypot(x-175, y-235) <= 112:
                    self.captured = True
                    self.started = now
            if self.captured:
                if now - self.started >= self.c["max_touch_hold_s"]:
                    self.blocked = True
                dx, dy = (x-175)/100.0, (235-y)/100.0
                length = math.hypot(dx, dy)
                if self.blocked or length <= self.c["joystick_deadzone"]:
                    self.vector = (0.0, 0.0)
                else:
                    strength = min(1.0, (length-self.c["joystick_deadzone"])/(1-self.c["joystick_deadzone"]))
                    self.vector = (dx/length*strength, dy/length*strength)
        self.pressed = bool(pressed)
        return self.vector, action


def draw_screen(disp, snapshot, ui, notice=""):
    from maix import image
    rgb = image.Color.from_rgb
    img = image.Image(640, 480, image.Format.FMT_RGB888, bg=rgb(10, 16, 26))
    white, muted, cyan, amber = rgb(235, 242, 250), rgb(145, 160, 180), rgb(40, 215, 205), rgb(255, 195, 70)
    img.draw_string(16, 12, "F32C JOYSTICK | MECH ONLY", white, scale=1.5)
    state = snapshot["phase"]
    img.draw_string(16, 48, state, amber if snapshot["fault"] else cyan, scale=1.3)
    img.draw_string(16, 80, "Release: hold pose | DISARM: torque off", muted)
    img.draw_circle(175, 235, 106, rgb(70, 95, 120), 3)
    img.draw_circle(175, 235, 50, rgb(35, 55, 75), 1)
    img.draw_line(65, 235, 285, 235, rgb(40, 60, 80), 1)
    img.draw_line(175, 125, 175, 345, rgb(40, 60, 80), 1)
    dx, dy = ui.vector
    img.draw_circle(int(175+dx*86), int(235-dy*86), 24, cyan, -1)
    img.draw_string(140, 108, "+PITCH", muted)
    img.draw_string(286, 226, "-YAW" if ui.c["joystick_signs"][0] < 0 else "+YAW", muted)
    img.draw_string(16, 352, "LIFT FINGER TO RESTART" if ui.blocked else "Hold joystick; max 8s per touch", amber if ui.blocked else muted)
    x, y = snapshot["angles"]
    valid = snapshot["reference_valid"] and snapshot.get("control_feedback_seen", True)
    img.draw_string(352, 120, "YAW   %7.2f deg" % x if valid else "YAW   --", white, scale=1.2)
    img.draw_string(352, 150, "PITCH %7.2f deg" % y if valid else "PITCH --", white, scale=1.2)
    raw = snapshot["raw_feedback"]
    mech = snapshot["mechanical_angles"]
    for i, label in enumerate(("TOTAL UNUSED", "MECH")):
        values = raw if i == 0 else mech
        text = " / ".join("--" if v is None else "%.1f" % v for v in values)
        img.draw_string(352, 194+i*26, label + " " + text, muted)
    img.draw_string(352, 252, "REF " + ("VALID" if snapshot["reference_valid"] else "INVALID"), cyan if snapshot["reference_valid"] else amber)
    img.draw_string(352, 278, "ZERO " + ("USER CONFIRMED" if snapshot["zero_confirmed"] else "NOT CONFIRMED"), muted)
    age = snapshot["age"]
    img.draw_string(352, 304, "FEEDBACK AGE --" if age is None else "FEEDBACK AGE %.2fs" % age, muted)
    labels = {"arm": "ARM", "stop": "DISARM", "zero": "ZERO OK", "home": "HOME", "recheck": "RECHECK STATUS"}
    for name, (bx, by, bw, bh) in BUTTONS.items():
        color = rgb(145, 40, 45) if name == "stop" else rgb(35, 65, 85)
        img.draw_rect(bx, by, bw, bh, color, -1)
        img.draw_string(bx+12, by+15, labels[name], white, scale=1.2)
    message = snapshot["fault"] or notice or "HOME holds zero ARMED; RECHECK while DISABLED."
    img.draw_string(16, 451, str(message)[:78], amber)
    disp.show(img)


def main():
    from maix import app, display, touchscreen, sys
    if str(sys.device_id()).lower() != "maixcam2":
        raise RuntimeError("This UART3 pin mapping is for MaixCAM2")
    validate_config(CONFIG)
    motor = MechanicalOnlyService(dict(CONFIG))
    disp = ts = None
    ui = JoystickUI(CONFIG)
    try:
        disp = display.Display()
        ts = touchscreen.TouchScreen()
        print("MOTOR_JOYSTICK_START UART3 B2->motor RX, B3<-motor TX; 115200 8N1; common GND")
        print("START DISABLED. ZERO OK confirms existing physical calibration; does not change it.")
        print("MECHANICAL ONLY: mode=%d position=0x03 feedback=2; yaw input reversed; pitch -100..30" % CONFIG["position_mode"])
        motor.start()
        last_draw = last_log = 0.0
        notice = "Verify saved physical zero, then ZERO OK -> ARM"
        while not app.need_exit():
            now = time.monotonic()
            x, y, pressed = ts.read()
            x, y = x*640/disp.width(), y*480/disp.height()
            vector, action = ui.touch(x, y, pressed, now)
            motor.input(*vector)
            try:
                if action == "stop":
                    motor.request_arm(False)
                    notice = "DISARM requested; torque-off has no feedback ACK"
                elif action == "zero":
                    motor.confirm_reference()
                    notice = "User confirmed saved physical zero; no 0x0A/0x08 sent"
                elif action == "arm":
                    motor.request_arm(True)
                    notice = "ARM requested at current measured position"
                elif action == "home":
                    motor.request_home()
                    notice = "Returning using mechanical feedback; cumulative angle untouched"
                elif action == "recheck":
                    motor.request_recheck()
                    notice = "Checking while disabled; then ZERO OK and manual ARM"
            except Exception as exc:
                notice = str(exc)
                print("MOTOR_UI_REQUEST_REJECTED", notice)
            snapshot = motor.snapshot()
            if now-last_draw >= 0.08:
                draw_screen(disp, snapshot, ui, notice)
                last_draw = now
            if now-last_log >= 1:
                print("MOTOR_JOYSTICK " + json.dumps(snapshot), flush=True)
                last_log = now
            time.sleep(0.01)
    finally:
        motor.close()
        for resource in (ts, disp):
            if resource is not None and hasattr(resource, "close"):
                try:
                    resource.close()
                except Exception as exc:
                    print("MOTOR_UI_CLOSE_FAILED", str(exc))
        print("MOTOR_JOYSTICK_STOP (disable requests sent; not electrical power cutoff)")


if __name__ == "__main__":
    main()
