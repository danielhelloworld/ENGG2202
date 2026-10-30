"""Arbitrate local/web control; network heartbeats never feed the UART worker."""
from collections import deque


class MotorMailbox:
    def __init__(self, hub, driver):
        self.hub, self.driver, self.c = hub, driver, driver.c

    def snapshot(self):
        return self.driver.snapshot()

    @property
    def arm_requested(self):
        return self.driver.arm_requested

    def input(self, x, y):
        state = self.driver.snapshot()
        if (x or y) and (state.get("home_requested") or (self.driver.arm_requested and not state["armed"])):
            self.hub.manual = (0., 0.)
            return  # Do not queue a manual move behind HOME.
        self.hub.manual = (x, y)
        if x or y:
            self.hub.tracking.clear()

    def request_arm(self, armed):
        if armed and not self.hub.fresh():
            raise RuntimeError("Fresh camera/inference required before ARM")
        if not armed:
            self.hub.tracking.clear()
            self.hub.manual = (0., 0.)
        self.driver.request_arm(armed)
        if armed:
            self.hub.tracking.clear()
            self.hub.manual = (0., 0.)
            self.hub.notice = "ARM：使能并保持当前位置；HOME 单独回机械原点"

    def confirm_reference(self):
        self.driver.confirm_reference()

    def request_home(self):
        if not self.hub.fresh():
            raise RuntimeError("Fresh camera/inference required before HOME")
        self.driver.request_home()
        self.hub.tracking.clear()
        self.hub.manual = (0., 0.)

    def request_recheck(self):
        self.driver.request_recheck()
        self.hub.tracking.clear()
        self.hub.manual = (0., 0.)


class TrackingHub(BrowserControl):
    def __init__(self, driver, c, clock=time.monotonic):
        self.driver, self.c = driver, c
        self.tracking = ClickTracker(c)
        self.manual = (0., 0.)
        self.local_owner = True
        self.frames = deque(maxlen=c["frame_history"])
        self.latest = self.video = None
        self.performance = {}
        self.detached_token = None
        self.frame_id = 0
        super().__init__(MotorMailbox(self, driver), clock, WEB_CONFIG["lease_s"])
        if c["mechanical_zero_confirmed"]:
            self.notice = "使用驱动已保存机械零点；开机自动回零保持" if c["startup_auto_home"] else "使用已保存机械零点；ARM 保持，HOME 单独回零"

    def recheck(self, token):
        value = super().recheck(token)
        if self.c["mechanical_zero_confirmed"]:
            self.notice = "重新检查中；通过后继续使用已保存零点，手动 ARM 或 HOME"
            value["notice"] = self.notice
        return value

    def fresh(self):
        return self.latest is not None and 0 <= self.clock()-self.latest["stamp"] <= self.c["vision_max_age"]

    def poll(self):
        with self.lock:
            if self.token and self.clock() >= self.deadline:
                self.background(self.token)
                self.notice = "网页连接超时：手动输入已松开，相机继续跟踪／回零或保持"

    def telemetry(self):
        with self.lock:
            data = self.driver.telemetry()
            data.update(state=self.state(), visual_target=self.tracking.selected)
            return data

    def connect(self):
        with self.lock:
            if not self.fresh():
                raise ControlError("Waiting for fresh YOLO frames")
            self.poll()
            state = self.driver.snapshot()
            if (not self.token and self.local_owner and state["armed"] and self.driver.arm_requested
                    and state["reference_valid"] and not state["fault"] and not any(self.manual)):
                # Take over onboard tracking/home/hold without disarming or clearing ID.
                self.token = secrets.token_urlsafe(24)
                self.detached_token = None
                self.deadline, self.seq = self.clock()+self.lease_s, 0
                self.local_owner = False
                self.notice = "网页已接管；保持当前跟踪／回零／保持状态"
                return {"token": self.token, "state": self.state()}
            result = super().connect()
            self.detached_token = None
            self.local_owner = False
            return result

    def background(self, token):
        with self.lock:
            if not token or token != self.token:
                raise ControlError("控制连接已失效", 403)
            # Revoke the old session before accepting any later queued key command.
            self.detached_token = self.token
            self.token = None
            self.hold_start, self.blocked = None, False
            self.manual = (0., 0.)
            self.local_owner = True
            self.notice = "网页已离开焦点，手动输入已松开；相机继续跟踪／回零或保持"
            return self.state()

    def stop(self, token):
        with self.lock:
            # An explicit STOP racing the background handoff must work on either
            # side of revocation, but cannot stop a newly connected controller.
            if not self.token and token and token == self.detached_token:
                token = None
            result = super().stop(token)
            self.detached_token = None
            return result

    def state(self):
        with self.lock:
            value = super().state()
            value.update(vision_state=self.tracking.state, selected=self.tracking.selected,
                         c_next_home=self.tracking.c_next_home,
                         frame_id=self.latest["id"] if self.latest else None,
                         vision_age=None if self.latest is None else self.clock()-self.latest["stamp"],
                         prediction_active=self.tracking.prediction_active,
                         prediction_center=self.tracking.prediction_point,
                         estimated_target_velocity=self.tracking.predictor.motion_velocity,
                         performance=dict(self.performance),
                         control_owner="WEB" if self.token else "LOCAL" if self.local_owner else "NONE")
            return value

    def offer_frame(self, tracks, width, height, stamp, capture_pose=...):
        with self.lock:
            if not (width > 0 and height > 0 and math.isfinite(stamp)):
                raise ValueError("Invalid frame")
            bounded = []
            for t in tracks[:self.c["max_tracks"]]:
                x, y, w, h = t["box"]
                if not all(math.isfinite(v) for v in (x, y, w, h)) or w <= 0 or h <= 0:
                    continue
                tr = dict(t)
                tr["key"] = tuple(t["key"])
                tr["cx"], tr["cy"] = (x+w/2)/width, (y+h/2)/height
                tr["norm_box"] = (x/width, y/height, w/width, h/height)
                bounded.append(tr)
            self.frame_id += 1
            frame = dict(id=self.frame_id, stamp=stamp, width=width, height=height, tracks=bounded)
            self.latest = frame
            self.frames.append(frame)
            if self.fresh():
                motor = self.driver.snapshot()
                if motor["armed"] and not motor["fault"] and not motor.get("home_requested"):
                    if capture_pose is Ellipsis:
                        capture_pose = motor["angles"] if motor["reference_valid"] and not motor.get("feedback_degraded") and motor["age"] is not None and motor["age"] <= self.c["pose_max_age_s"] else None
                    event = self.tracking.step(bounded, stamp, capture_pose, max(0., self.clock()-stamp))
                    if event == "home":
                        self.motor.request_home()
                        self.notice = "目标丢失超过 2 秒，按 1101 逻辑释放并回原点"
            return frame

    def publish_media(self, frame, jpeg):
        with self.lock:
            self.video = dict(id=frame["id"], stamp=frame["stamp"], jpeg=bytes(jpeg))

    def media(self):
        with self.lock:
            if self.video is None or self.clock()-self.video["stamp"] > self.c["vision_max_age"]:
                return None
            return self.video

    def select_at(self, frame_id, nx, ny):
        if type(frame_id) is not int or any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 1 for v in (nx, ny)):
            raise ControlError("Invalid click coordinates", 400)
        motor = self.driver.snapshot()
        if not motor["armed"] or motor["fault"] or motor.get("home_requested") or motor.get("outside_hold") or not self.fresh():
            raise ControlError("先 ARM，等待 HOME 完成，并确认实时画面")
        frame = next((f for f in self.frames if f["id"] == frame_id), None)
        if frame is None or self.clock()-frame["stamp"] > self.c["vision_max_age"]:
            raise ControlError("点击画面已过期，请在最新画面重新选择")
        hits = []
        for tr in frame["tracks"]:
            x, y, w, h = tr["norm_box"]
            if x <= nx <= x+w and y <= ny <= y+h:
                hits.append(tr)
        if not hits:
            raise ControlError("请点击检测框内")
        chosen = min(hits, key=lambda t: t["norm_box"][2]*t["norm_box"][3])
        try:
            self.tracking.select(chosen["key"], self.latest["tracks"], self.clock())
        except RuntimeError as exc:
            raise ControlError(str(exc))
        self.manual = (0., 0.)
        self.notice = "已锁定 %s ID=%s" % (chosen["label"], chosen["key"])

    def c_action(self):
        if self.tracking.c_next_home:
            self.motor.request_home()
            self.notice = "第二次 C：返回原点，到位后保持使能"
        else:
            self.tracking.clear(arm_c=True)
            self.manual = (0., 0.)
            self.notice = "已清除目标并保持当前位置；再按一次 C 返回原点"

    def command(self, body):
        with self.lock:
            action = body.get("action")
            if action not in ("select", "c"):
                value = super().command(body)
                if action == "arm":
                    self.notice = "ARM：使能并保持当前位置；HOME 单独回机械原点"
                    value["notice"] = self.notice
                return value
            if body.get("x") != 0 or body.get("y") != 0:
                raise ControlError("松开摇杆后选择目标或按 C", 400)
            request = dict(body, action="")
            super().command(request)  # Shared token/sequence/deadline checks.
            try:
                if action == "c":
                    self.c_action()
                else:
                    self.select_at(body.get("frame_id"), body.get("nx"), body.get("ny"))
            except RuntimeError as exc:
                raise ControlError(str(exc))
            return self.state()

    def local_action(self, action=None, vector=(0., 0.), click=None):
        with self.lock:
            self.poll()
            if action == "stop":
                self._stop("屏幕 DISARM：已请求失能")
                self.local_owner = False
                return
            if self.token:
                raise ControlError("网页正在控制；先 DISARM 再使用屏幕")
            self.local_owner = True
            if action == "arm":
                self.motor.request_arm(True)
            elif action == "zero":
                self.motor.confirm_reference()
            elif action == "recheck":
                self.motor.request_recheck()
            elif action == "home":
                self.motor.request_home()
            elif action == "c":
                self.c_action()
            elif click:
                self.select_at(*click)
            else:
                self.motor.input(*vector)

    def tick(self):
        with self.lock:
            self.poll()
            motor = self.driver.snapshot()
            if motor["fault"]:
                self.tracking.clear()
                self.manual = (0., 0.)
            if not self.fresh():
                if motor["armed"] or self.driver.arm_requested:
                    age = float("inf") if self.latest is None else self.clock()-self.latest["stamp"]
                    if age > self.c["vision_disable_age_s"]:
                        self.motor.request_arm(False)
                        self.notice = "相机／YOLO 持续无有效画面，已请求失能"
                    else:
                        if hasattr(self.driver, "set_response_context"):
                            self.driver.set_response_context("HOLD")
                        self.driver.input(0., 0.)
                        self.notice = "相机／YOLO 暂时延迟，暂停运动并保持使能"
                return
            if not self.token and not self.local_owner:
                return
            if self.tracking.selected is not None:
                vector = self.tracking.drive(self.clock())
                context = self.tracking.state
            else:
                vector = tuple(v*r/cap for v,r,cap in zip(self.manual, self.c["manual_rate"], self.c["max_rate"]))
                context = "MANUAL" if any(vector) else "HOLD"
            if hasattr(self.driver, "set_response_context"):
                self.driver.set_response_context(context)
            self.driver.input(*vector)  # Only the live application loop feeds motor heartbeat.
