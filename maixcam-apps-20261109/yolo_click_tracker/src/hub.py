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

    def request_arm(self, armed, reason="手动 DISARM／停止请求"):
        if armed and not self.hub.fresh():
            raise RuntimeError("Fresh camera/inference required before ARM")
        if not armed:
            if self.hub.software_autotune.active:
                self.hub.software_autotune.abort("DISARM requested")
            self.hub.tracking.clear()
            self.hub.manual = (0., 0.)
        if hasattr(self.driver,"disable_history"):
            self.driver.request_arm(armed,reason=reason)
        else:
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
        self.preview_source = None
        self.jpeg_lock = threading.Lock()
        self.last_jpeg_attempt = float("-inf")
        self.performance = {}
        self.timings = StageTimings()
        self.detached_token = None
        self.last_control_rejection = None
        self.frame_id = 0
        super().__init__(MotorMailbox(self, driver), clock, WEB_CONFIG["lease_s"])
        self.software_autotune = SoftwareAutoTuner(self)
        if c["mechanical_zero_confirmed"]:
            self.notice = "使用驱动已保存机械零点；开机自动回零保持" if c["startup_auto_home"] else "使用已保存机械零点；ARM 保持，HOME 单独回零"

    def recheck(self, token):
        value = super().recheck(token)
        if self.c["mechanical_zero_confirmed"]:
            self.notice = "重新检查中；通过后继续使用已保存零点，手动 ARM 或 HOME"
            value["notice"] = self.notice
        return value

    def _disable_request(self, reason):
        self.motor.request_arm(False, reason=reason)

    def _reject_command(self, reason, status=409):
        # A repeated/busy action or mixed button/stick packet is a recoverable
        # operator conflict, not a reason to drop a gravity-loaded camera.
        self.motor.input(0., 0.)
        self.driver.input(0., 0.)
        self.hold_start, self.blocked = None, False
        self.deadline = self.clock()+self.lease_s
        self.last_control_rejection = dict(reason=reason,time=self.clock())
        self.notice = "操作未接受，已松开手动输入并保留使能："+reason
        raise ControlError(self.notice, status)

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
                         last_control_rejection=self.last_control_rejection,
                         software_autotune=dict(active=self.software_autotune.active,
                                                state=self.software_autotune.session.state,
                                                reason=self.software_autotune.session.reason,
                                                saved_path=self.software_autotune.saved_path),
                         c_next_home=self.tracking.c_next_home,
                         frame_id=self.latest["id"] if self.latest else None,
                         vision_age=None if self.latest is None else self.clock()-self.latest["stamp"],
                         prediction_active=self.tracking.prediction_active,
                         prediction_center=self.tracking.prediction_point,
                         tracking_error_deg=tuple(e*f for e,f in zip(self.tracking.control_error,self.c['camera_fov'])),
                         tracking_aim_deg=self.tracking.aim_goal, tracking_centered=tuple(self.tracking.settled),
                         overhead=dict(self.tracking.overhead.snapshot(),reacquired=self.tracking.overhead_reacquired),
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
                    current_pose = motor["angles"] if motor["reference_valid"] and not motor.get("feedback_degraded") and motor["age"] is not None and motor["age"] <= self.c["pose_max_age_s"] else None
                    event = self.tracking.step(bounded, stamp, capture_pose, max(0., self.clock()-stamp),current_pose)
                    self.software_autotune.observe(frame, motor)
                    if event == "home":
                        self.motor.request_home()
                        self.notice = "目标丢失超过 2 秒，按 1101 逻辑释放并回原点"
            return frame

    def publish_media(self, frame, jpeg):
        with self.lock:
            self.video = dict(id=frame["id"], stamp=frame["stamp"], jpeg=bytes(jpeg))

    def offer_preview_image(self, frame, image):
        # Runtime hands off a finished overlay. Keep only the newest source.
        with self.lock:
            self.preview_source = (frame, image)

    def close_preview(self):
        with self.lock:
            self.preview_source = None
        with self.jpeg_lock:
            pass  # Let any in-flight encode release its image before camera teardown.

    def media(self):
        with self.lock:
            now = self.clock()
            cached = self.video
            if cached is not None and now-cached["stamp"] > self.c["vision_max_age"]:
                cached = None
            source = self.preview_source
            if (source is None or now-source[0]["stamp"] > self.c["vision_max_age"]
                    or (cached is not None and cached["id"] == source[0]["id"])
                    or now-self.last_jpeg_attempt < 1/self.c["web_preview_fps"]
                    or not self.jpeg_lock.acquire(blocking=False)):
                return cached
            self.last_jpeg_attempt = now
        try:
            frame, image = source
            started = self.clock()
            jpeg = image.to_jpeg(self.c["jpeg_quality"]).to_bytes()
            self.timings.add("web_encode", (self.clock()-started)*1000)
            with self.lock:
                self.performance["web_encode_ms"] = (self.clock()-started)*1000
                if self.clock()-frame["stamp"] > self.c["vision_max_age"]:
                    return None
                self.publish_media(frame, jpeg)
                return self.video
        finally:
            self.jpeg_lock.release()

    def select_at(self, frame_id, nx, ny):
        if type(frame_id) is not int or any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 1 for v in (nx, ny)):
            raise ControlError("Invalid click coordinates", 400)
        motor = self.driver.snapshot()
        if motor.get('angle_move',{}).get('active'):
            raise ControlError("请先停止角度测试，再选择跟踪目标")
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
        if hasattr(self.driver,'cancel_angle_move'):
            self.driver.cancel_angle_move()
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
            if action in ("pi", "record_start", "record_stop", "pi_autotune", "pi_autotune_cancel", "pi_autotune_save"):
                raise ControlError("使用独立电机调试程序调整 PI", 400)
            if self.software_autotune.active and (body.get("x") or body.get("y") or action in ("arm", "home", "zero")):
                self.software_autotune.abort("manual control requested")
            if action not in ("select", "c", "autotune", "autotune_cancel", "autotune_save", "angle_start", "angle_cancel", "inertia_start", "native_start"):
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
                if action in ("angle_start","inertia_start","native_start"):
                    if self.software_autotune.active:
                        raise RuntimeError("先停止软件整定")
                    if not self.fresh():
                        raise RuntimeError("相机数据暂时过期，请等待恢复")
                    if action in ('inertia_start','native_start'):
                        hold=self.driver.snapshot().get('hold_target_deg')
                        yaw=hold[0] if hold else self.driver.snapshot()['angles'][0]
                        self.driver.request_angle_move(yaw,body.get('pitch'),body.get('speed'),inertia_test=True,
                                                       native_pitch=action=='native_start',native_accel=body.get('accel',3))
                    else:
                        self.driver.request_angle_move(body.get('yaw'),body.get('pitch'),body.get('speed'))
                    self.tracking.clear();self.manual=(0.,0.)
                    self.notice="角度测试已提交；相机本地执行，到位后保持使能"
                elif action == "angle_cancel":
                    self.driver.cancel_angle_move()
                    self.notice="已请求停止角度测试并保持使能"
                elif action == "autotune":
                    if self.driver.snapshot().get('angle_move',{}).get('active'):
                        raise RuntimeError("先停止角度测试")
                    self.software_autotune.start(body.get("axis"))
                elif action == "autotune_cancel":
                    self.software_autotune.abort()
                elif action == "autotune_save":
                    self.notice = "已保存软件整定结果 " + self.software_autotune.save()
                elif action == "c":
                    if self.software_autotune.active:
                        self.software_autotune.abort()
                    self.c_action()
                else:
                    if self.software_autotune.active:
                        self.software_autotune.abort()
                    self.select_at(body.get("frame_id"), body.get("nx"), body.get("ny"))
            except (RuntimeError, ValueError, OSError) as exc:
                raise ControlError(str(exc))
            return self.state()

    def local_action(self, action=None, vector=(0., 0.), click=None):
        with self.lock:
            self.poll()
            if self.software_autotune.active and (action or click or any(vector)):
                self.software_autotune.abort("local control requested")
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
            if self.software_autotune.active and not motor["armed"]:
                self.software_autotune.abort("motor disabled")
            if motor["fault"]:
                if self.software_autotune.active:
                    self.software_autotune.abort("motor fault")
                self.tracking.clear()
                self.manual = (0., 0.)
            if motor.get('inertia_test_active'):
                self.driver.set_response_context('HOLD');self.driver.input(0.,0.)
                return  # The UART-owned test is independent of vision/PC latency.
            if not self.fresh():
                if self.software_autotune.active:
                    self.software_autotune.abort("vision feedback expired")
                if motor["armed"] or self.driver.arm_requested:
                    age = float("inf") if self.latest is None else self.clock()-self.latest["stamp"]
                    if age > self.c["vision_disable_age_s"]:
                        self.motor.request_arm(False,reason="VISION_TIMEOUT：相机／YOLO 数据 %.2fs 未更新" % age)
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
                vector = self.tracking.drive(self.clock(),motor['angles'])
                context = self.tracking.state
                if self.c['native_tracking'] and context in ('COAST','OVERHEAD') and self.tracking.aim_goal is not None:
                    context='TRACK'  # Bounded local prediction / committed zenith relay; no browser dependency.
            else:
                vector = tuple(v*r/cap for v,r,cap in zip(self.manual, self.c["manual_rate"], self.c["max_rate"]))
                context = "MANUAL" if any(vector) else "HOLD"
            if self.c['native_tracking'] and context=='TRACK' and self.tracking.aim_goal is None:
                context,vector='HOLD',(0.,0.)
            if hasattr(self.driver, "set_response_context"):
                self.driver.set_response_context(context)
            if hasattr(self.driver,'offer_tracking_goal'):
                self.driver.offer_tracking_goal(self.tracking.aim_goal if context=='TRACK' else None,
                    self.latest['stamp'],self.tracking.native_active_axes,velocity=self.tracking.aim_velocity,
                    extrapolate=self.tracking.state=='TRACK' and not self.tracking.overhead.active)
            self.driver.input(*vector)  # Only the live application loop feeds motor heartbeat.
