"""Load-aware drive profiles and measured traces; never saves motor parameters."""
from collections import deque


# These retain the existing drive settings. They are NOT measured PID presets.
# Rates/accelerations are yaw, pitch; edit each stage independently after testing.
TUNING_CONFIG = {
    "arm_order": (1, 0),          # Establish pitch holding torque first.
    "startup_auto_home": True,  # User requested fast return to SAVED mechanical zero.
    "startup_hold_s": 0.0,      # No extra commissioning hold delay.
    "startup_timeout_s": 30.0,
    "startup_settle_s": 0.2,
    "record_seconds": 60.0,
    "record_samples": 2400,
    "profiles": {
        "startup": {"rpm": (5, 5), "acceleration": (10, 10),
                    "rate": (24.0, 16.0), "target_acceleration": (80.0, 60.0)},
        "manual": {"rpm": (5, 5), "acceleration": (10, 10),
                   "rate": (24.0, 16.0), "target_acceleration": (80.0, 60.0)},
        "home": {"rpm": (5, 5), "acceleration": (10, 10),
                 "rate": (10.0, 8.0), "target_acceleration": (80.0, 60.0)},
    },
}


def validate_tuning_config(c, tuning):
    if tuple(sorted(tuning["arm_order"])) != (0, 1):
        raise ValueError("arm_order must contain yaw=0 and pitch=1 exactly once")
    if not 0 <= tuning["startup_hold_s"] <= 3:
        raise ValueError("startup_hold_s must be in 0..3 seconds")
    if type(tuning["startup_auto_home"]) is not bool:
        raise ValueError("startup_auto_home must be a boolean")
    if not 1 <= tuning["startup_timeout_s"] <= 60 or not .1 <= tuning["startup_settle_s"] <= 1:
        raise ValueError("Invalid startup timeout or settle time")
    if not 1 <= tuning["record_seconds"] <= 120 or not 100 <= tuning["record_samples"] <= 6000:
        raise ValueError("Invalid bounded recording size")
    for name in ("startup", "manual", "home"):
        profile = tuning["profiles"][name]
        for key in ("rpm", "acceleration", "rate", "target_acceleration"):
            if len(profile[key]) != 2:
                raise ValueError("Profiles require one value per axis")
        for axis in range(2):
            rpm, acceleration = profile["rpm"][axis], profile["acceleration"][axis]
            rate, target_accel = profile["rate"][axis], profile["target_acceleration"][axis]
            if type(rpm) is not int or not 1 <= rpm <= c["motor_rpm"]:
                raise ValueError("Profile RPM must respect the existing motor speed cap")
            if type(acceleration) is not int or not 1 <= acceleration <= 30:
                raise ValueError("Profile drive acceleration must be in 1..30")
            if not math.isfinite(rate) or not 0 < rate <= rpm*6:
                raise ValueError("Profile target rate exceeds motor RPM")
            if not math.isfinite(target_accel) or not 0 < target_accel <= 100:
                raise ValueError("Profile software acceleration must be in 0..100")


def summarize_tuning_trace(samples):
    """Sampling statistics, not a fitted plant model or an autotuning result."""
    if len(samples) < 2:
        return {"samples": len(samples), "axes": []}
    duration = samples[-1]["time_s"]-samples[0]["time_s"]
    gaps = [b["time_s"]-a["time_s"] for a, b in zip(samples, samples[1:])]
    result = {"samples": len(samples), "duration_s": duration,
              "sample_rate_hz": (len(samples)-1)/duration if duration > 0 else None,
              "max_gap_s": max(gaps), "axes": []}
    tail = [row for row in samples if row["time_s"] >= samples[-1]["time_s"]-1.0]
    for axis in range(2):
        active = [r for r in samples if r["armed"] and r["target_deg"] is not None]
        errors = [r["angle_deg"][axis]-r["target_deg"][axis] for r in active]
        # A stationary end segment needs observed coverage, a fixed sent target,
        # torque requested throughout, and no sampling gap hidden in the window.
        holding = (len(tail) >= 3 and tail[-1]["time_s"]-tail[0]["time_s"] >= .6
                   and all(r["armed"] and r["target_deg"] is not None for r in tail)
                   and max(b["time_s"]-a["time_s"] for a, b in zip(tail, tail[1:])) <= .2)
        if holding:
            targets = [r["target_deg"][axis] for r in tail]
            holding = max(targets)-min(targets) < .051
        values = [r["angle_deg"][axis] for r in tail]
        result["axes"].append({
            "axis": ("YAW", "PITCH")[axis],
            "tracking_rmse_deg": math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
            "tracking_peak_error_deg": max(map(abs, errors)) if errors else None,
            "holding_window_s": tail[-1]["time_s"]-tail[0]["time_s"] if holding else None,
            "holding_peak_to_peak_deg": max(values)-min(values) if holding else None,
            "holding_mean_error_deg": sum(v-t for v, t in zip(values, targets))/len(values) if holding else None,
        })
    return result


class TuningMotorService(MechanicalOnlyService):
    """Same UART owner and mechanical guards; PI can change at a stationary hold."""
    def __init__(self, c, transport_factory=None, tuning=None):
        import copy
        self.tuning_config = copy.deepcopy(TUNING_CONFIG if tuning is None else tuning)
        validate_tuning_config(c, self.tuning_config)
        c = dict(c)
        c["max_rate"] = tuple(self.tuning_config["profiles"]["manual"]["rate"])
        c["home_rate"] = tuple(self.tuning_config["profiles"]["home"]["rate"])
        super().__init__(c, transport_factory)
        self.drive_profile = None
        self._drive_values = [None, None]
        self.pi_last_sent = [None, None]  # No documented PI readback command.
        self.pi_status = "电机 PI 未读取；输入框不是实机值"
        self.pi_pending = None
        self.pi_applying = False
        self.history = deque(maxlen=64)
        self.trace_lock = threading.RLock()
        self.trace = None
        self.latest_tuning_sample = None
        self.sample_seq = 0
        self.startup_attempted = False
        self.startup_active = False
        self.startup_waiting_owner = False
        self.startup_deadline = None
        self.startup_status = "PENDING" if self.tuning_config["startup_auto_home"] else "OFF"
        self.pi_autotune = F32CPiAutoTuner(self)
        self.autotune_request = None

    @property
    def autotune_active(self):
        return self.pi_autotune.active or self.autotune_request is not None

    def request_pi_autotune(self, axis, kp, ki):
        if type(axis) is not int or axis not in (0, 1) or any(type(v) is not int or not 0 <= v <= 256 for v in (kp, ki)):
            raise ValueError("Choose an axis and supply the known KP/KI baseline")
        if self.autotune_active or not self.armed or self.home_requested or any(self.joystick):
            raise RuntimeError("Wait for an enabled stationary hold")
        self.pi_autotune.check_start(axis, kp, ki)
        self.pi_autotune.detached = False
        self.autotune_request = (axis, kp, ki)

    def request_home(self):
        if self.autotune_active:
            raise RuntimeError("Cancel PI auto tune before HOME")
        super().request_home()

    def cancel_pi_autotune(self):
        self.autotune_request = None
        self.pi_autotune.cancel(time.monotonic())

    def _manual_phase(self, moving):
        if self.pi_autotune.detached and self.pi_autotune.session.state in ("COMPLETE", "CANCELLED"):
            return "AUTOTUNE_HOLD"
        return super()._manual_phase(moving)

    def snapshot(self):
        result = super().snapshot()
        with self.lock:
            tuning = dict(pi_last_sent=list(self.pi_last_sent), pi_status=self.pi_status,
                          pi_busy=self.pi_pending is not None or self.pi_applying,
                          drive_profile=self.drive_profile, profiles=self.tuning_config["profiles"],
                          startup_hold_s=self.tuning_config["startup_hold_s"],
                          startup_status=self.startup_status)
            tuning["autotune"] = self.pi_autotune.export()
            tuning["autotune"]["active"] = self.autotune_active
        with self.trace_lock:
            tuning.update(latest=self.latest_tuning_sample, recording=bool(self.trace and self.trace["recording"]),
                          count=len(self.trace["samples"]) if self.trace else 0,
                          result=self.trace.get("summary") if self.trace else None,
                          record_reason=self.trace.get("end_reason") if self.trace else None)
        result["tuning"] = tuning
        result["arm_requested"] = self.arm_requested
        result["startup_takeover"] = self.startup_waiting_owner
        result["autotune_takeover"] = self.autotune_active and self.pi_autotune.detached
        return result

    def input(self, x, y):
        if self.autotune_active and (x or y):
            raise RuntimeError("Cancel PI auto tune and wait for rollback before manual movement")
        if self.phase == "ARMING_HOLD":
            x = y = 0.0  # Never queue a joystick target during startup holding.
        super().input(x, y)

    def request_arm(self, armed):
        if armed:
            with self.lock:
                if self.pi_pending is not None or self.pi_applying:
                    raise RuntimeError("Wait for temporary PI writes before ARM")
        else:
            self.autotune_request = None
            self.pi_autotune.interrupted("DISARM interrupted PI auto tune; baseline restore pending")
            self.startup_waiting_owner = False
            with self.lock:
                if self.pi_pending is not None:
                    self.pi_pending = None
                    self.pi_status = "临时 PI 请求已取消，未发送"
        super().request_arm(armed)

    def request_recheck(self):
        with self.lock:
            if self.pi_pending is not None or self.pi_applying:
                raise RuntimeError("Wait for temporary PI writes before RECHECK")
        super().request_recheck()

    def claim_startup_hold(self):
        """Transfer an autonomous zero hold to a real controller, without DISARM."""
        with self.lock:
            if not (self.startup_waiting_owner and self.armed and self.ready
                    and self.reference_valid and not self.fault and self.phase in ("HOME_HOLD", "AUTOTUNE_HOLD")):
                return False
            self.command_time = time.monotonic()
            self.startup_waiting_owner = False
            self.pi_autotune.detached = False
            return True

    def _heartbeat_check(self):
        if self.pi_autotune.active:
            if self.abort_requested or self.stop_event.is_set() or not self.arm_requested:
                raise UserDisarm("PI auto tune stopped")
            return  # Session owns a bounded, local experiment clock.
        if self.startup_active or self.startup_waiting_owner:
            if self.abort_requested or self.stop_event.is_set() or not self.arm_requested:
                raise UserDisarm("Autonomous startup/hold cancelled")
            if self.startup_active and time.monotonic() >= self.startup_deadline:
                raise RuntimeError("Startup HOME timeout")
            return  # Local autonomous transaction/hold, not a synthetic browser heartbeat.
        super()._heartbeat_check()

    def _control_loop(self):
        if not self.startup_attempted:
            self.startup_attempted = True
            if self.tuning_config["startup_auto_home"]:
                if self.abort_requested or self.stop_event.is_set():
                    self.startup_status = "CANCELLED"
                    return super()._control_loop()
                self.startup_active = True
                self.startup_status = "HOMING"
                self.startup_deadline = time.monotonic()+self.tuning_config["startup_timeout_s"]
                self.abort_requested = False
                self.zero_confirmed = True  # Explicit configuration trusts the saved zero.
                self.arm_requested = True
                try:
                    self.manual_target = self.sent_target = self._arm_current()
                    self.armed = True
                    self.last_send = time.monotonic()
                    self.home_requested = True
                    settle = self.c["home_settle_s"]
                    try:
                        self.c["home_settle_s"] = self.tuning_config["startup_settle_s"]
                        self._home_and_clear()
                    finally:
                        self.c["home_settle_s"] = settle
                    self.startup_waiting_owner = True
                    self.startup_status = "HOME_HOLD"
                except UserDisarm:
                    self.startup_status = "CANCELLED"
                    self._disable_now()
                    self.phase = "DISABLED"
                finally:
                    self.startup_active = False
        return super()._control_loop()

    def request_speed_pi(self, axis, kp=None, ki=None):
        if self.autotune_active:
            raise RuntimeError("Wait for PI auto tune to finish before changing gains")
        if type(axis) is not int or axis not in (0, 1):
            raise ValueError("Choose YAW or PITCH")
        if (kp is None and ki is None) or any(v is not None and (type(v) is not int or not 0 <= v <= 256) for v in (kp, ki)):
            raise ValueError("KP/KI must be integers in 0..256")
        with self.lock:
            if (self.fault or not self.ready or not self.reference_valid or self.startup_active
                    or self.home_requested or any(self.joystick)
                    or self.phase not in ("DISABLED", "ARMED", "HOME_HOLD")):
                raise RuntimeError("Apply PI at a stationary hold or while DISABLED")
            if self.pi_pending is not None or self.pi_applying:
                raise RuntimeError("A PI request is already pending")
            self.pi_pending = (axis, kp, ki)
            self.abort_requested = False  # Explicit new request after connect/DISARM.
            self.pi_status = "等待串口线程检查静止并发送临时 PI"

    def _write(self, data):
        if data[2] not in (0x0F, 0x10):
            return super()._write(data)
        time.sleep(.002)
        self._require_reference(fresh=True)
        if self.armed:
            self._heartbeat_check()
        if (not self.pi_applying or self.home_requested or any(self.joystick) or self.abort_requested
                or self.stop_event.is_set()):
            raise UserDisarm("PI write cancelled")
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete PI UART write; parameter state unknown")

    def _process_pi(self):
        with self.lock:
            request, self.pi_pending = self.pi_pending, None
            if request is None:
                return
            self.pi_applying = True
        axis, kp, ki = request
        wrote = False
        try:
            if self.home_requested or any(self.joystick) or self.abort_requested or self.startup_active:
                raise UserDisarm("PI request cancelled")
            # Require fresh stationary mechanical observations of BOTH axes.
            now = time.monotonic()
            with self.lock:
                recent = [s for s in self.history if s[0] >= now-.45]
            if (len(recent) < 3 or recent[-1][0]-recent[0][0] < .3
                    or max(b[0]-a[0] for a, b in zip(recent, recent[1:])) > .2):
                raise ValueError("Keep supported axes stationary for at least 0.3 s, then retry")
            if any(max(s[1][a] for s in recent)-min(s[1][a] for s in recent) > .2 for a in range(2)):
                raise ValueError("Axes are moving; support the load before changing PI")
            mid = self.c["motor_ids"][axis]
            if self._feedback(mid, FEEDBACK_SPEED) != 0:
                raise ValueError("Selected axis speed is not zero")
            # Validate the complete pair before sending either parameter.
            old = self.pi_last_sent[axis] or [None, None]
            updated = list(old)
            self.pi_last_sent[axis] = None
            wrote = True  # Even an incomplete first write can change device state.
            for index, (opcode, value) in enumerate(((0x0F, kp), (0x10, ki))):
                if value is not None:
                    self._write(_frame(mid, opcode, value.to_bytes(2, "big")))
                    updated[index] = value
            self.pi_last_sent[axis] = updated
            self.pi_status = "%s 临时 PI 已发送：KP=%s / KI=%s，无参数 ACK/回读；未保存" % (("YAW", "PITCH")[axis], updated[0], updated[1])
        except (ValueError, UserDisarm) as exc:
            self.pi_status = str(exc) + ("；参数状态未知" if wrote else "；未发送 PI")
            if wrote:
                raise
        except Exception:
            self.pi_status = "PI 通信失败，参数状态未知；未保存"
            raise
        finally:
            with self.lock:
                self.pi_applying = False

    def _profile(self, name):
        profile = self.tuning_config["profiles"][name]
        for axis, mid in enumerate(self.c["motor_ids"]):
            values = (profile["rpm"][axis], profile["acceleration"][axis])
            if self._drive_values[axis] != values:
                self._heartbeat_check()
                self._write(set_speed(mid, values[0]))
                self._heartbeat_check()
                self._write(set_acceleration(mid, values[1]))
                self._drive_values[axis] = values
        self.c["command_accel_deg_s2"] = tuple(profile["target_acceleration"])
        self.drive_profile = name
        return profile

    def _arm_current(self):
        _, angles = self._measure()
        for mid in self.c["motor_ids"]:
            self._write(select_mode(mid, self.c["position_mode"]))
        self._drive_values = [None, None]  # Reapply after possible motor power cycle.
        self._profile("startup")
        for mid, angle in zip(self.c["motor_ids"], self.mechanical):
            self._write(set_single_turn_position(mid, angle))
        self.sent_target = self.manual_target = angles
        for axis in self.tuning_config["arm_order"]:
            self._write(enable(self.c["motor_ids"][axis]))
        self.phase = "ARMING_HOLD"
        deadline = time.monotonic()+self.tuning_config["startup_hold_s"]
        while time.monotonic() < deadline:
            self._heartbeat_check()
            self._measure()
            if self.stop_event.wait(.02):
                raise UserDisarm("Startup hold cancelled")
        self.stream_target = angles
        self.stream_velocity = (0.0, 0.0)
        # Cache the actual preloaded target, never a later sagged feedback pose.
        self.position_units = [int.from_bytes(set_single_turn_position(mid, q/sign)[3:5], "big")
                               for mid, q, sign in zip(self.c["motor_ids"], angles, self.c["motor_signs"])]
        self.last_position_send = time.monotonic()
        self.hold_pending = False
        self.guard.reset()
        return angles

    def _advance(self, goal, rate):
        if self.autotune_request is not None:
            request, self.autotune_request = self.autotune_request, None
            self.pi_autotune.start(*request, time.monotonic())
        if self.pi_autotune.active:
            self.pi_autotune.advance(time.monotonic())
            return
        name = "startup" if self.startup_active else "home" if self.phase in ("RETURNING", "SETTLING", "HOME_HOLD") else "manual"
        profile = self._profile(name)
        return super()._advance(goal, profile["rate"])

    def _measure(self):
        result = super()._measure()
        now = time.monotonic()
        with self.lock:
            self.history.append((now, self.angles))
        self._record_sample(now)
        if self.ready:
            self._process_pi()  # This method is called only by the UART owner.
        return result

    def start_tuning_capture(self, note=""):
        if not isinstance(note, str) or len(note) > 200:
            raise ValueError("Recording note must be at most 200 characters")
        state = super().snapshot()
        if not state["ready"] or state["fault"] or not state["reference_valid"]:
            raise RuntimeError("Valid mechanical feedback required before recording")
        with self.trace_lock:
            if self.trace and self.trace["recording"]:
                raise RuntimeError("Stop the current recording first")
            self.trace = dict(schema=1, note=note, recording=True, samples=[],
                              started_monotonic_s=time.monotonic(), started_unix_s=time.time(),
                              limits=self.c["limits"], motor_ids=self.c["motor_ids"],
                              profiles=self.tuning_config["profiles"],
                              position_mode=self.c["position_mode"],
                              pi_source="last commands sent, NOT device readback",
                              angle_resolution_deg=.1, summary=None, end_reason=None)

    def stop_tuning_capture(self, reason="manual"):
        with self.trace_lock:
            if self.trace and self.trace["recording"]:
                self.trace["recording"] = False
                self.trace["end_reason"] = reason
                self.trace["summary"] = summarize_tuning_trace(self.trace["samples"])

    def _record_sample(self, now):
        self.sample_seq += 1
        row = dict(seq=self.sample_seq, time_s=now,
                   feedback_time_s=[s[0] if s else None for s in self.mechanical_samples],
                   angle_deg=self.angles, target_deg=self.sent_target,
                   armed=bool(self.armed or (self.arm_requested and self.phase == "ARMING_HOLD")),
                   phase=self.phase, profile=self.drive_profile,
                   input=self.joystick, pi_last_sent=[list(p) if p else None for p in self.pi_last_sent])
        with self.trace_lock:
            self.latest_tuning_sample = row
            if self.trace and self.trace["recording"]:
                self.trace["samples"].append(row)
                if (now-self.trace["started_monotonic_s"] >= self.tuning_config["record_seconds"]
                        or len(self.trace["samples"]) >= self.tuning_config["record_samples"]):
                    self.stop_tuning_capture("recording limit")

    def export_tuning(self):
        with self.trace_lock:
            if not self.trace:
                if hasattr(self.pi_autotune.session, "baseline"):
                    return dict(pi_autotune=self.pi_autotune.export())
                raise ValueError("Start a recording or PI auto tune first")
            result = dict(self.trace, samples=list(self.trace["samples"]))
        result["summary"] = summarize_tuning_trace(result["samples"])
        result["pi_autotune"] = self.pi_autotune.export()
        return result

    def _record_fault(self, exc):
        self.autotune_request = None
        self.pi_autotune.interrupted(str(exc))
        self.startup_attempted = True
        self.startup_active = self.startup_waiting_owner = False
        self.startup_status = "FAULT"
        self.stop_tuning_capture("fault: " + str(exc))
        super()._record_fault(exc)

    def close(self):
        self.stop_tuning_capture("application closed")
        super().close()
