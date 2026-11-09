"""F32C volatile PI search, advanced exclusively by the existing UART owner."""
import json
import os


class F32CPiAutoTuner:
    def __init__(self, service):
        self.service = service
        self.axis = None
        self.origin = None
        self.target = None
        self.last_sample = None
        self.detached = False
        self.saved_path = None
        self.last_applied = None
        self.session = AutoTuneExperiment(self._apply, self._offset, amplitude=1., segment_s=1.5)

    @property
    def active(self):
        return self.session.active

    def check_start(self, axis, kp, ki):
        s = self.service
        if type(axis) is not int or axis not in (0, 1):
            raise ValueError("Choose one PI tuning axis")
        if any(type(v) is not int or not 0 <= v <= 256 for v in (kp, ki)):
            raise ValueError("Known baseline KP and KI in 0..256 are required")
        if (self.active or not s.armed or not s.arm_requested or not s.ready or s.fault
                or s.home_requested or s.startup_active or any(s.joystick)
                or s.pi_pending is not None or s.pi_applying):
            raise RuntimeError("PI auto tune requires an idle, enabled hold")
        if s.c["position_mode"] != 4:
            raise RuntimeError("PI response trials require single-turn direct mode 4")
        if not s.reference_valid or time.monotonic()-s.feedback_time > s.c["reference_max_age_s"]:
            raise RuntimeError("Fresh mechanical feedback is required")
        origin = tuple(s.sent_target)
        if any(abs(a-b) > .3 for a, b in zip(origin, s.angles)):
            raise RuntimeError("Wait for the existing target to settle")
        if any(not lo+3 <= q <= hi-3 for q, (lo, hi) in zip(origin, s.c["limits"])):
            raise RuntimeError("Move at least 3 degrees inside limits before PI auto tune")
        return origin

    def start(self, axis, kp, ki, now):
        origin = self.check_start(axis, kp, ki)
        self.axis, self.origin = axis, origin
        self.last_sample = None
        self.saved_path = None
        self.last_applied = None
        baseline = dict(kp=kp, ki=ki)
        candidates = []
        for key in ("kp", "ki"):
            for direction in (-1, 1):
                profile = dict(baseline)
                profile[key] = max(0, min(256, profile[key]+direction*max(1, round(profile[key]*.15))))
                if profile != baseline and profile not in candidates:
                    candidates.append(profile)
        self.session.start(baseline, candidates, now)

    def _apply(self, profile):
        s = self.service
        s._heartbeat_check()
        s._require_reference(fresh=True)
        mid = s.c["motor_ids"][self.axis]
        if s._feedback(mid, FEEDBACK_SPEED) != 0:
            raise RuntimeError("Motor is still moving; PI profile not applied")
        old = self.last_applied
        self.last_applied = None
        s.pi_last_sent[self.axis] = None
        s.pi_applying = True
        try:
            for index, key in enumerate(("kp", "ki")):
                if old is None or old[index] != profile[key]:
                    s._write(_frame(mid, (0x0F, 0x10)[index], profile[key].to_bytes(2, "big")))
            s.pi_last_sent[self.axis] = [profile["kp"], profile["ki"]]
            self.last_applied = list(s.pi_last_sent[self.axis])
            s.pi_status = "自动整定临时 PI 已发送，等待响应测试"
        finally:
            s.pi_applying = False

    def _offset(self, degrees):
        s = self.service
        q = self.origin[self.axis]+degrees
        mid = s.c["motor_ids"][self.axis]
        packet = set_single_turn_position(mid, q/s.c["motor_signs"][self.axis])
        units = int.from_bytes(packet[3:5], "big")
        if units != s.position_units[self.axis]:
            s._write(packet)  # Existing reference, lead-distance and limit gate.
            s.position_units[self.axis] = units
            s.last_position_send = time.monotonic()
        target = list(s.sent_target)
        target[self.axis] = round(signed_mechanical(units/10.)*s.c["motor_signs"][self.axis], 7)
        self.target = target[self.axis]
        s.sent_target = s.manual_target = s.stream_target = tuple(target)
        s.stream_velocity = (0., 0.)

    def advance(self, now):
        if not self.active:
            return
        if now <= self.session.last_stamp:
            return
        s = self.service
        valid = (s.armed and s.arm_requested and not s.fault and s.reference_valid
                 and now-s.feedback_time <= s.c["reference_max_age_s"] and not s.home_requested
                 and all(abs(a-b) <= 2.5 for a, b in zip(s.angles, self.origin)))
        speed = 0.
        if self.last_sample is not None and now > self.last_sample[0]:
            speed = (s.angles[self.axis]-self.last_sample[1])/(now-self.last_sample[0])
        self.last_sample = (now, s.angles[self.axis])
        self.session.observe(now, self.target-s.angles[self.axis], speed, valid)
        s.last_send = now
        if not self.active:
            if self.session.state == "FAILED":
                s.request_arm(False)
                raise RuntimeError(self.session.reason)
            if self.detached:
                s.startup_waiting_owner = True

    def cancel(self, now):
        self.session.cancel(now)

    def interrupted(self, reason):
        if self.active:
            self.session.fail(reason)

    def export(self):
        return dict(self.session.snapshot(), axis=self.axis, origin=self.origin,
                    motor_id=self.service.c["motor_ids"][self.axis] if self.axis is not None else None,
                    saved_path=self.saved_path,
                    parameters_basis="Known user baseline; writes have no documented ACK/readback",
                    motion="+1 deg, center, -1 deg, center")

    def save(self):
        if self.active or self.session.state != "COMPLETE":
            raise RuntimeError("Finish verification before saving the PI profile")
        path = "/root/gimbal_autotune/pi_profile_axis%d.json" % self.axis
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path+".tmp", "w", encoding="utf-8") as output:
            json.dump(self.export(), output, ensure_ascii=False, allow_nan=False)
        os.replace(path+".tmp", path)
        self.saved_path = path
        return path
