"""Single UART owner, joystick mailbox, verified home and cumulative reset."""

class UserDisarm(Exception):
    pass


class JoystickMotorService(MotorService):
    def __init__(self, c, transport_factory=None):
        super().__init__(c, transport_factory)
        self.joystick = (0.0, 0.0)
        self.home_requested = False
        self.abort_requested = False
        self.clear_allowed = False
        self.phase = "CONNECTING"
        self.zero_confirmed = bool(c["mechanical_zero_confirmed"])
        self.manual_target = None
        self.was_moving = False

    def input(self, x, y):
        if any(not math.isfinite(v) or abs(v) > 1 for v in (x, y)):
            raise ValueError("Joystick vector must be finite and bounded")
        with self.lock:
            self.joystick = (float(x), float(y))
            self.command_time = time.monotonic()

    def confirm_reference(self):
        with self.lock:
            if self.fault or not self.ready or self.armed or self.arm_requested or not self.reference_valid:
                raise RuntimeError("Confirm saved physical zero only while ready and disabled")
            self.zero_confirmed = True
        print("USER_CONFIRMED_SAVED_MECHANICAL_ZERO (no calibration command sent)")

    def request_arm(self, armed):
        if armed:
            with self.lock:
                if not self.zero_confirmed:
                    raise RuntimeError("Check saved physical zero, then tap ZERO OK")
                if self.home_requested or self.phase not in ("DISABLED", "HOME_DONE_DISABLED"):
                    raise RuntimeError("ARM only from disabled idle; wait for HOME to finish")
                if self.fault or not self.ready or not self.reference_valid:
                    raise RuntimeError(self.fault or "Valid dual-axis feedback required")
                self.abort_requested = False
                self.arm_requested = True
        else:
            with self.lock:
                self.abort_requested = True
                self.arm_requested = False
                self.home_requested = False
                self.joystick = (0.0, 0.0)

    def request_home(self):
        with self.lock:
            if self.fault or not self.armed or not self.arm_requested or not self.reference_valid or not self.zero_confirmed:
                raise RuntimeError("Confirm physical zero and ARM before HOME")
            if self.home_requested or self.phase not in ("ARMED", "MANUAL", "HOME_HOLD"):
                raise RuntimeError("Home already active or motor not ready")
            self.home_requested = True
            self.joystick = (0.0, 0.0)

    def snapshot(self):
        value = super().snapshot()
        with self.lock:
            value.update(phase=self.phase, zero_confirmed=self.zero_confirmed,
                         target=self.sent_target, home_requested=self.home_requested)
        return value

    def _write(self, data):
        time.sleep(0.002)
        if self.stop_event.is_set() and data[2] != FUNCTION_DISABLE:
            raise UserDisarm("Motor stop requested")
        if data[2] in (FUNCTION_ENABLE, FUNCTION_MULTI_TURN_POSITION, 0x09):
            self._heartbeat_check()
            if data[2] == 0x09 and (not self.clear_allowed or self.armed):
                raise RuntimeError("Cumulative reset outside verified disabled-home transaction")
        if data[2] in (FUNCTION_ENABLE, FUNCTION_MULTI_TURN_POSITION):
            self._require_reference(fresh=True)
            if not self.arm_requested:
                raise UserDisarm("No active ARM request")
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete motor UART write")

    def _heartbeat_check(self):
        if self.abort_requested or self.stop_event.is_set():
            raise UserDisarm("User requested DISARM")
        if time.monotonic() - self.command_time > self.c["watchdog_s"]:
            raise RuntimeError("Control heartbeat expired; verify state before rearming")

    def _disable_now(self):
        errors = self._disable_all()
        self.armed = self.arm_requested = False
        self.home_requested = False
        self.joystick = (0.0, 0.0)
        self.manual_target = self.sent_target = self.angles
        self.guard.reset()
        if errors:
            raise RuntimeError("Dual-axis disable request failed: " + "; ".join(errors))

    def _advance(self, goal, rate):
        self._heartbeat_check()
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_send))
        target = tuple(old + clip(new-old, -speed*dt, speed*dt)
                       for old, new, speed in zip(self.sent_target, goal, rate))
        for mid, q, sign, zero, limits in zip(self.c["motor_ids"], target, self.c["motor_signs"], self.reference_zero, self.c["limits"]):
            if not limits[0] <= q <= limits[1]:
                raise RuntimeError("Target outside mechanical limits")
            self._write(set_multi_turn_position(mid, zero + q / sign))
        self.sent_target, self.last_send = target, now
        self.guard.update(now, self.angles, target)

    def _at_home(self):
        tol = self.c["home_tolerance_deg"]
        return (all(abs(v) <= tol for v in self.angles)
                and all(v is not None and abs(v) <= tol for v in self.mechanical))

    def _release_hold(self):
        self.manual_target = self.angles

    def _stationary_home(self):
        """After disable, independently check physical home and zero speed 3 times."""
        readings = []
        for _ in range(3):
            self._heartbeat_check()
            self._measure()
            if not self._at_home():
                raise RuntimeError("Axis left physical home after disable; cumulative reset blocked")
            for mid in self.c["motor_ids"]:
                if self._feedback(mid, FEEDBACK_SPEED) != 0:
                    raise RuntimeError("Axis moving; cumulative reset blocked")
            readings.append(self.raw_feedback)
            if self.stop_event.wait(0.08):
                raise UserDisarm("Stopped")
        for axis in range(2):
            values = [sample[axis] for sample in readings]
            if max(values) - min(values) > self.c["reference_stationary_deg"]:
                raise RuntimeError("Axis not stationary; cumulative reset blocked")

    def _home_and_clear(self):
        started = time.monotonic()
        settled = None
        previous = None
        self.phase = "RETURNING"
        self.guard.reset()
        while True:
            self._heartbeat_check()
            self._measure()
            now = time.monotonic()
            if now - started > self.c["home_timeout_s"]:
                raise RuntimeError("Home timeout; cumulative reset NOT sent")
            self._advance((0.0, 0.0), self.c["home_rate"])
            stopped = self._at_home()
            if stopped:
                for mid in self.c["motor_ids"]:
                    stopped = (self._feedback(mid, FEEDBACK_SPEED) == 0) and stopped
            if previous is not None:
                stopped = stopped and all(abs(a-b) <= self.c["reference_stationary_deg"]
                                          for a, b in zip(previous, self.raw_feedback))
            previous = self.raw_feedback
            if stopped:
                self.phase = "SETTLING"
                if settled is None:
                    settled = now
                if now - settled >= self.c["home_settle_s"]:
                    break
            else:
                settled = None
                self.phase = "RETURNING"
            if self.stop_event.wait(0.02):
                raise UserDisarm("Stopped")
        self._disable_now()
        self.phase = "CHECK_DISABLED"
        self._stationary_home()
        self._heartbeat_check()
        self._require_reference(fresh=True)
        self.phase = "CLEARING"
        self.reference_valid = False  # No position or enable until both axes verified.
        self.clear_allowed = True
        try:
            for mid in self.c["motor_ids"]:
                self._heartbeat_check()
                if abs(signed_mechanical(self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE))) > self.c["home_tolerance_deg"]:
                    raise RuntimeError("Physical home lost before cumulative reset")
                if self._feedback(mid, FEEDBACK_SPEED) != 0:
                    raise RuntimeError("Motor moving before cumulative reset")
                self._write(_frame(mid, 0x09))
                if self.stop_event.wait(0.05):
                    raise UserDisarm("Stopped during reset")
                total = self._feedback(mid)
                mech = signed_mechanical(self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE))
                if abs(total) > self.c["clear_tolerance_deg"] or abs(mech) > self.c["home_tolerance_deg"]:
                    raise RuntimeError("Cumulative reset readback failed; restart required")
            self.phase = "VERIFYING"
            self._heartbeat_check()
            self._capture_reference()  # Sole intentional reset of continuity history.
            self._measure()
            self._heartbeat_check()
            if not self._at_home() or any(abs(v) > self.c["clear_tolerance_deg"] for v in self.raw_feedback):
                raise RuntimeError("Post-reset zero verification failed")
            self.sent_target = self.manual_target = self.angles
            self.phase = "HOME_DONE_DISABLED"
            print("HOME_DONE: physical home checked, cumulative zero read back; remains DISABLED")
        except Exception:
            self.reference_valid = False
            raise
        finally:
            self.clear_allowed = False

    def _recheck_requested(self):
        return False

    def _perform_recheck(self):
        raise RuntimeError("Recheck unsupported by this service")

    def _manual_phase(self, moving):
        return "MANUAL" if moving else "ARMED"

    def _control_loop(self):
        while not self.stop_event.wait(0.02):
            if self._recheck_requested():
                return
            try:
                self._measure()
                if self.arm_requested and not self.armed:
                    self.manual_target = self.sent_target = self._arm_current()
                    self.armed = True
                    self.was_moving = False
                    self.last_send = time.monotonic()
                    self.guard.reset()
                    self.phase = "ARMED"
                if self.armed and not self.arm_requested:
                    self._disable_now()
                    self.phase = "DISABLED"
                if not self.armed:
                    continue
                self._heartbeat_check()
                if self.home_requested:
                    self._home_and_clear()
                    continue
                now = time.monotonic()
                dt = min(0.1, max(0.0, now - self.last_send))
                with self.lock:
                    vector = self.joystick
                moving = any(abs(v) > 0 for v in vector)
                if not moving and self.was_moving:
                    self._release_hold()  # Release holds current measured pose.
                if moving:
                    self.manual_target = tuple(clip(clip(old+v*rate*dt, actual-self.c["manual_lead_deg"], actual+self.c["manual_lead_deg"]), *lim)
                                               for old, actual, v, rate, lim in zip(self.manual_target, self.angles, vector, self.c["max_rate"], self.c["limits"]))
                self.was_moving = moving
                self.phase = self._manual_phase(moving)
                self._advance(self.manual_target, self.c["max_rate"])
            except UserDisarm:
                if not self.reference_valid:
                    raise RuntimeError("Reset interrupted; zero invalid; restart required")
                self._disable_now()
                self.phase = "DISABLED"

    def _record_fault(self, exc):
        if not self.stop_event.is_set():
            self.phase = "FAULT"
            self._latch_fault(str(exc), reference_lost=not self.reference_valid)
            print("MOTOR_FAULT", str(exc))

    def _run(self):
        try:
            try:
                self.serial = self._open()
                self.sent_target = self.manual_target = self._initialize()
                self.ready = True
                self.phase = "DISABLED"
                self.last_send = self.command_time = time.monotonic()
            except Exception as exc:
                self._record_fault(exc)
            while not self.stop_event.is_set():
                try:
                    if self._recheck_requested():
                        self._perform_recheck()
                    if self.fault:
                        if self.stop_event.wait(0.1):
                            break
                        self._disable_all()
                        if self.serial is not None:
                            self._read_diagnostics()
                    else:
                        self._control_loop()
                except Exception as exc:
                    self._record_fault(exc)
        finally:
            self._disable_all()
            if self.serial is not None:
                try:
                    self.serial.close()
                except Exception as exc:
                    print("MOTOR_CLOSE_FAILED", str(exc))
            self.ready = self.armed = self.arm_requested = self.reference_valid = False
