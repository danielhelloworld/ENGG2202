"""Separate ARM/hold and HOME, with bounded recovery and motion diagnostics."""
from collections import deque


class FeedbackUnavailable(RuntimeError):
    pass


class MotionProtectionFault(RuntimeError):
    pass


class TrackingMotionGuard(MotionGuard):
    def __init__(self, c):
        super().__init__(c)
        self.last_event = None
        self.request_windows = [None, None]
        self.paused_at = None
        self.reverse_since = [None, None]
        self.warning = None

    def reset(self):
        super().reset()
        self.request_windows = [None, None]
        self.paused_at = None
        self.reverse_since = [None, None]
        self.warning = None

    def pause(self, now):
        if self.paused_at is None:
            self.paused_at = now

    def resume(self, now):
        if self.paused_at is not None:
            duration = max(0., now-self.paused_at)
            for windows in (self.windows, self.request_windows):
                for axis,window in enumerate(windows):
                    if window is not None:
                        windows[axis] = (window[0]+duration, *window[1:])
            self.reverse_since = [None if t is None else t+duration for t in self.reverse_since]
            self.paused_at = None

    def check_request(self, now, actual, commanded, requested, homing=False):
        for axis,(position,request,limits) in enumerate(zip(actual,requested,self.c["limits"])):
            at_goal = abs(position) <= self.c["home_tolerance_deg"] if homing else (
                (request < 0 and position <= limits[0]+self.c["home_tolerance_deg"]) or
                (request > 0 and position >= limits[1]-self.c["home_tolerance_deg"]))
            if abs(request) < 1. or at_goal:
                self.request_windows[axis] = None
                continue
            direction = 1 if request > 0 else -1
            window = self.request_windows[axis]
            if window is None or window[2] != direction or (position-window[1])*direction >= self.c["motion_progress_deg"]:
                self.request_windows[axis] = (now, position, direction)
            elif now-window[0] >= self.c["motion_grace_s"]+self.c["motion_window_s"]:
                reason = "Axis %d requested movement has no physical progress" % axis
                self.last_event = dict(reason=reason,time=now,actual=tuple(actual),commanded=commanded)
                raise RuntimeError("SUSPECTED_BLOCK: "+reason)

    def update(self, now, actual, commanded):
        self.warning = None
        try:
            for axis,(position,goal) in enumerate(zip(actual,commanded)):
                error = goal-position
                if abs(error) < self.c["motion_error_deg"]:
                    self.windows[axis] = self.reverse_since[axis] = None
                    continue
                direction = 1 if error > 0 else -1
                window = self.windows[axis]
                if window is None or window[2] != direction:
                    self.windows[axis] = (now,position,direction)
                    self.reverse_since[axis] = None
                    continue
                elapsed,progress = now-window[0],(position-window[1])*direction
                if elapsed < self.c["motion_grace_s"]:
                    continue
                if progress <= -self.c["motion_reverse_deg"]:
                    if self.reverse_since[axis] is None:
                        self.reverse_since[axis] = now
                    if now-self.reverse_since[axis] >= self.c["motion_reverse_persist_s"]:
                        raise RuntimeError("Axis %d moving opposite commanded direction" % axis)
                    self.warning = "Axis %d brief reverse movement; observing" % axis
                else:
                    self.reverse_since[axis] = None
                if elapsed >= self.c["motion_grace_s"]+self.c["motion_window_s"]:
                    if progress < self.c["motion_progress_deg"]:
                        raise RuntimeError("Axis %d motion stalled (position heuristic)" % axis)
                    self.windows[axis] = (now,position,direction)
        except RuntimeError as exc:
            self.last_event = dict(reason=str(exc), time=now,
                                   actual=tuple(actual), commanded=tuple(commanded))
            raise RuntimeError("SUSPECTED_BLOCK: %s; actual=%s target=%s; DISARM requested" % (
                str(exc), tuple(round(v, 2) for v in actual), tuple(round(v, 2) for v in commanded))) from exc


class VisualMotorService(MechanicalOnlyService):
    def __init__(self, c, transport_factory=None):
        super().__init__(c, transport_factory)
        self.guard = TrackingMotionGuard(c)
        self.arm_homing = False
        self.recovery_active = False
        self.recovery_origin = None
        self.startup_completed = False
        self.startup_initialized = self.startup_cancelled = False
        self.outside_hold_origin = None
        self.recovery_rate = self.c["startup_home_rate"]
        self.feedback_cache = [None, None]
        self.feedback_issues = [None, None]
        self.feedback_rejects = [0, 0]
        self.feedback_missing_since = [None, None]
        self.feedback_query_started = [None, None]
        self.last_feedback_fault = None
        self.strict_feedback = False
        self.response = ResponseTuner(c)
        self.manual_ramp = ManualRateRamp(c)
        self.position_intent = [None, None]
        self.programmed_rpm = [None, None]
        self.telemetry_lock = threading.Lock()
        self.telemetry_history = deque(maxlen=c["telemetry_samples"])
        self.last_telemetry = 0.
        self.response_context = "MANUAL"
        self.motion_anomalies = {}
        self.motion_warning = None
        self.motion_paused = False

    def set_response_context(self, context):
        if context not in ("MANUAL", "TRACK", "COAST", "LOST", "HOLD"):
            raise ValueError("Invalid response context")
        if context != self.response_context:
            self.manual_ramp.reset()
        self.response_context = context

    def _position_lead_deg(self, axis):
        if self.recovery_active:
            return self.c["home_position_lead_deg"][axis]
        if self.outside_hold_origin is not None:
            return self.c["manual_lead_deg"]
        if self.response_context == "MANUAL" and self.joystick[axis]:
            return self.c["manual_position_lead_deg"][axis]
        if self.response_context == "TRACK":
            return self.c["tracking_position_lead_deg"][axis]
        return self.c["manual_lead_deg"]

    def _effective_accel(self):
        ceiling = (self.c["home_accel_deg_s2"] if self.recovery_active else
                   self.c["command_accel_deg_s2"] if self.response_context == "MANUAL" else
                   self.c["tracking_accel_deg_s2"])
        return tuple(a*min(1.,tuned/base) for a,tuned,base in
                     zip(ceiling,self.response.accel,self.c["command_accel_deg_s2"]))

    def _driver_rates(self):
        if self.recovery_active:
            return self.recovery_rate
        if self.outside_hold_origin is not None:
            return tuple(min(v,cap) for v,cap in zip(self.c["manual_rate"],(48.,36.)))
        if self.response_context == "MANUAL":
            return self.c["manual_rate"]
        if self.response_context == "COAST":
            return self.c["prediction_coast_rate"]
        return self.c["tracking_rate"]

    def _sync_driver_speed(self):
        for axis,(mid,rate) in enumerate(zip(self.c["motor_ids"],self._driver_rates())):
            rpm = min(self.c["motor_rpm"],max(1,int(math.ceil(rate/6.))))
            if self.programmed_rpm[axis] != rpm:
                self._write(set_speed(mid,rpm))
                self.programmed_rpm[axis] = rpm

    def _response_snapshot(self):
        result = self.response.snapshot()
        result["software_accel_deg_s2"] = self._effective_accel()
        return result

    def request_arm(self, armed):
        if not armed and not self.ready and self.phase == "CONNECTING":
            self.startup_cancelled = True
        super().request_arm(armed)

    def _feedback_once(self, motor_id, kind):
        for _ in range(8):
            if not self._read():
                break
        else:
            raise RuntimeError("Motor UART input never drained")
        self.parser = FeedbackParser()
        if kind == FEEDBACK_MECHANICAL_ANGLE:
            self.feedback_query_started[self.c["motor_ids"].index(motor_id)] = time.monotonic()
        self._write(request_feedback(motor_id, kind))
        deadline = time.monotonic()+self.c["feedback_timeout"]
        received = bytearray()
        while not self.stop_event.is_set():
            # Read queued bytes BEFORE checking the deadline. NPU/Python scheduling
            # may delay this thread while the correct UART reply is already ready.
            data = self._read()
            received.extend(data or b"")
            for mid,response_kind,raw in self.parser.feed(bytes(data or b"")):
                if mid == motor_id and response_kind == kind:
                    return raw if kind == FEEDBACK_SPEED else raw/10.
            if time.monotonic() >= deadline:
                break
            time.sleep(.001)
        raise FeedbackUnavailable("No valid feedback from motor %d type=%d RX=%s" % (motor_id,kind,bytes(received[-64:]).hex()))

    def _require_reference(self, fresh=False):
        if self.fault or not self.reference_valid:
            self._lose_reference("mechanical reference unavailable; RECHECK while disabled")
        if fresh and time.monotonic()-self.feedback_time >= self.c["reference_max_age_s"]:
            try:
                while True:
                    self._heartbeat_check()
                    self._read_measurement()  # Data age alone is not a failed query.
                    if not any(self.feedback_issues):
                        break  # A motion write must not proceed with a cached reply.
            except UserDisarm:
                raise
            except MotionProtectionFault as exc:
                self._latch_fault(str(exc))
                raise
            except Exception as exc:
                self._lose_reference(str(exc))
        for axis,since in enumerate(self.feedback_missing_since):
            if since is not None and time.monotonic()-since >= self.c["feedback_loss_grace_s"]:
                self._lose_reference("ID=%d continuous mechanical query failures for >=%.0fms" % (
                    self.c["motor_ids"][axis], self.c["feedback_loss_grace_s"]*1000))

    def _feedback(self, motor_id, kind=FEEDBACK_TOTAL_ANGLE):
        try:
            return super()._feedback(motor_id, kind)
        except FeedbackUnavailable:
            if kind == FEEDBACK_SPEED and self.armed:
                return math.nan  # Unknown speed must postpone HOME settling, never mean zero.
            raise

    def _unavailable_angle(self, axis, reason):
        self.feedback_issues[axis] = reason
        self.feedback_rejects[axis] += 1
        if self.feedback_missing_since[axis] is None:
            self.feedback_missing_since[axis] = self.feedback_query_started[axis] or time.monotonic()
        cached = self.feedback_cache[axis]
        if not self.strict_feedback and cached is not None and time.monotonic()-self.feedback_missing_since[axis] < self.c["feedback_loss_grace_s"]:
            return cached[0]  # Its ORIGINAL timestamp is retained; this is not a fresh sample.
        raise FeedbackUnavailable("ID=%d continuous mechanical query failures for %.0fms: %s" % (
            self.c["motor_ids"][axis], self.c["feedback_loss_grace_s"]*1000, reason))

    def _mechanical_feedback(self, mid):
        axis = self.c["motor_ids"].index(mid)
        try:
            raw = self._feedback(mid, FEEDBACK_MECHANICAL_ANGLE)
        except FeedbackUnavailable as exc:
            return self._unavailable_angle(axis, str(exc))
        if not math.isfinite(raw) or not 0 <= raw <= 360:
            return self._unavailable_angle(axis, "Invalid mechanical feedback: %r" % raw)
        value, now = round(signed_mechanical(raw), 7), time.monotonic()
        previous = self.mechanical_samples[axis]
        pending_home = self.arm_requested and not self.armed and self.home_requested
        if previous is not None and (self.recovery_active or pending_home or self.outside_hold_origin is not None):
            value = previous[1]+signed_mechanical(value-previous[1])
        if previous is not None and (self.armed or self.arm_requested):
            dt = max(0., now-previous[0])
            if self.feedback_missing_since[axis] is not None:
                # A rejected offset must not become believable just by waiting.
                dt = min(dt,max(0.,self.feedback_missing_since[axis]-previous[0]))
            # Position packets and feedback are sampled at different instants.
            # Credit at most one position-packet period for this phase offset,
            # only when a successfully written target provides an independent
            # bounded movement corridor. Never refresh a rejected sample's time.
            phase_slack = self.c["position_period_s"] if self.position_intent[axis] is not None else 0.
            bound = max(self.c["motor_rpm"]*6*(dt+phase_slack)+self.c["reference_jump_slack_deg"],
                        self.c["manual_lead_deg"]+self.c["recovery_margin_deg"])
            intent = self.position_intent[axis]
            if intent is not None:
                # A faster driver ceiling must not make an idle 20-degree
                # reference jump plausible. Trust only the last written target
                # corridor, plus the existing noise/recovery tolerances.
                travel = abs(signed_mechanical(intent/self.c["motor_signs"][axis]-previous[1]))
                allowance = self.c["reference_jump_slack_deg"]+(self.c["recovery_hard_margin_deg"] if self.recovery_active else 0.)
                bound = min(bound,max(self.c["manual_lead_deg"]+self.c["recovery_margin_deg"],travel+allowance))
            if abs(value-previous[1]) > bound:
                self.last_angle_jump = dict(axis=axis, motor_id=mid, previous_deg=previous[1],
                                           current_deg=value, elapsed_s=dt, allowed_deg=bound)
                return self._unavailable_angle(axis, "Unexpected mechanical angle jump: %.1f->%.1f" % (previous[1],value))
        self.feedback_cache[axis] = (value, now)
        self.feedback_issues[axis] = None
        self.feedback_missing_since[axis] = None
        with self.lock:
            values = list(self.mechanical)
            values[axis] = value
            self.mechanical = tuple(values)
            self.mechanical_feedback_times[axis] = now
        return value

    def _measure(self):
        was_paused = self.motion_paused
        try:
            # Do not apply a 100ms age gate before giving a delayed UART thread
            # a chance to obtain the next real reply.
            if self.fault or not self.reference_valid:
                self._lose_reference("mechanical reference unavailable; RECHECK while disabled")
            result = self._read_measurement()
            self._require_reference(fresh=False)
        except MotionProtectionFault as exc:
            self._latch_fault(str(exc))
            raise
        except Exception as exc:
            if self.stop_event.is_set() or self.reference_fault:
                raise
            self._lose_reference(str(exc))
        degraded = any(self.feedback_issues)
        now = time.monotonic()
        self.motion_paused = bool(degraded or self.motion_warning or now-self.command_time > self.c["control_pause_age_s"])
        if self.motion_paused:
            self.manual_ramp.reset()
            self.guard.pause(now)
        else:
            self.guard.resume(now)
        if was_paused and not self.motion_paused and self.armed:
            # Resume from the measured pose, without a queued movement or old ramp.
            self.stream_target = self.manual_target = self.angles
            self.stream_velocity = (0., 0.)
            self.last_send = time.monotonic()
        requested = (tuple(-math.copysign(rate,q) if abs(q) > self.c["home_tolerance_deg"] else 0.
                           for q,rate in zip(self.angles,self.recovery_rate)) if self.recovery_active else
                     tuple(v*r for v,r in zip(self.joystick,self.c["max_rate"])))
        self.response.update(time.monotonic(), self.angles, self.sent_target, requested,
                             self.armed, healthy=not self.motion_paused)
        if self.armed and not self.motion_paused:
            self.guard.check_request(now, self.angles, self.sent_target, requested, self.recovery_active)
        self._record_telemetry(requested)
        return result

    def _record_telemetry(self, requested):
        now = time.monotonic()
        if now-self.last_telemetry < self.c["telemetry_period_s"]:
            return
        self.last_telemetry = now
        row = dict(t=now, phase=self.phase, control_context=self.response_context, armed=self.armed, actual_deg=self.angles,
                   commanded_deg=self.sent_target if self.armed else None, requested_rate_deg_s=requested,
                   planned_velocity_deg_s=self.stream_velocity,
                   manual_acceleration_deg_s2=tuple(self.manual_ramp.acceleration),
                   feedback_age_s=now-self.feedback_time, feedback_degraded=bool(any(self.feedback_issues)),
                   motion_warning=self.motion_warning, motion_paused=self.motion_paused,
                   response=self._response_snapshot())
        with self.telemetry_lock:
            self.telemetry_history.append(row)

    def telemetry(self):
        with self.telemetry_lock:
            rows = list(self.telemetry_history)
        return dict(schema=1, units="degrees, seconds", samples=rows,
                    notes="Only armed, fresh-feedback samples are used for software tuning. No torque/current feedback; driver PID unchanged.")

    def _initialize(self):
        angles = super()._initialize()  # Disabled reads accept positions outside work limits.
        self.startup_initialized = True
        if self.c["startup_auto_home"] and not self.startup_cancelled and not self.abort_requested:
            # User-authorized automatic startup uses the existing saved zero.
            # No zero-setting, parameter save or cumulative reset is performed.
            self.zero_confirmed = True
            self.arm_requested = True
            self.home_requested = True
            print("STARTUP_AUTO_HOME: automatic enable and bounded home from current pose")
        return angles

    def _capture_reference(self):
        self.reference_valid = False
        self.feedback_cache = [None, None]
        self.feedback_missing_since = [None, None]
        self.strict_feedback = True
        try:
            for mid in self.c["motor_ids"]:
                first = self._mechanical_feedback(mid)
                last = self._mechanical_feedback(mid)
                if abs(signed_mechanical(last-first)) > self.c["reference_stationary_deg"]:
                    raise RuntimeError("Axis moved during mechanical initialization")
        finally:
            self.strict_feedback = False
        self.mechanical_samples = [None, None]
        self.reference_valid = True

    def _read_measurement(self):
        mechanical, stamps = [], []
        for axis, mid in enumerate(self.c["motor_ids"]):
            value = self._mechanical_feedback(mid)
            mechanical.append(value)
            stamps.append(self.feedback_cache[axis][1])
        angles = tuple(round(v*s, 7) for v,s in zip(mechanical, self.c["motor_signs"]))
        current_anomalies = {}
        def check_corridor(axis,q,lo,hi,soft,hard,label):
            deviation = max(lo-q,q-hi,0.)
            if deviation <= soft+1e-7:
                return
            reason = "%s axis %d outside %.1f..%.1f by %.2f deg" % (label,axis,lo,hi,deviation)
            if deviation > hard+1e-7:
                raise MotionProtectionFault("MOTION_HARD_LIMIT: "+reason)
            key = (label,axis)
            since = self.motion_anomalies.get(key,time.monotonic())
            current_anomalies[key] = since
            if time.monotonic()-since >= self.c["motion_violation_grace_s"]:
                raise MotionProtectionFault("MOTION_PATH_PERSISTENT: "+reason)
        if self.recovery_active and self.recovery_origin is not None:
            margin = self.c["recovery_margin_deg"]
            for axis, (q, start) in enumerate(zip(angles, self.recovery_origin)):
                check_corridor(axis,q,min(start,0.),max(start,0.),margin,self.c["recovery_hard_margin_deg"],"Recovery")
        elif self.armed and self.outside_hold_origin is not None:
            for axis,(q,start,(lo,hi)) in enumerate(zip(angles,self.outside_hold_origin,self.c["limits"])):
                margin = self.c["limit_feedback_margin_deg"]+(self.c["manual_lead_deg"] if not lo <= start <= hi else 0.)
                hard = max(margin+.5,self.c["recovery_hard_margin_deg"] if not lo <= start <= hi else self.c["limit_hard_margin_deg"])
                check_corridor(axis,q,min(start,lo),max(start,hi),margin,hard,"Outside-range recovery")
            tol = self.c["limit_feedback_margin_deg"]
            if all(lo-tol <= q <= hi+tol for q,(lo,hi) in zip(angles,self.c["limits"])):
                self.outside_hold_origin = None
                self.manual_target = angles
        elif self.armed and not self.recovery_active:
            margin = self.c["limit_feedback_margin_deg"]
            for axis,(q,(lo,hi)) in enumerate(zip(angles,self.c["limits"])):
                check_corridor(axis,q,lo,hi,margin,self.c["limit_hard_margin_deg"],"Mechanical position outside normal work limits")
        self.motion_anomalies = current_anomalies
        self.motion_warning = "; ".join("%s axis %d: pausing to recheck" % key for key in current_anomalies) or None
        with self.lock:
            self.mechanical, self.angles = tuple(mechanical), angles
            self.raw_feedback = (None, None)
            self.feedback_time = min(stamps)
            self.mechanical_samples = list(zip(stamps, mechanical))
        return tuple(mechanical), angles

    def _write(self, data):
        if data[2] in (3, FUNCTION_ENABLE) and any(self.feedback_issues):
            raise FeedbackUnavailable("Motion paused until fresh mechanical feedback returns")
        if data[2] == 3 and not self.recovery_active and self.outside_hold_origin is not None:
            origin = self.outside_hold_origin
            time.sleep(.002)
            self._heartbeat_check()
            self._require_reference(fresh=True)
            if not self.arm_requested:
                raise UserDisarm("Outside-range motion cancelled")
            if (len(data) != 7 or data[1] not in self.c["motor_ids"]
                    or data[0] != 0x7A or data[-1] != 0x7B or bcc_xor(data[:-2]) != data[-2]):
                raise RuntimeError("Invalid outside-range position transaction")
            axis = self.c["motor_ids"].index(data[1])
            units = int.from_bytes(data[3:5], "big")
            canonical = signed_mechanical(units/10.)*self.c["motor_signs"][axis]
            q = self.angles[axis]+signed_mechanical(canonical-self.angles[axis])
            actual = self.angles[axis]
            lo,hi = self.c["limits"][axis]
            local = abs(q-actual) <= self._position_lead_deg(axis)+.051
            inward = (lo <= q <= hi if lo <= actual <= hi else
                      hi-.051 <= q <= actual+.051 if actual > hi else
                      actual-.051 <= q <= lo+.051)
            margin = self.c["limit_feedback_margin_deg"]+(self.c["manual_lead_deg"] if not lo <= origin[axis] <= hi else 0.)
            if (units >= 3600 or not local or not inward or
                    not min(origin[axis],lo)-margin <= q <= max(origin[axis],hi)+margin):
                raise RuntimeError("Outside-range target must hold or make a local inward step")
            self._send_position(data,axis,q)
            return
        if data[2] != 3:
            return super()._write(data)
        time.sleep(.002)
        self._heartbeat_check()
        self._require_reference(fresh=True)
        self._heartbeat_check()  # A refresh may have overlapped a user DISARM.
        if not self.arm_requested:
            raise UserDisarm("No active ARM request")
        if len(data) != 7 or data[1] not in self.c["motor_ids"] or data[0] != 0x7A or data[-1] != 0x7B or bcc_xor(data[:-2]) != data[-2]:
            raise RuntimeError("Invalid recovery target frame")
        axis = self.c["motor_ids"].index(data[1])
        units = int.from_bytes(data[3:5], "big")
        actual = self.angles[axis]
        canonical = signed_mechanical(units/10.)*self.c["motor_signs"][axis]
        target = actual+signed_mechanical(canonical-actual)
        if not self.recovery_active:
            lo,hi = self.c["limits"][axis]
            if units >= 3600 or not lo <= canonical <= hi or abs(canonical-actual) > self._position_lead_deg(axis)+.051:
                raise RuntimeError("Unsafe single-turn step; no target sent")
            self._send_position(data,axis,canonical)
            return
        if not self.arm_homing or not self.home_requested or self.recovery_origin is None:
            raise RuntimeError("Recovery target outside active homing transaction")
        start = self.recovery_origin[axis]
        margin = self.c["recovery_margin_deg"]
        if (units >= 3600 or not min(start, 0)-margin <= target <= max(start, 0)+margin
                or abs(target-actual) > self._position_lead_deg(axis)+.051
                or abs(target) > abs(actual)+.051 or target*actual < -.001):
            raise RuntimeError("Unsafe recovery step: only local motion toward mechanical zero allowed")
        self._send_position(data,axis,target)

    def _send_position(self, data, axis, target):
        if self.serial.write(data) != len(data):
            raise RuntimeError("Incomplete motor UART write")
        # Publish per-axis intent immediately: a refresh between the two axis
        # writes must already recognize motion toward the first sent target.
        self.position_intent[axis] = target

    def _arm_current(self):
        if not self.arm_requested or self.stop_event.is_set():
            raise UserDisarm("ARM homing cancelled")
        if self.home_requested:
            self._begin_home()
        while True:
            self._heartbeat_check()
            mechanical, angles = self._measure()
            if not any(self.feedback_issues):
                break  # Bounded by the per-axis loss deadline; no cached-pose enable.
        if not self.home_requested:
            self.outside_hold_origin = angles if any(not lo <= q <= hi for q,(lo,hi) in zip(angles,self.c["limits"])) else None
        for mid in self.c["motor_ids"]:
            self._write(select_mode(mid, self.c["position_mode"]))
        self._sync_driver_speed()
        for mid,angle in zip(self.c["motor_ids"], mechanical):
            self._write(set_acceleration(mid, self.c["motor_acceleration"]))
            self._write(set_single_turn_position(mid, angle))
        for mid in self.c["motor_ids"]:
            self._write(enable(mid))
        self.stream_target, self.stream_velocity = angles, (0., 0.)
        self.manual_ramp.reset()
        self.position_units = [int.from_bytes(set_single_turn_position(mid, angle)[3:5], "big")
                               for mid,angle in zip(self.c["motor_ids"], mechanical)]
        self.last_position_send = time.monotonic()
        self.hold_pending = False
        print("ARM: current pose preloaded; HOME separately requested=%s" % self.home_requested)
        return angles

    def request_home(self):
        if not self.armed:
            raise RuntimeError("先 ARM 在当前位置使能，再点 HOME；HOME 不会自动使能")
        super().request_home()

    def _begin_home(self):
        self.manual_ramp.reset()
        self.arm_homing = self.home_requested = self.recovery_active = True
        self.recovery_origin = self._measure()[1]
        self.outside_hold_origin = None
        outside = any(not lo <= q <= hi for q,(lo,hi) in zip(self.recovery_origin, self.c["limits"]))
        self.recovery_rate = self.c["startup_home_rate"] if outside else self.c["home_rate"]

    def _advance(self, goal, rate):
        pause = bool(any(self.feedback_issues) or self.motion_warning or time.monotonic()-self.command_time > self.c["control_pause_age_s"])
        if pause:
            self._heartbeat_check()
            self._require_reference(fresh=False)  # No motion write: retry on the next normal read cycle.
            self.last_send = time.monotonic()
            self.stream_velocity = (0., 0.)
            self.manual_ramp.reset()
            self.motion_paused = True
            self.guard.pause(self.last_send)
            return  # Retain enable, but do not send a new position during the short grace.
        if self.motion_paused:
            self.motion_paused = False
            self.stream_target = self.manual_target = self.angles
            self.stream_velocity = (0.,0.)
            self.guard.resume(time.monotonic())
        if not self.recovery_active:
            if self.outside_hold_origin is not None:
                if any(self.joystick) or self.hold_pending:
                    return self._advance_velocity_stream()
                self._heartbeat_check()
                self._require_reference(fresh=True)
                self.last_send = time.monotonic()
                self.guard.update(self.last_send, self.angles, self.sent_target)
                return
            if self.phase == "MANUAL" and not self.hold_pending:
                return self._advance_velocity_stream()
            return super()._advance(goal, rate)
        self._heartbeat_check()
        self._require_reference(fresh=True)
        self._sync_driver_speed()
        if tuple(goal) != (0., 0.):
            raise RuntimeError("Only zero is allowed as a recovery goal")
        now = time.monotonic()
        dt = min(.1, max(0., now-self.last_send))
        target, velocity = [], []
        for axis,(old, old_v, actual, start, speed, accel) in enumerate(zip(self.stream_target, self.stream_velocity,
                self.angles, self.recovery_origin, self.recovery_rate, self._effective_accel())):
            desired = -math.copysign(min(speed, math.sqrt(2*accel*abs(old))), old) if old else 0.
            v = clip(desired, old_v-accel*dt, old_v+accel*dt)
            q = clip(old+v*dt, min(old, 0.), max(old, 0.))
            q = clip(q, actual-self._position_lead_deg(axis), actual+self._position_lead_deg(axis))
            q = clip(q, min(start, 0.), max(start, 0.))
            q = clip(q, min(actual, 0.), max(actual, 0.))
            if q == 0:
                v = 0.
            target.append(q)
            velocity.append(v)
        self.stream_target, self.stream_velocity = tuple(target), tuple(velocity)
        self.last_send = now
        if now-self.last_position_send >= self.c["position_period_s"]:
            sent = list(self.sent_target)
            for axis, (mid, q, sign) in enumerate(zip(self.c["motor_ids"], target, self.c["motor_signs"])):
                packet = set_single_turn_position(mid, q/sign)
                units = int.from_bytes(packet[3:5], "big")
                if units != self.position_units[axis]:
                    self._write(packet)
                    self.position_units[axis] = units
                    canonical = signed_mechanical(units/10.)*sign
                    sent[axis] = self.angles[axis]+signed_mechanical(canonical-self.angles[axis])
            self.sent_target = tuple(sent)
            self.last_position_send = now
        self.guard.update(now, self.angles, self.sent_target)

    def _advance_velocity_stream(self):
        """Continuous manual/tracking rate: do not brake at each short waypoint."""
        self._heartbeat_check()
        self._require_reference(fresh=True)
        self._sync_driver_speed()
        now = time.monotonic()
        dt = min(.1, max(0., now-self.last_send))
        with self.lock:
            vector = self.joystick
        force_hold = self.hold_pending
        target, velocity = [], []
        for axis,(old, old_v, actual, v, speed, accel, limits) in enumerate(zip(self.stream_target,
                self.stream_velocity, self.angles, vector, self.c["max_rate"],
                self._effective_accel(), self.c["limits"])):
            manual = self.response_context == "MANUAL"
            cap = (self.c["manual_rate"][axis] if manual else
                   self.c["prediction_coast_rate"][axis] if self.response_context == "COAST" else
                   self.c["tracking_rate"][axis])
            desired = (clip(v*speed, -cap, cap) if self.response_context == "COAST" else
                       clip(v*speed*self.response.scale[axis]+self.response.correction[axis], -cap, cap))
            if manual:
                # Position-error damping may slow a stick command to zero, but
                # it must not reverse that command or move an idle axis.
                direction = 1. if v > 0 else -1.
                desired = direction*max(0.,desired*direction) if v else 0.
            lo,hi = limits
            if self.response_context in ("MANUAL","TRACK") and lo <= actual <= hi:
                # Brake before a working limit; retain the strict final target gate.
                distance = max(0., (hi-old) if desired > 0 else (old-lo))
                brake_accel = accel*self.c["adaptive_accel_min"][axis]/self.response.accel[axis]
                # Include time to reverse acceleration through the jerk ramp.
                ramp_s = (2*accel/self.c["manual_jerk_deg_s3"][axis]+self.c["manual_response_s"] if manual else
                          self.c["position_period_s"])
                brake_rate = math.sqrt((brake_accel*ramp_s)**2+2*brake_accel*distance)-brake_accel*ramp_s
                desired = clip(desired, -brake_rate, brake_rate)
            new_v = (self.manual_ramp.advance(axis, old_v, desired, accel, dt, cap) if manual else
                     clip(desired, old_v-accel*dt, old_v+accel*dt))
            if not manual:
                new_v = clip(new_v,-cap,cap)  # COAST/selection never inherits a faster manual rate.
            candidate = old+(old_v+new_v)*.5*dt if manual else old+new_v*dt
            lo,hi = limits
            bounds = limits
            if self.outside_hold_origin is not None and actual > hi:
                bounds = (hi,max(actual,self.outside_hold_origin[axis]))
                new_v = min(0.,new_v)
                if desired >= 0:
                    candidate,new_v = actual,0.
                else:
                    candidate = old+new_v*dt
            elif self.outside_hold_origin is not None and actual < lo:
                bounds = (min(actual,self.outside_hold_origin[axis]),lo)
                new_v = max(0.,new_v)
                if desired <= 0:
                    candidate,new_v = actual,0.
                else:
                    candidate = old+new_v*dt
            if force_hold:
                candidate,new_v = actual,0.
            q = clip(clip(candidate, actual-self._position_lead_deg(axis), actual+self._position_lead_deg(axis)), *bounds)
            if (manual and v and not force_hold and self.outside_hold_origin is None
                    and old_v*v >= 0 and new_v*v >= 0):
                # Small feedback retreats must not retract an already-written
                # forward target. If its lead is temporarily too large, retain
                # it without resending until real feedback catches up. Likewise
                # do not command behind an overshooting motor while the stick
                # is still asking it to move forward. Real reversals are allowed
                # after the rate ramp has decelerated through zero.
                anchor = max(self.sent_target[axis],actual) if v > 0 else min(self.sent_target[axis],actual)
                if (q-anchor)*v < 0:
                    q = clip(anchor,*bounds)
            if q != candidate and (not manual or candidate != clip(candidate, *bounds)):
                # Lead limiting is anti-windup, not a reason to restart the ramp
                # every cycle while the real motor is still catching up.
                new_v = 0.
                self.manual_ramp.reset(axis)
            if force_hold or (self.outside_hold_origin is not None and new_v == 0.):
                self.manual_ramp.reset(axis)
            target.append(q)
            velocity.append(new_v)
        self.stream_target, self.stream_velocity = tuple(target), tuple(velocity)
        self.last_send = now
        if force_hold or now-self.last_position_send >= self.c["position_period_s"]:
            sent = list(self.sent_target)
            for axis,(mid,q,sign) in enumerate(zip(self.c["motor_ids"], target, self.c["motor_signs"])):
                packet = set_single_turn_position(mid, q/sign)
                units = int.from_bytes(packet[3:5], "big")
                if units != self.position_units[axis]:
                    self._write(packet)
                    self.position_units[axis] = units
                    sent[axis] = round(signed_mechanical(units/10.)*sign, 7)
            self.sent_target = tuple(sent)
            self.last_position_send = now
            self.hold_pending = False
        self.guard.update(now, self.angles, self.sent_target)

    def _home_and_clear(self):
        try:
            if not self.recovery_active:
                self._begin_home()
                self.stream_target = self.angles
                self.stream_velocity = (0., 0.)
            super()._home_and_clear()
            self.startup_completed = True
        finally:
            self.arm_homing = False
            self.recovery_active = False
            self.recovery_origin = None
            self.outside_hold_origin = None

    def _disable_now(self):
        super()._disable_now()
        self.response.reset()
        self.manual_ramp.reset()
        self.motion_paused = False
        self.position_intent = [None, None]
        self.programmed_rpm = [None, None]
        self.arm_homing = False
        self.recovery_active = False
        self.recovery_origin = None
        self.outside_hold_origin = None

    def _latch_fault(self, reason, reference_lost=False):
        if self.last_feedback_fault is None:
            now = time.monotonic()
            self.last_feedback_fault = dict(reason=reason,time=now,
                accepted_age_s=tuple(None if q is None else now-q[1] for q in self.feedback_cache),
                failed_query_duration_s=tuple(None if t is None else now-t for t in self.feedback_missing_since),
                issues=tuple(self.feedback_issues),phase=self.phase)
        self.response.reset()
        self.manual_ramp.reset()
        self.arm_homing = self.home_requested = self.home_holding = False
        self.recovery_active = False
        self.recovery_origin = None
        self.outside_hold_origin = None
        super()._latch_fault(reason, reference_lost)

    def _perform_recheck(self):
        self.feedback_cache = [None, None]
        self.feedback_missing_since = [None, None]
        self.strict_feedback = True
        try:
            super()._perform_recheck()
        finally:
            self.strict_feedback = False
        self.zero_confirmed = self.c["mechanical_zero_confirmed"]
        self.guard.last_event = None
        self.last_feedback_fault = None
        self.motion_anomalies = {}
        self.motion_warning = None

    def snapshot(self):
        result = super().snapshot()
        result["app_version"] = self.c["app_version"]
        result["protection"] = dict(home_feedback_margin_deg=self.c["recovery_margin_deg"],
            home_hard_margin_deg=self.c["recovery_hard_margin_deg"],
            motion_violation_grace_s=self.c["motion_violation_grace_s"],
            feedback_loss_grace_s=self.c["feedback_loss_grace_s"],
            stall_observation_s=self.c["motion_grace_s"]+self.c["motion_window_s"],
            control_pause_age_s=self.c["control_pause_age_s"],watchdog_s=self.c["watchdog_s"])
        result.update(arm_homing=self.arm_homing,
                      auto_home_on_arm=self.c["auto_home_on_arm"],
                      tracking_rate=self.c["tracking_rate"],
                      suspicious_motion=self.guard.last_event)
        result.update(max_rate=self.c["manual_rate"], motor_rate_ceiling=self.c["max_rate"])
        result.update(manual_jerk_deg_s3=self.c["manual_jerk_deg_s3"],
                      manual_acceleration_deg_s2=tuple(self.manual_ramp.acceleration),
                      command_accel_ceiling_deg_s2=self.c["command_accel_deg_s2"], motor_rpm=self.c["motor_rpm"])
        result["applied_motor_rpm"] = tuple(self.programmed_rpm)
        now = time.monotonic()
        result.update(feedback_degraded=bool(any(self.feedback_issues)),
                      feedback_loss_grace_s=self.c["feedback_loss_grace_s"],
                      feedback_axis_age_s=tuple(None if v is None else now-v[1] for v in self.feedback_cache),
                      feedback_issues=tuple(self.feedback_issues), feedback_rejected_samples=tuple(self.feedback_rejects),
                      feedback_failed_query_duration_s=tuple(None if t is None else now-t for t in self.feedback_missing_since),
                      last_feedback_fault=self.last_feedback_fault,
                      motion_warning=self.motion_warning or self.guard.warning, motion_paused=self.motion_paused,
                      response=self._response_snapshot(), default_saved_zero=self.c["mechanical_zero_confirmed"])
        result["response_context"] = self.response_context
        result.update(position_lead_deg=tuple(self._position_lead_deg(a) for a in (0,1)),
                      home_accel_deg_s2=self.c["home_accel_deg_s2"],
                      tracking_accel_deg_s2=self.c["tracking_accel_deg_s2"])
        result.update(recovery_active=self.recovery_active, startup_auto_home=self.c["startup_auto_home"],
                      outside_hold=self.outside_hold_origin is not None,
                      outside_manual_inward_only=self.outside_hold_origin is not None,
                      startup_completed=self.startup_completed,
                      startup_initialized=self.startup_initialized, startup_cancelled=self.startup_cancelled,
                      outside_work_limits=any(not lo-self.c["limit_feedback_margin_deg"] <= q <= hi+self.c["limit_feedback_margin_deg"]
                                              for q,(lo,hi) in zip(self.angles, self.c["limits"])))
        return result
