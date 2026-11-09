"""Separate ARM/hold and HOME, with bounded recovery and motion diagnostics."""
from collections import deque


class FeedbackUnavailable(RuntimeError):
    pass


class MotionProtectionFault(RuntimeError):
    pass


class TrackingReplan(RuntimeError):
    pass


class TrackingMotionGuard(MotionGuard):
    def __init__(self, c):
        super().__init__(c)
        self.last_event = None
        self.request_windows = [None, None]
        self.paused_at = None
        self.reverse_since = [None, None]
        self.warning = None
        self.tracking_samples = deque(maxlen=256)
        self.tracking_error_since = [None,None]
        self.tracking_reverse = [None,None]

    def reset(self):
        super().reset()
        self.request_windows = [None, None]
        self.paused_at = None
        self.reverse_since = [None, None]
        self.warning = None
        self.tracking_samples.clear()
        self.tracking_error_since = [None,None]
        self.tracking_reverse = [None,None]

    def update_tracking(self, now, actual, commanded, planned):
        """Moving targets need physical-motion evidence, not point-error direction."""
        self.warning = None
        self.tracking_samples.append((now,tuple(actual)))
        duration = self.c["motion_grace_s"]+self.c["motion_window_s"]
        while len(self.tracking_samples)>1 and now-self.tracking_samples[1][0] >= duration:
            self.tracking_samples.popleft()
        for axis,(position,goal,rate) in enumerate(zip(actual,commanded,planned)):
            error = goal-position
            if abs(error) < self.c["tracking_error_deg"]:
                self.tracking_error_since[axis] = None
            elif self.tracking_error_since[axis] is None:
                self.tracking_error_since[axis] = now
            positions = [row[1][axis] for row in self.tracking_samples]
            since = self.tracking_error_since[axis]
            if (since is not None and now-since >= duration and now-self.tracking_samples[0][0] >= duration
                    and max(positions)-min(positions) < self.c["motion_progress_deg"]):
                reason = "SUSPECTED_BLOCK: tracking axis %d has no physical movement for %.1fs" % (axis,duration)
                self.last_event = dict(reason=reason,time=now,actual=tuple(actual),commanded=tuple(commanded))
                raise RuntimeError(reason)
            # Overshoot and planned braking are not evidence of wrong wiring.
            direction = 1 if rate > 0 else -1
            if abs(rate) < 2 or error*direction < self.c["tracking_error_deg"]:
                self.tracking_reverse[axis] = None
                continue
            window = self.tracking_reverse[axis]
            if window is None or window[2] != direction or (position-window[1])*direction >= self.c["motion_progress_deg"]:
                self.tracking_reverse[axis] = (now,position,direction,None)
                continue
            progress = (position-window[1])*direction
            if now-window[0] >= self.c["motion_grace_s"] and progress <= -self.c["tracking_reverse_deg"]:
                first = now if window[3] is None else window[3]
                self.tracking_reverse[axis] = (*window[:3],first)
                self.warning = "Tracking axis %d reverse transient; observing" % axis
                if now-first >= self.c["tracking_reverse_persist_s"]:
                    reason = "SUSPECTED_BLOCK: tracking axis %d persistently moving opposite planned motion" % axis
                    self.last_event = dict(reason=reason,time=now,actual=tuple(actual),commanded=tuple(commanded))
                    raise RuntimeError(reason)
            elif window[3] is not None:
                self.tracking_reverse[axis] = (*window[:3],None)

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
            self.tracking_samples = deque(((t+duration,p) for t,p in self.tracking_samples),maxlen=256)
            self.tracking_error_since = [None if t is None else t+duration for t in self.tracking_error_since]
            self.tracking_reverse = [None if w is None else (w[0]+duration,w[1],w[2],None if w[3] is None else w[3]+duration) for w in self.tracking_reverse]
            self.paused_at = None

    def check_request(self, now, actual, commanded, requested, homing=False):
        for axis,(position,request,limits) in enumerate(zip(actual,requested,self.c["limits"])):
            # A residual below the position-error guard is not evidence of a
            # blockage, even while the home planner still requests motion.
            at_goal = abs(position) <= max(self.c["home_tolerance_deg"], self.c["motion_error_deg"]) if homing else (
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
        c=dict(c)
        self.configured_limits=c['limits']
        if c.get('continuous_yaw',False):
            c['limits']=((float('-inf'),float('inf')),tuple(c['limits'][1]))
        super().__init__(c, transport_factory)
        self.timings = StageTimings()
        self.last_measure_started = None
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
        self.recheck_previous_fault = None
        self.recheck_started = self.recheck_finished = None
        self.strict_feedback = False
        self.response = ResponseTuner(c)
        self.manual_ramp = ManualRateRamp(c)
        self.home_ramp = ManualRateRamp(dict(c,manual_jerk_deg_s3=c['home_jerk_deg_s3'],
                                            manual_response_s=c['home_response_s']))
        self.tracking_ramp = ManualRateRamp(dict(c,manual_jerk_deg_s3=c["tracking_jerk_deg_s3"],
                                                manual_response_s=c["tracking_response_s"]))
        self.brake_ramp = ManualRateRamp(dict(c,manual_jerk_deg_s3=c["brake_jerk_deg_s3"],
                                             manual_response_s=c["brake_response_s"]))
        self.braking = False
        self.soft_hold = False
        self.brake_started = None
        self.release_source = "MANUAL"
        self.angle_move = AngleMove()
        self.angle_pending = None
        self.angle_cancel = False
        self.inertia_test_active = False
        self.native_pitch_requested = self.native_pitch_engaged = False
        self.native_pitch_sent = self.native_restore_pending = False
        self.native_pitch_accel = 1
        self.native_tracker = (SpeedTrackingStream if c['tracking_motor_control']=='speed' else NativeTrackingStream)(self,c)
        self.hold_target = None
        self.hold_error_since = [None,None]
        self.hold_error_best = [None,None]
        self.hold_jump_candidate = [None,None]
        self.position_intent = [None, None]
        self.programmed_rpm = [None, None]
        self.position_write_time = [None, None]
        self.position_write_interval = [None, None]
        self.position_write_step = [None, None]
        self.position_write_count = [0, 0]
        self.telemetry_lock = threading.Lock()
        self.telemetry_history = deque(maxlen=c["telemetry_samples"])
        self.last_telemetry = 0.
        self.response_context = "MANUAL"
        self.motion_anomalies = {}
        self.motion_warning = None
        self.motion_paused = False
        self.last_disable_reason = None
        self.disable_history = deque(maxlen=8)

    def set_response_context(self, context):
        if context not in ("MANUAL", "TRACK", "COAST", "LOST", "HOLD"):
            raise ValueError("Invalid response context")
        if context != self.response_context:
            preserve = context in ("HOLD","LOST","COAST") and self.response_context in ("MANUAL","TRACK") and self.armed
            if not preserve:
                self.manual_ramp.reset()
                self.tracking_ramp.reset()
            self.guard.tracking_samples.clear()
            self.guard.tracking_error_since = [None,None]
            self.guard.tracking_reverse = [None,None]
        self.response_context = context
        if context == 'TRACK':
            self.hold_target = None
        if context in ("MANUAL","TRACK"):
            self.release_source = context

    def input(self, x, y):
        super().input(x,y)
        if x or y:
            self.hold_target = None
            if self.angle_move.active or self.angle_pending:
                self.angle_cancel = True
            if self.braking:
                ramp = self.tracking_ramp if self.response_context == "TRACK" else self.manual_ramp
                ramp.acceleration = list(self.brake_ramp.acceleration)
            self.braking = self.soft_hold = False

    def capture_pose(self):
        with self.lock:
            if (self.reference_valid and not any(self.feedback_issues)
                    and time.monotonic()-self.feedback_time <= self.c["pose_max_age_s"]):
                return self.angles
            return None

    def _fixed_hold_active(self):
        native_hold=(self.response_context in ('HOLD','LOST','COAST') and all(not engaged or held for engaged,held in
                     zip(self.native_tracker.engaged,self.native_tracker.gap_held)))
        return (self.hold_target is not None and self.armed and not self.recovery_active and not self.home_requested
                and self.outside_hold_origin is None and not self.angle_move.active and self.angle_pending is None
                and (not any(self.native_tracker.engaged) or native_hold) and not self.braking and not any(self.joystick) and self.response_context!='TRACK')

    def offer_tracking_goal(self, goal, stamp, active, velocity=(0.,0.)):
        with self.lock:self.native_tracker.offer(goal,stamp,active,velocity)

    def _heartbeat_check(self):
        if self.inertia_test_active:
            if self.abort_requested or self.stop_event.is_set():
                raise UserDisarm('Inertia experiment cancelled')
            return  # One-shot board trajectory does not depend on PC/vision heartbeat.
        return super()._heartbeat_check()

    def _latch_hold(self, target):
        self.hold_target=tuple(round(float(q),7)+0. for q in target)
        self.stream_target=self.manual_target=self.hold_target
        self.stream_velocity=(0.,0.)
        self.hold_error_since=[None,None];self.hold_error_best=[None,None]
        self.soft_hold=True

    def _advance_fixed_hold(self):
        self._heartbeat_check();self._require_reference(fresh=True);self._sync_driver_speed()
        now=time.monotonic();self.last_send=now
        # F32C's existing position loop holds this already-written reference.
        # Do not replace it with the hand-displaced encoder angle or a sliding
        # feedback-relative waypoint; that would move the equilibrium itself.
        self.stream_target=self.manual_target=self.hold_target
        self.stream_velocity=(0.,0.)
        for axis,(actual,goal) in enumerate(zip(self.angles,self.hold_target)):
            error=abs(goal-actual)
            if error<self.c['motion_error_deg']:
                self.hold_error_since[axis]=self.hold_error_best[axis]=None
                continue
            best=self.hold_error_best[axis]
            if best is None or error<best-self.c['motion_progress_deg']:
                self.hold_error_since[axis]=now;self.hold_error_best[axis]=error
            elif now-self.hold_error_since[axis]>=self.c['hold_block_timeout_s']:
                raise RuntimeError('SUSPECTED_BLOCK: fixed hold axis %d has no return progress for %.1fs' %
                                   (axis,self.c['hold_block_timeout_s']))

    def _position_lead_deg(self, axis):
        if self.inertia_test_active:
            lo,hi=self.c['limits'][axis]
            return hi-lo  # Test trajectory is not throttled by normal feedback lead.
        if self.native_tracker.engaged[axis]:
            return self.c['native_target_window_deg'][axis]
        if self.angle_move.active or self.angle_pending:
            return self.c["manual_position_lead_deg"][axis]
        if self.recovery_active:
            return self.c["home_position_lead_deg"][axis]
        if self.outside_hold_origin is not None:
            return self.c["manual_lead_deg"]
        if self.braking:
            return self.c["manual_position_lead_deg"][axis] if self.release_source=="MANUAL" else self.c["tracking_position_lead_deg"][axis]
        if self.response_context == "MANUAL" and self.joystick[axis]:
            return self.c["manual_position_lead_deg"][axis]
        if self.response_context == "TRACK":
            return self.c["tracking_position_lead_deg"][axis]
        return self.c["manual_lead_deg"]

    def _effective_accel(self):
        if self.recovery_active:
            return tuple(self.c['home_accel_deg_s2'])
        ceiling = (self.c["home_accel_deg_s2"] if self.recovery_active else
                   self.c["manual_accel_deg_s2"] if self.response_context == "MANUAL" else
                   self.c["tracking_accel_deg_s2"])
        if self.response_context in ("MANUAL", "TRACK") and not self.recovery_active:
            return tuple(ceiling)  # Fixed trajectory; feedback must not modulate it each packet.
        return tuple(a*min(1.,tuned/base) for a,tuned,base in
                     zip(ceiling,self.response.accel,self.c["command_accel_deg_s2"]))

    def _driver_rates(self):
        if self.inertia_test_active:
            speed=self.angle_pending[2] if self.angle_pending else self.angle_move.speed
            return (self.c['manual_rate'][0],speed)
        if self.angle_move.active or self.angle_pending:
            return self.c["manual_rate"]
        if self.recovery_active:
            return self.recovery_rate
        if self.native_tracker.wanted():
            return native_tracking_rates(self.c)
        if self._fixed_hold_active():
            return self.c['hold_return_rate']
        if self.outside_hold_origin is not None:
            return tuple(min(v,cap) for v,cap in zip(self.c["manual_rate"],(48.,36.)))
        if self.braking:
            return self.c["manual_rate"] if self.release_source=="MANUAL" else self.c["tracking_rate"]
        if self.response_context == "MANUAL":
            return self.c["manual_rate"]
        if self.response_context == "COAST":
            return self.c["prediction_coast_rate"]
        return self.c["tracking_rate"]

    def _sync_driver_speed(self):
        if isinstance(self.native_tracker, SpeedTrackingStream) and any(self.native_tracker.engaged):
            return  # In speed mode this opcode commands motion, not a position speed cap.
        for axis,(mid,rate) in enumerate(zip(self.c["motor_ids"],self._driver_rates())):
            rpm = min(self.c["motor_rpm"],max(1,int(math.ceil(rate/6.))))
            if self.programmed_rpm[axis] != rpm:
                self._write(set_speed(mid,rpm))
                self.programmed_rpm[axis] = rpm

    def _response_snapshot(self):
        result = self.response.snapshot()
        result["diagnostic_drive_scale"] = result["drive_scale"]
        result["diagnostic_rate_correction_deg_s"] = result["rate_correction_deg_s"]
        result.update(drive_scale=(1.,1.),rate_correction_deg_s=(0.,0.),adaptive_applied=False)
        result["software_accel_deg_s2"] = self._effective_accel()
        return result

    def request_arm(self, armed, reason="手动 DISARM／停止请求"):
        if not armed and not self.ready and self.phase == "CONNECTING":
            self.startup_cancelled = True
        super().request_arm(armed)
        if armed:
            self.last_disable_reason = None
        else:
            self.last_disable_reason = dict(reason=reason,time=time.monotonic(),phase=self.phase,
                actual_deg=self.angles,commanded_deg=self.sent_target,context=self.response_context)
            self.disable_history.append(dict(self.last_disable_reason))
            print("MOTOR_DISABLE_REQUEST "+json.dumps(self.last_disable_reason),flush=True)

    def _read(self, size=512, timeout=0):
        # Draining is nonblocking. A reply is nine bytes, so do not wait for
        # an impossible 512-byte batch before returning each valid response.
        started = time.monotonic()
        try:
            try:
                return self.serial.read(size, timeout=timeout)
            except TypeError:
                return self.serial.read(size, timeout)
        finally:
            self.timings.add("uart_read", (time.monotonic()-started)*1000)

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
            data = self._read(9, 2)
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
        if previous is not None and ((axis==0 and self.c.get('continuous_yaw',False)) or self.recovery_active or pending_home or self.outside_hold_origin is not None):
            value = previous[1]+signed_mechanical(value-previous[1])
        if previous is not None and (self.armed or self.arm_requested) and not self.inertia_test_active:
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
                if self.response_context in ("TRACK","LOST","COAST","HOLD") or self.angle_move.active:
                    allowance += min(self.c["tracking_feedback_margin_deg"],abs(self.stream_velocity[axis])*self.c["tracking_feedback_phase_s"])
                bound = min(bound,max(self.c["manual_lead_deg"]+self.c["recovery_margin_deg"],travel+allowance))
            if abs(value-previous[1]) > bound:
                # Fixed-position disturbance or near-zero HOME settling can
                # legitimately move faster than the outgoing position stream.
                # Accept no new target here. Reconfirm large encoder changes
                # with a second fresh consistent packet inside work limits.
                near_home = (self.recovery_active and self.sent_target is not None
                             and abs(self.sent_target[axis])<=self.c['home_tolerance_deg'])
                normal = self.c['limits'][axis][0]<=value*self.c['motor_signs'][axis]<=self.c['limits'][axis][1]
                candidate=self.hold_jump_candidate[axis]
                if normal and (self._fixed_hold_active() or near_home):
                    confirmed=(candidate is not None and 0<now-candidate[1]<=self.c['jump_confirm_window_s']
                               and abs(value-candidate[0])<=self.c['motor_rpm']*6*(now-candidate[1])+self.c['reference_jump_slack_deg'])
                    self.hold_jump_candidate[axis]=(value,now)
                    if confirmed:bound=abs(value-previous[1])
            if abs(value-previous[1]) > bound:
                self.last_angle_jump = dict(axis=axis, motor_id=mid, previous_deg=previous[1],
                                           current_deg=value, elapsed_s=dt, allowed_deg=bound)
                return self._unavailable_angle(axis, "Unexpected mechanical angle jump: %.1f->%.1f" % (previous[1],value))
        self.hold_jump_candidate[axis]=None
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
        started = time.monotonic()
        if self.last_measure_started is not None:
            self.timings.add("uart_cycle_interval", (started-self.last_measure_started)*1000)
        self.last_measure_started = started
        try:
            return self._measure_impl()
        finally:
            self.timings.add("uart_measure", (time.monotonic()-started)*1000)

    def _measure_impl(self):
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
        self.motion_paused = bool(self.armed and (degraded or (not self.inertia_test_active and
                                  (self.motion_warning or now-self.command_time > self.c["control_pause_age_s"])) ))
        if self.motion_paused:
            if isinstance(self.native_tracker, SpeedTrackingStream): self.native_tracker.stop()
            self.manual_ramp.reset()
            self.tracking_ramp.reset()
            self.guard.pause(now)
        else:
            self.guard.resume(now)
        if was_paused and not self.motion_paused and self.armed:
            # Resume from the measured pose, without a queued movement or old ramp.
            self.stream_target = self.manual_target = self.hold_target if self._fixed_hold_active() else self.angles
            self.stream_velocity = (0., 0.)
            self.last_send = time.monotonic()
        requested = (tuple(-math.copysign(rate,q) if abs(q) > self.c["home_tolerance_deg"] else 0.
                           for q,rate in zip(self.angles,self.recovery_rate)) if self.recovery_active else
                     tuple(v*r for v,r in zip(self.joystick,self.c["max_rate"])))
        self.response.update(time.monotonic(), self.angles, self.sent_target, requested,
                             self.armed, healthy=not self.motion_paused)
        if self.armed and not self.motion_paused and not self.inertia_test_active:
            if self.response_context == "TRACK" and not self.recovery_active:
                self.guard.update_tracking(now,self.angles,self.sent_target,self.stream_velocity)
            else:
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
                   planned_velocity_deg_s=self.stream_velocity, braking_active=self.braking,soft_hold=self.soft_hold,
                   angle_move=self.angle_move.snapshot(self.angle_pending),
                   inertia_test_active=self.inertia_test_active,
                   hold_target_deg=self.hold_target,hold_fixed_active=self._fixed_hold_active(),
                   manual_acceleration_deg_s2=tuple(self.manual_ramp.acceleration),
                   feedback_age_s=now-self.feedback_time, feedback_degraded=bool(any(self.feedback_issues)),
                   position_write_interval_s=tuple(self.position_write_interval),
                   position_write_step_deg=tuple(self.position_write_step),
                   position_write_count=tuple(self.position_write_count),
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
            # Saved single-turn zero needs no stationary/multiturn alignment.
            # A gravity-loaded pitch may already be falling while disabled.
            for mid in self.c["motor_ids"]:
                self._mechanical_feedback(mid)
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
        if self.inertia_test_active:
            for axis,(q,(lo,hi)) in enumerate(zip(angles,self.c['limits'])):
                if not lo<=q<=hi:
                    raise MotionProtectionFault('INERTIA_ANGLE_LIMIT: axis %d %.1f outside %.1f..%.1f' % (axis,q,lo,hi))
        elif self.recovery_active and self.recovery_origin is not None:
            margin = self.c["recovery_margin_deg"]
            for axis, (q, start, (lo,hi)) in enumerate(zip(angles, self.recovery_origin,self.c['limits'])):
                # Saved-zero homing may settle across zero inside the ordinary
                # working range. An outside start still must not drift farther
                # outward; final position writes remain local and toward zero.
                low = start if start < -self.c['home_tolerance_deg'] else lo
                high = start if start > self.c['home_tolerance_deg'] else hi
                check_corridor(axis,q,low,high,margin,self.c["recovery_hard_margin_deg"],"Recovery")
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
        started = time.monotonic()
        try:
            return self._write_impl(data)
        finally:
            self.timings.add("uart_transaction", (time.monotonic()-started)*1000)

    def _write_impl(self, data):
        speed_track = (isinstance(self.native_tracker, SpeedTrackingStream)
                       and data[1] in self.c['motor_ids']
                       and self.native_tracker.engaged[self.c['motor_ids'].index(data[1])])
        if speed_track and data[2] in (FUNCTION_MODE, FUNCTION_SPEED):
            if (len(data) != 7 or data[0] != FRAME_HEADER or data[-1] != FRAME_TAIL
                    or bcc_xor(data[:-2]) != data[-2]):
                raise RuntimeError('Invalid speed tracking frame')
            value = int.from_bytes(data[3:5], 'big', signed=data[2]==FUNCTION_SPEED)
            if data[2] == FUNCTION_MODE:
                if value not in (MODE_SPEED, self.c['position_mode']):
                    raise RuntimeError('Invalid speed tracking mode')
                if value == MODE_SPEED:
                    self._heartbeat_check(); self._require_reference(fresh=True)
                    if not self.armed or not self.arm_requested: raise UserDisarm('Speed mode requires ARM')
            elif value:
                self._heartbeat_check(); self._require_reference(fresh=True)
                if not self.armed or not self.arm_requested: raise UserDisarm('Speed requires ARM')
                if any(self.feedback_issues): raise FeedbackUnavailable('Speed requires fresh feedback')
                axis = self.c['motor_ids'].index(data[1])
                rate = speed_rpm_to_logical_rate(self.c, axis, value)
                lo, hi = self.c['limits'][axis]
                if axis == 1 and self.c.get('overhead_tracking', False): lo = max(lo, -90.)
                margin = self.c['speed_tracking_margin_deg']
                if (abs(rate) > native_tracking_rates(self.c)[axis]
                        or (rate > 0 and self.angles[axis] >= hi-margin)
                        or (rate < 0 and self.angles[axis] <= lo+margin)):
                    raise RuntimeError('Unsafe tracking speed')
            # Stop packets bypass stale feedback/heartbeat checks, but retain UART checks.
            return JoystickMotorService._write(self, data)
        native_test = data[1]==self.c['motor_ids'][1] and self.native_pitch_requested and self.inertia_test_active
        native_track = (data[1] in self.c['motor_ids'] and self.c['native_tracking']
                        and self.native_tracker.engaged[self.c['motor_ids'].index(data[1])])
        if (len(data)==7 and data[2]==FUNCTION_MODE and int.from_bytes(data[3:5],'big')==2 and (native_test or native_track)):
            if data[0]!=0x7A or data[-1]!=0x7B or bcc_xor(data[:-2])!=data[-2]:
                raise RuntimeError('Invalid native mode frame')
            return JoystickMotorService._write(self,data)  # UART worker's temporary single-turn T mode only.
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
            if axis==0 and self.c.get('continuous_yaw',False):canonical=target
            if units >= 3600 or not lo <= canonical <= hi:
                raise RuntimeError("Unsafe single-turn step; no target sent")
            if abs(canonical-actual) > self._position_lead_deg(axis)+.051:
                if self.response_context in ("TRACK","LOST","COAST","HOLD") or self.angle_move.active:
                    raise TrackingReplan("Tracking feedback changed after planning; refresh local target")
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
        now = time.monotonic()
        previous_time,previous_target = self.position_write_time[axis],self.position_intent[axis]
        self.position_write_interval[axis] = None if previous_time is None else now-previous_time
        self.position_write_step[axis] = None if previous_target is None else target-previous_target
        self.position_write_count[axis] += 1
        self.position_write_time[axis] = now
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
        if not self.home_requested and self.outside_hold_origin is None:
            self._latch_hold(angles)
        print("ARM: current pose preloaded; HOME separately requested=%s" % self.home_requested)
        return angles

    def request_home(self):
        if not self.armed:
            raise RuntimeError("先 ARM 在当前位置使能，再点 HOME；HOME 不会自动使能")
        super().request_home()
        self._cancel_angle("HOME requested")
        self.braking = self.soft_hold = False

    def request_angle_move(self, yaw, pitch, speed, inertia_test=False, native_pitch=False, native_accel=3):
        if native_pitch and (type(native_accel) is not int or not 1<=native_accel<=5):
            raise ValueError('Native acceleration parameter must be integer 1..5')
        values=(yaw,pitch,speed)
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
            raise ValueError("角度与速度必须为有限数字")
        if self.c.get('continuous_yaw',False):
            yaw=round(self.angles[0]+signed_mechanical(yaw-self.angles[0]),1)
            values=(yaw,pitch,speed)  # Absolute heading takes the nearest equivalent turn.
        if not 1.<=speed<=min(self.c['manual_rate']):
            raise ValueError("测试速度须在 1～%g°/s 内" % min(self.c['manual_rate']))
        if any(not lo<=q<=hi for q,(lo,hi) in zip(values[:2],self.c['limits'])):
            raise ValueError("目标角度超出工作限位")
        with self.lock:
            if (not self.armed or not self.reference_valid or self.fault or self.home_requested or self.recovery_active
                    or self.outside_hold_origin is not None or any(self.feedback_issues)
                    or time.monotonic()-self.feedback_time>self.c['pose_max_age_s']
                    or any(not lo<=q<=hi for q,(lo,hi) in zip(self.angles,self.c['limits']))):
                raise RuntimeError("先使能、完成回零，并等待正常范围内的新鲜角度反馈")
            if self.angle_move.active or self.angle_pending or self.native_pitch_engaged or self.braking or any(self.joystick) or max(abs(v) for v in self.stream_velocity)>1.:
                raise RuntimeError("请等待当前运动停止，或先停止角度测试")
            self.angle_pending=(round(yaw,1),round(pitch,1),float(speed),time.monotonic())
            self.angle_cancel=False
            self.inertia_test_active=bool(inertia_test)
            self.native_pitch_requested=bool(native_pitch and inertia_test)
            self.native_pitch_accel=native_accel
            self.native_pitch_sent=self.native_restore_pending=False

    def cancel_angle_move(self):
        with self.lock:
            self.angle_cancel=True

    def _cancel_angle(self, reason):
        self.native_restore_pending=self.native_pitch_engaged
        self.inertia_test_active=self.native_pitch_engaged
        self.native_pitch_requested=False
        if self.angle_pending:
            self.angle_move.state='CANCELLED';self.angle_move.reason=reason
        self.angle_pending=None
        self.angle_move.cancel(reason)
        self.angle_cancel=False

    def _native_position(self, q):
        axis=1;mid=self.c['motor_ids'][axis]
        packet=set_single_turn_position(mid,q/self.c['motor_signs'][axis])
        self._write(packet)
        self.position_units[axis]=int.from_bytes(packet[3:5],'big')
        self.last_position_send=time.monotonic()
        target=list(self.sent_target);target[axis]=round(q,1);self.sent_target=tuple(target)

    def _restore_native_pitch(self, target=None):
        if not self.native_pitch_engaged:
            self.native_restore_pending=False
            return
        mid=self.c['motor_ids'][1]
        try:
            if self.armed:
                self._require_reference(fresh=True)
                q=self.angles[1] if target is None else target
                self._native_position(q)  # Preload before switching, without enable toggles.
            self._write(select_mode(mid,self.c['position_mode']))
            self._write(set_acceleration(mid,self.c['motor_acceleration']))
            if self.armed:self._native_position(q)
        finally:
            self.native_pitch_engaged=self.native_pitch_requested=False
            self.native_pitch_sent=self.native_restore_pending=False
            self.inertia_test_active=False

    def _advance_native_pitch(self, now):
        mid=self.c['motor_ids'][1];goal=self.angle_move.goal[1]
        if not self.native_pitch_sent:
            self._native_position(self.angles[1])
            self.native_pitch_engaged=True
            self._write(select_mode(mid,2))
            rpm=max(1,int(math.ceil(self.angle_move.speed/6.)))
            self._write(set_speed(mid,rpm));self.programmed_rpm[1]=rpm
            self._write(set_acceleration(mid,self.native_pitch_accel))
            self._native_position(goal)
            self.angle_move.first_write=time.monotonic()
            self.angle_move.profile='native_t'
            distance=abs(goal-self.angle_move.origin[1]);speed=rpm*6.;accel=360.*self.native_pitch_accel
            self.angle_move.duration=(2*(distance/accel)**.5 if distance<=speed*speed/accel else distance/speed+speed/accel)
            self.native_pitch_sent=True
        self.angle_move.progress=min(self.angle_move.duration,now-self.angle_move.started)
        self.stream_target=self.manual_target=self.sent_target;self.stream_velocity=(0.,0.)
        self.last_send=now
        self.angle_move.observe(now,self.angles,self.response.velocity,self.c['home_tolerance_deg'],allow_timeout=False)
        if not self.angle_move.active:
            self._restore_native_pitch(goal)
            self._latch_hold(self.sent_target)

    def _advance_angle_move(self):
        self._heartbeat_check();self._require_reference(fresh=True);self._sync_driver_speed()
        now=time.monotonic();dt=min(.1,max(0.,now-self.last_send))
        with self.lock:
            if self.angle_pending:
                yaw,pitch,speed,accepted=self.angle_pending
                self.angle_move.begin(self.angles,(yaw,pitch),speed,accepted,now,self.c)
                self.angle_pending=None
                self.braking=self.soft_hold=False
                self.manual_ramp.reset();self.tracking_ramp.reset()
        if self.native_pitch_requested:
            return self._advance_native_pitch(now)
        waypoint=self.angle_move.waypoint(dt,self.angles,tuple(self._position_lead_deg(a) for a in (0,1)))
        self.last_send=now
        if waypoint is not None and now-self.last_position_send>=self.c['position_period_s']:
            target,velocity=waypoint
            sent=list(self.sent_target)
            for axis,(mid,q,sign) in enumerate(zip(self.c['motor_ids'],target,self.c['motor_signs'])):
                packet=set_single_turn_position(mid,q/sign);units=int.from_bytes(packet[3:5],'big')
                if units!=self.position_units[axis]:
                    self._write(packet);self.position_units[axis]=units
                    self.last_position_send=now
                    if self.angle_move.first_write is None:self.angle_move.first_write=time.monotonic()
                    sent[axis]=decoded_position(self.c,axis,units,self.angles[axis])
            self.sent_target=tuple(sent)
            self.stream_target=target;self.stream_velocity=velocity
        self.manual_target=self.sent_target
        if not self.inertia_test_active:self.guard.update(now,self.angles,self.sent_target)
        self.angle_move.observe(now,self.angles,self.response.velocity,self.c['home_tolerance_deg'],
                                allow_timeout=not self.inertia_test_active)
        if not self.angle_move.active:
            self.inertia_test_active=False
            self.stream_target=self.manual_target=self.sent_target
            self.stream_velocity=(0.,0.);self.soft_hold=True
            self._latch_hold(self.sent_target)

    def _rebase_yaw_for_home(self):
        if not self.c.get('continuous_yaw',False):return
        shift=round(self.angles[0]-signed_mechanical(self.angles[0]),7)
        if not shift:return
        raw_shift=shift/self.c['motor_signs'][0]
        with self.lock:
            for name in ('angles','manual_target','sent_target','stream_target','hold_target'):
                values=getattr(self,name,None)
                if values is not None:setattr(self,name,(values[0]-shift,values[1]))
            self.mechanical=(self.mechanical[0]-raw_shift,self.mechanical[1])
            for name in ('feedback_cache','mechanical_samples'):
                values=list(getattr(self,name))
                if values[0] is not None:
                    if name=='feedback_cache':values[0]=(values[0][0]-raw_shift,values[0][1])
                    else:values[0]=(values[0][0],values[0][1]-raw_shift)
                setattr(self,name,values)
            if self.position_intent[0] is not None:self.position_intent[0]-=shift
            self.hold_jump_candidate[0]=None
        self.response.reset();self.guard.reset()

    def _begin_home(self):
        while any(self.native_tracker.engaged) and not self.native_tracker.restore():
            self._heartbeat_check();self._measure();time.sleep(.005)
        if self.native_pitch_engaged:self._restore_native_pitch()
        self._rebase_yaw_for_home()  # Coordinate-only shift: HOME uses nearest mechanical zero.
        self.home_ramp.reset()
        self.hold_target = None
        self.braking = self.soft_hold = False
        self.manual_ramp.reset()
        self.arm_homing = self.home_requested = self.recovery_active = True
        self.recovery_origin = self._measure()[1]
        self.outside_hold_origin = None
        outside = any(not lo <= q <= hi for q,(lo,hi) in zip(self.recovery_origin, self.c["limits"]))
        self.recovery_rate = self.c["startup_home_rate"] if outside else self.c["home_rate"]

    def _manual_phase(self, moving):
        if (self.angle_move.active or self.angle_pending) and not moving:
            return "MANUAL"  # Existing HOME authorization accepts this active position mode.
        if self.braking and not moving:
            return "BRAKING"
        return super()._manual_phase(moving)

    def _release_hold(self):
        if any(self.native_tracker.engaged):
            self.manual_target=self.stream_target=self.sent_target
            self.stream_velocity=(0.,0.);self.hold_pending=False
            return
        self.hold_target = None  # An intentional manual release defines a new landing pose.
        eligible = (self.was_moving and self.response_context in ("MANUAL","TRACK","HOLD","LOST","COAST")
                    and self.armed and self.reference_valid and not self.recovery_active and not self.home_requested
                    and self.outside_hold_origin is None and not any(self.feedback_issues)
                    and all(lo<=q<=hi for q,(lo,hi) in zip(self.angles,self.c["limits"]))
                    and any(abs(v)>.5 for v in self.stream_velocity))
        if not eligible:
            self.braking = self.soft_hold = False
            return super()._release_hold()
        self.braking,self.soft_hold = True,False
        self.brake_started=time.monotonic()
        source=self.tracking_ramp if self.release_source=="TRACK" else self.manual_ramp
        self.brake_ramp.acceleration=list(source.acceleration)
        # Blend the bounded trajectory speed with filtered encoder velocity;
        # noisy feedback may not reverse or increase the outgoing motion.
        self.stream_velocity=tuple(math.copysign(min(abs(v),max(0.,.7*abs(v)+.3*measured*(1 if v>0 else -1))),v)
                                   if v else 0. for v,measured in zip(self.stream_velocity,self.response.velocity))
        self.manual_target=self.stream_target
        self.hold_pending=False

    def _advance_braking(self):
        self._heartbeat_check();self._require_reference(fresh=True);self._sync_driver_speed()
        now=time.monotonic();dt=min(.1,max(0.,now-self.last_send))
        target,velocity=[],[]
        for axis,(old,v,actual,accel,bounds) in enumerate(zip(self.stream_target,self.stream_velocity,self.angles,
                                                          self.c["brake_accel_deg_s2"],self.c["limits"])):
            cap=self.c["manual_rate"][axis] if self.release_source=="MANUAL" else self.c["tracking_rate"][axis]
            new=self.brake_ramp.advance(axis,v,0.,accel,dt,cap)
            new=math.copysign(max(0.,new*(1 if v>0 else -1)),v) if v else 0.
            candidate=old+(v+new)*.5*dt
            bounded=clip(candidate,*bounds)
            q=clip(bounded,actual-self._position_lead_deg(axis),actual+self._position_lead_deg(axis))
            # Feedback lag can hold a waypoint, but ordinary braking must not
            # retract an already-sent target and pull against ongoing inertia.
            if v and (q-self.sent_target[axis])*v<0:
                q=clip(self.sent_target[axis],*bounds)
            if bounded!=candidate:
                new=0.;self.brake_ramp.reset(axis)
            target.append(q);velocity.append(new)
        self.stream_target,self.stream_velocity=tuple(target),tuple(velocity)
        self.last_send=now
        if now-self.last_position_send>=self.c["position_period_s"]:
            sent=list(self.sent_target)
            for axis,(mid,q,sign) in enumerate(zip(self.c["motor_ids"],target,self.c["motor_signs"])):
                packet=set_single_turn_position(mid,q/sign);units=int.from_bytes(packet[3:5],"big")
                if units!=self.position_units[axis]:
                    self._write(packet);self.position_units[axis]=units;self.last_position_send=now
                    sent[axis]=decoded_position(self.c,axis,units,self.angles[axis])
            self.sent_target=tuple(sent)
        self.guard.update(now,self.angles,self.sent_target)
        if max(abs(v) for v in self.stream_velocity)<.4 or now-self.brake_started>=self.c["brake_timeout_s"]:
            self.braking=False;self.soft_hold=True
            self.stream_target=self.manual_target=self.sent_target
            self.stream_velocity=(0.,0.);self.manual_ramp.reset();self.tracking_ramp.reset();self.brake_ramp.reset()
            self.hold_pending=False
            self._latch_hold(self.sent_target)

    def _advance(self, goal, rate):
        try:
            return self._advance_impl(goal,rate)
        except TrackingReplan:
            # Fresh feedback may move beyond an obsolete hold target when TRACK
            # switches to LOST/HOLD and its lead shrinks. Drop the unsent packet;
            # recapture a local hold on the next cycle, without revoking ARM.
            self.stream_target = self.manual_target = self.angles
            self.stream_velocity = (0.,0.)
            self.manual_ramp.reset()
            self.tracking_ramp.reset()
            self.hold_pending = self.response_context != "TRACK"
            self.last_send = time.monotonic()

    def _advance_impl(self, goal, rate):
        if self.native_restore_pending:self._restore_native_pitch()
        if any(self.native_tracker.engaged) and not self.native_tracker.wanted():
            if not self.native_tracker.restore():
                self._heartbeat_check();self._require_reference(fresh=False)
                return
            goal=self.manual_target
        pause = bool(any(self.feedback_issues) or (not self.inertia_test_active and
                     (self.motion_warning or time.monotonic()-self.command_time > self.c["control_pause_age_s"])))
        if pause:
            if isinstance(self.native_tracker, SpeedTrackingStream): self.native_tracker.stop()
            self.home_ramp.reset()
            self.braking = self.soft_hold = False
            self._heartbeat_check()
            self._require_reference(fresh=False)  # No motion write: retry on the next normal read cycle.
            self.last_send = time.monotonic()
            self.stream_velocity = (0., 0.)
            self.manual_ramp.reset()
            self.tracking_ramp.reset()
            self.motion_paused = True
            self.guard.pause(self.last_send)
            return  # Retain enable, but do not send a new position during the short grace.
        if self.motion_paused:
            self.motion_paused = False
            self.stream_target = self.manual_target = self.hold_target if self._fixed_hold_active() else self.angles
            self.stream_velocity = (0.,0.)
            self.guard.resume(time.monotonic())
        if not self.recovery_active:
            if self.native_tracker.wanted():
                return self.native_tracker.advance()
            if self.c['native_tracking'] and self.response_context=='TRACK' and self.angle_pending is None and not self.angle_move.active:
                self._heartbeat_check();self._require_reference(fresh=True)
                self.last_send=time.monotonic();self.stream_velocity=(0.,0.)
                return  # Do not fall back to old rate packets when the absolute aim is stale.
            if self.angle_cancel:
                active=self.angle_move.active or self.angle_pending is not None
                self._cancel_angle("Operator interrupted")
                if self.native_restore_pending:self._restore_native_pitch()
                if active:
                    was_moving=self.was_moving;self.was_moving=True
                    self.release_source="MANUAL"
                    self._release_hold();self.was_moving=was_moving
                    goal=self.manual_target  # Discard this cycle's obsolete pre-cancel goal.
            if self.angle_move.active or self.angle_pending:
                return self._advance_angle_move()
            if self.braking:
                return self._advance_braking()
            if self._fixed_hold_active():
                return self._advance_fixed_hold()
            if self.soft_hold and not any(self.joystick):
                self._heartbeat_check();self._require_reference(fresh=True)
                self.last_send=time.monotonic()
                self.guard.update(self.last_send,self.angles,self.sent_target)
                return  # Retain final landing target instead of chasing measured overshoot.
            if self.response_context == "TRACK" and self.outside_hold_origin is None:
                try:
                    return self._advance_velocity_stream()
                except TrackingReplan:
                    self.stream_target = self.manual_target = self.angles
                    self.stream_velocity = (0.,0.)
                    self.tracking_ramp.reset()
                    self.last_send = time.monotonic()
                    return
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
            v = self.home_ramp.advance(axis,old_v,desired,accel,dt,speed)
            q = clip(old+(old_v+v)*.5*dt, min(old, 0.), max(old, 0.))
            q = clip(q, actual-self._position_lead_deg(axis), actual+self._position_lead_deg(axis))
            q = clip(q, min(start, 0.), max(start, 0.))
            q = clip(q, min(actual, 0.), max(actual, 0.))
            if q == 0:
                v = 0.
                self.home_ramp.reset(axis)
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
                    self.last_position_send = now
                    self.position_units[axis] = units
                    canonical = signed_mechanical(units/10.)*sign
                    sent[axis] = self.angles[axis]+signed_mechanical(canonical-self.angles[axis])
            self.sent_target = tuple(sent)
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
            # The F32C already closes its position/velocity loops. Constant stick
            # input must stay constant; an additional error-derived gain/correction
            # loop turns ordinary following lag and overshoot into speed ripple.
            desired = clip(v*speed, -cap, cap)
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
                     self.tracking_ramp.advance(axis, old_v, desired, accel, dt, cap) if self.response_context == "TRACK" else
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
            if (self.response_context in ("MANUAL","TRACK") and v and not force_hold and self.outside_hold_origin is None
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
            if q != candidate and candidate != clip(candidate, *bounds):
                # Lead limiting is anti-windup, not a reason to restart the ramp
                # every cycle while the real motor is still catching up.
                new_v = 0.
                self.manual_ramp.reset(axis)
                self.tracking_ramp.reset(axis)
            if force_hold or (self.outside_hold_origin is not None and new_v == 0.):
                self.manual_ramp.reset(axis)
                self.tracking_ramp.reset(axis)
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
                    self.last_position_send = now
                    self.position_units[axis] = units
                    sent[axis] = decoded_position(self.c,axis,units,self.angles[axis])
            self.sent_target = tuple(sent)
            self.hold_pending = False
        if self.response_context != "TRACK":
            self.guard.update(now, self.angles, self.sent_target)

    def _home_and_clear(self):
        try:
            if not self.recovery_active:
                self._begin_home()
                self.stream_target = self.angles
                self.stream_velocity = (0., 0.)
            super()._home_and_clear()
            self.startup_completed = True
            self._latch_hold(self.sent_target)
        finally:
            self.arm_homing = False
            self.recovery_active = False
            self.recovery_origin = None
            self.outside_hold_origin = None

    def _disable_now(self):
        self.home_ramp.reset()
        self.hold_jump_candidate = [None,None]
        self.hold_target = None
        self._cancel_angle("DISARM requested")
        self.braking = self.soft_hold = False
        super()._disable_now()
        if self.native_pitch_engaged:self._restore_native_pitch()
        if any(self.native_tracker.engaged):self.native_tracker.restore()
        self.response.reset()
        self.manual_ramp.reset()
        self.tracking_ramp.reset()
        self.motion_paused = False
        self.position_intent = [None, None]
        self.programmed_rpm = [None, None]
        self.arm_homing = False
        self.recovery_active = False
        self.recovery_origin = None
        self.outside_hold_origin = None

    def _latch_fault(self, reason, reference_lost=False):
        self.home_ramp.reset()
        self.hold_jump_candidate = [None,None]
        self.hold_target = None
        self._cancel_angle(reason)
        self.braking = self.soft_hold = False
        if self.last_feedback_fault is None:
            now = time.monotonic()
            self.last_feedback_fault = dict(reason=reason,time=now,
                actual_deg=self.angles,commanded_deg=self.sent_target,
                planned_velocity_deg_s=self.stream_velocity,control_context=self.response_context,
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
        if self.native_pitch_engaged:
            try:self._restore_native_pitch()
            except Exception as exc:print('NATIVE_MODE_RESTORE_FAILED',str(exc),flush=True)
        if any(self.native_tracker.engaged):
            try:self.native_tracker.restore()
            except Exception as exc:print('TRACKING_MODE_RESTORE_FAILED',str(exc),flush=True)

    def _recheck_stationary_required(self):
        # The saved single-turn zero is not recalibrated here. An eccentric
        # disabled pitch may fall; fresh valid dual-axis feedback is sufficient.
        return not self.c["mechanical_zero_confirmed"]

    def _perform_recheck(self):
        self.hold_jump_candidate = [None,None]
        self.recheck_started = time.monotonic()
        self.recheck_finished = None
        self.recheck_previous_fault = dict(reason=self.fault,feedback_fault=self.last_feedback_fault,
                                           disable_reason=self.last_disable_reason,angles=self.angles)
        self.feedback_cache = [None, None]
        self.feedback_missing_since = [None, None]
        self.strict_feedback = True
        try:
            super()._perform_recheck()
        finally:
            self.strict_feedback = False
            self.recheck_finished = time.monotonic()
        self.zero_confirmed = self.c["mechanical_zero_confirmed"]
        self.guard.last_event = None
        self.last_feedback_fault = None
        self.motion_anomalies = {}
        self.motion_warning = None

    def snapshot(self):
        result = super().snapshot()
        if self.c.get('continuous_yaw',False):result['limits']=((None,None),tuple(self.c['limits'][1]))
        result.update(continuous_yaw=self.c.get('continuous_yaw',False),
                      yaw_heading_deg=signed_mechanical(self.angles[0]),yaw_angle_unwrapped_deg=self.angles[0])
        result["app_version"] = self.c["app_version"]
        result["angle_move"] = self.angle_move.snapshot(self.angle_pending)
        result['inertia_test_active']=self.inertia_test_active
        result['native_pitch_engaged']=self.native_pitch_engaged
        result['tracking_motor_control']=self.c['tracking_motor_control']
        result['speed_mode_signs']=self.c['speed_mode_signs']
        result['speed_tracking_accel_deg_s2']=self.c['speed_tracking_accel_deg_s2']
        result['speed_tracking_brake_deg_s2']=self.c['speed_tracking_brake_deg_s2']
        result['speed_tracking_driver_accel']=self.c['speed_tracking_driver_accel']
        result['speed_tracking_diagnostics']=dict(getattr(self.native_tracker,'diagnostics',{}))
        result['tracking_command_rpm']=tuple(getattr(self.native_tracker,'command_rpm',(0,0)))
        result.update(native_tracking_enabled=self.c['native_tracking'],native_tracking_engaged=tuple(self.native_tracker.engaged),
                      native_tracking_rate=native_tracking_rates(self.c),native_tracking_accel=self.c['native_tracking_accel'],
                      native_tracking_rate_limit=self.c['native_tracking_rate'])
        result.update(native_tracking_setup_count=self.native_tracker.setup_count,
                      native_tracking_gap_hold_count=self.native_tracker.gap_count,
                      native_aim_rejected_frames=self.native_tracker.aim_rejections,
                      native_measured_velocity_deg_s=tuple(self.native_tracker.velocity),
                      native_damping_deg=tuple(self.native_tracker.damping),
                      native_center_deadzone=self.c['native_center_deadzone'])
        result['planned_velocity_deg_s']=self.stream_velocity
        result.update(hold_return_rate=self.c['hold_return_rate'],home_jerk_deg_s3=self.c['home_jerk_deg_s3'])
        result.update(hold_target_deg=self.hold_target,hold_fixed_active=self._fixed_hold_active(),
                      hold_error_deg=None if self.hold_target is None else tuple(q-a for q,a in zip(self.hold_target,self.angles)),
                      hold_disturbance_limits=result['limits'],hold_block_timeout_s=self.c['hold_block_timeout_s'])
        result["recheck_diagnostic"] = dict(stationary_required=self._recheck_stationary_required(),
            started=self.recheck_started,finished=self.recheck_finished,previous=self.recheck_previous_fault)
        result.update(braking_active=self.braking,soft_hold=self.soft_hold,
                      manual_accel_deg_s2=self.c["manual_accel_deg_s2"],brake_timeout_s=self.c["brake_timeout_s"])
        result.update(last_disable_reason=self.last_disable_reason,disable_history=list(self.disable_history),
                      motor_signs=self.c["motor_signs"],
                      direction_mapping=dict(right_input_mechanical_sign=self.c["joystick_signs"][0],
                                             up_input_mechanical_sign=self.c["joystick_signs"][1]))
        result["protection"] = dict(home_feedback_margin_deg=self.c["recovery_margin_deg"],
            tracking_reverse_deg=self.c["tracking_reverse_deg"],tracking_reverse_persist_s=self.c["tracking_reverse_persist_s"],
            tracking_stall_basis="physical movement range with sustained position error",
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
                      position_period_s=self.c["position_period_s"],motor_loop_period_s=self.c["motor_loop_period_s"],
                      position_write_interval_s=tuple(self.position_write_interval),
                      position_write_step_deg=tuple(self.position_write_step),
                      position_write_count=tuple(self.position_write_count),
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
