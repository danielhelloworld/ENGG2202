"""Latest absolute visual aim, retargeted inside F32C's single-turn planner."""


def velocity_damped_goal(goal, measured_velocity, target_velocity, c):
    correction=clip((measured_velocity-target_velocity)*c['native_velocity_damping_s'],
                    -c['native_damping_max_deg'],c['native_damping_max_deg'])
    return goal-correction,correction


class NativeTrackingStream:
    def __init__(self, driver, c):
        self.driver,self.c=driver,c
        self.engaged=[False,False]
        self.message=None
        self.last_send=0.
        self.observation_stamp=None
        self.offered_at=None
        self.gap_held=[False,False]
        self.centered=[False,False]
        self.setup_count=self.gap_count=0
        self.accepted_aim=self.aim_candidate=None
        self.aim_rejections=0
        self.previous_pose=[None,None]
        self.velocity=[0.,0.]
        self.damping=[0.,0.]

    def offer(self, goal, stamp, active, velocity=(0.,0.)):
        if not math.isfinite(stamp):raise ValueError('Invalid native observation timestamp')
        if stamp!=self.observation_stamp:self.offered_at=time.monotonic()
        self.observation_stamp=stamp
        if goal is None:
            self.message=None;return
        if (len(goal)!=2 or len(active)!=2 or len(velocity)!=2 or any(not math.isfinite(v) for v in velocity) or not math.isfinite(stamp)
                or any(not math.isfinite(q) for q in goal)):
            raise ValueError('Invalid native visual aim')
        goal=tuple(goal)
        if self.accepted_aim is not None and stamp>self.accepted_aim[1]:
            old,old_stamp=self.accepted_aim
            dt=min(.25,stamp-old_stamp)
            limit=self.c['motor_rpm']*6*dt+4.
            if any(abs(q-p)>limit for q,p in zip(goal,old)):
                candidate=self.aim_candidate
                if candidate is not None and candidate[1]==stamp:return
                self.aim_candidate=(goal,stamp)
                if not (candidate is not None and 0<stamp-candidate[1]<=.15
                        and all(abs(q-p)<=self.c['motor_rpm']*6*(stamp-candidate[1])+4. for q,p in zip(goal,candidate[0]))):
                    self.aim_rejections+=1
                    return  # Do not chase a single-frame world-aim discontinuity.
        self.accepted_aim=(goal,stamp);self.aim_candidate=None
        self.message=(goal,stamp,tuple(bool(v) for v in active),self.offered_at,tuple(velocity))

    def sample_velocity(self):
        d=self.driver
        for axis in (0,1):
            stamp=d.mechanical_feedback_times[axis]
            previous=self.previous_pose[axis]
            if stamp is None or (previous is not None and stamp<=previous[1]):continue
            if previous is not None:
                dt=stamp-previous[1]
                if .005<=dt<=.2:
                    rate=clip((d.angles[axis]-previous[0])/dt,-self.c['motor_rpm']*6*1.5,self.c['motor_rpm']*6*1.5)
                    self.velocity[axis]+=dt/(.03+dt)*(rate-self.velocity[axis])
                else:self.velocity[axis]=0.
            self.previous_pose[axis]=(d.angles[axis],stamp)

    def wanted(self):
        d=self.driver
        tracking=self.message is not None and d.response_context=='TRACK'
        gap=(d.response_context in ('LOST','COAST','HOLD') or (d.response_context=='TRACK' and self.message is None)) and any(self.engaged)
        return (self.c['native_tracking'] and (tracking or gap) and self.observation_stamp is not None
                and 0<=time.monotonic()-self.observation_stamp<=self.c['vision_max_age']
                and d.armed and not d.home_requested and not d.recovery_active
                and d.outside_hold_origin is None and not d.angle_move.active and d.angle_pending is None)

    def hold_gap(self):
        d=self.driver
        for axis in (0,1):
            if self.engaged[axis] and not self.gap_held[axis]:
                self.position(axis,d.angles[axis])
                self.gap_held[axis]=True
                self.gap_count+=1
        d.stream_target=d.manual_target=d.sent_target
        d.stream_velocity=(0.,0.);d.last_send=time.monotonic()
        d.hold_pending=False;d.braking=d.soft_hold=False
        if d.response_context in ('HOLD','LOST','COAST') and d.hold_target is None:d._latch_hold(d.sent_target)

    def position(self, axis, q):
        d=self.driver;mid=self.c['motor_ids'][axis]
        packet=set_single_turn_position(mid,q/self.c['motor_signs'][axis])
        d._write(packet)
        units=int.from_bytes(packet[3:5],'big');d.position_units[axis]=units
        target=list(d.sent_target);target[axis]=decoded_position(self.c,axis,units,d.angles[axis])
        d.sent_target=tuple(target);d.last_position_send=time.monotonic()

    def advance(self):
        d=self.driver;d._heartbeat_check();d._require_reference(fresh=True)
        self.sample_velocity()
        with d.lock:message=self.message
        if not self.wanted():
            self.restore();return
        if message is None or d.response_context in ('LOST','COAST','HOLD'):
            self.hold_gap()
            if d._fixed_hold_active():d._advance_fixed_hold()
            return
        for axis,mid in enumerate(self.c['motor_ids']):
            if not self.engaged[axis]:
                self.position(axis,d.angles[axis])
                self.engaged[axis]=True
                d._write(select_mode(mid,2))
                d._write(set_acceleration(mid,self.c['native_tracking_accel'][axis]))
                rpm=int(math.ceil(native_tracking_rates(self.c)[axis]/6.))
                d._write(set_speed(mid,rpm));d.programmed_rpm[axis]=rpm
                self.setup_count+=1
        now=time.monotonic()
        if not self.wanted():
            self.restore();return
        if self.message is None or d.response_context in ('LOST','COAST','HOLD'):
            self.hold_gap();return
        # Mode/speed setup can span several visual frames. Use the newest aim
        # after it completes rather than sending the pre-setup observation.
        with d.lock:message=self.message
        if message is None or not 0<=time.monotonic()-message[1]<=self.c['vision_max_age']:
            self.restore();return
        goal,stamp,active,offered_at,target_velocity=message
        self.gap_held=[False,False]
        if now-self.last_send>=self.c['native_target_period_s']:
            for axis,(q,actual,(lo,hi),moving) in enumerate(zip(goal,d.angles,self.c['limits'],active)):
                if axis==1 and self.c.get('overhead_tracking',False):lo=max(lo,-90.)
                relative_velocity=self.velocity[axis]-target_velocity[axis]
                braking=self.c['native_velocity_damping_s']>0 and abs(relative_velocity)>3.
                if not moving and self.centered[axis] and not braking:continue
                q,self.damping[axis]=velocity_damped_goal(q,self.velocity[axis],target_velocity[axis],self.c)
                lead=self.c['native_target_window_deg'][axis]
                q=clip(clip(q,lo,hi),actual-lead,actual+lead)
                if abs(q-d.sent_target[axis])>=self.c['native_target_deadband_deg']:
                    self.position(axis,q)
                    d.timings.add('native_goal_age_at_send',(time.monotonic()-stamp)*1000)
                    d.timings.add('native_offer_to_write',(time.monotonic()-offered_at)*1000)
                self.centered[axis]=not moving and not braking
            self.last_send=now
        d.stream_target=d.manual_target=d.sent_target
        d.stream_velocity=tuple(clip(v*r,-cap,cap) for v,r,cap in
                                 zip(d.joystick,self.c['max_rate'],native_tracking_rates(self.c)))
        d.last_send=now;d.hold_target=None;d.hold_pending=False;d.braking=d.soft_hold=False

    def restore(self):
        d=self.driver
        if not any(self.engaged):return True
        if d.armed and (not d.reference_valid or any(d.feedback_issues)
                        or time.monotonic()-d.feedback_time>=self.c['reference_max_age_s']):
            return False
        if d.armed and max(abs(v) for v in d.response.velocity)>8.:
            self.hold_gap()
            return False  # Brake in T mode before switching a moving axis to direct mode.
        for axis,mid in enumerate(self.c['motor_ids']):
            if not self.engaged[axis]:continue
            q=d.angles[axis]
            if d.armed:self.position(axis,q)
            d._write(select_mode(mid,self.c['position_mode']))
            d._write(set_acceleration(mid,self.c['motor_acceleration']))
            if d.armed:self.position(axis,q)
            self.engaged[axis]=False
            self.gap_held[axis]=False
            self.centered[axis]=False
        d.stream_target=d.manual_target=d.sent_target;d.stream_velocity=(0.,0.)
        self.accepted_aim=self.aim_candidate=None
        self.previous_pose=[None,None];self.velocity=[0.,0.];self.damping=[0.,0.]
        d.manual_ramp.reset();d.tracking_ramp.reset();d.hold_pending=False
        if d.armed and not any(d.joystick):d._latch_hold(d.sent_target)
        return True


def speed_rpm_to_logical_rate(c, axis, rpm):
    """Decode signed speed using speed-mode polarity and the encoder-axis sign."""
    return rpm * 6. * c['motor_signs'][axis] * c['speed_mode_signs'][axis]


def logical_rate_to_speed_rpm(c, axis, rate):
    """Truncate toward zero; the speed-mode sign is independent of visual input."""
    return int(rate / speed_rpm_to_logical_rate(c, axis, 1))


def speed_tracking_request(c, axis, error, measured, feedforward, moving, feedback_age=0.):
    """Bound relative closing speed by distance, braking and feedback latency."""
    relative = measured-feedforward
    correction = c['speed_tracking_kp'][axis]*error-c['speed_tracking_kd'][axis]*relative
    brake = c['speed_tracking_brake_deg_s2'][axis]
    reaction = c['speed_tracking_brake_latency_s']+clip(feedback_age, 0., .1)
    safe = math.sqrt((brake*reaction)**2+2*brake*abs(error))-brake*reaction
    correction = clip(correction, -safe, safe)
    if abs(feedforward)>=c['speed_tracking_motion_band_deg_s'][1] and correction*feedforward>0:
        # Catch up to a fast target without adding an unbounded closing surge.
        correction=math.copysign(min(abs(correction),c['speed_tracking_catchup_deg_s'][axis]),correction)
    if relative*error > 0 and abs(relative) > safe:
        correction = 0.  # Brake toward target velocity before crossing the aim.
    requested = feedforward+correction
    if not moving and abs(feedforward) < 1.: requested = 0.
    cap = native_tracking_rates(c)[axis]
    return clip(requested, -cap, cap)


def slew_tracking_speed(previous, requested, dt, accel, brake):
    """Brake faster than launch; a reversal must decelerate through zero first."""
    if previous*requested < 0.:
        stop_s = abs(previous)/brake
        if dt <= stop_s:
            return previous-math.copysign(brake*dt,previous)
        return math.copysign(min(abs(requested),accel*(dt-stop_s)),requested)
    limit = brake if abs(requested) < abs(previous) else accel
    return previous+clip(requested-previous,-limit*dt,limit*dt)


def tracking_motion_weight(c,velocity):
    lo,hi=c['speed_tracking_motion_band_deg_s']
    return clip((abs(velocity)-lo)/(hi-lo),0.,1.)


class SpeedTrackingStabilizer:
    """Filter only fresh world-aim observations near center; retain far-field response."""
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self):
        self.goal = self.feedforward = None
        self.stamp = None
        self.fast_counts = [0,0]

    def offer(self, goal, feedforward, actual, stamp):
        if self.stamp is not None and stamp == self.stamp:
            return self.goal, self.feedforward
        dt = .02 if self.stamp is None else clip(stamp-self.stamp,.001,.1)
        if self.goal is None or stamp < self.stamp:
            self.goal,self.feedforward = tuple(goal),tuple(feedforward)
        else:
            goals,rates = [],[]
            for axis,(q,ff,pose) in enumerate(zip(goal,feedforward,actual)):
                near = 1.-clip(abs(q-pose)/self.c['speed_tracking_smooth_radius_deg'],0.,1.)
                consistent = ff*self.feedforward[axis]>0 and abs(ff)>=self.c['speed_tracking_motion_band_deg_s'][0]
                self.fast_counts[axis] = min(8,self.fast_counts[axis]+1) if consistent else 0
                if self.fast_counts[axis]>=2:
                    near *= 1.-tracking_motion_weight(self.c,ff)
                old = self.goal[axis]
                if (old-pose)*(q-pose) < 0: old = pose  # Drop the old push after crossing center.
                tau = self.c['speed_tracking_goal_tau_s']*near
                goals.append(old+dt/(tau+dt)*(q-old))
                old_ff = self.feedforward[axis]
                if old_ff*ff < 0: old_ff = 0.
                tau = self.c['speed_tracking_feedforward_tau_s']*near
                rates.append(old_ff+dt/(tau+dt)*(ff-old_ff))
            self.goal,self.feedforward = tuple(goals),tuple(rates)
        self.stamp = stamp
        return self.goal,self.feedforward


def stable_speed_rpm(c, axis, rate, previous):
    """Require a little headroom before increasing a quantized RPM magnitude.

    Reductions and safety stops are immediate; never keep a quantized command
    whose magnitude exceeds the current bounded angular-rate request.
    """
    candidate = logical_rate_to_speed_rpm(c,axis,rate)
    if candidate == 0: return 0
    if candidate*previous < 0: return 0  # Send zero before reversing the physical command.
    magnitude = abs(rate)/6.
    if abs(candidate) < 4 and abs(candidate) > abs(previous):
        allowed = min(abs(candidate),int(max(0.,magnitude-c['speed_tracking_rpm_hysteresis'])+1e-9))
        # A large acceleration step may cross multiple bins: send the lower
        # available bin instead of waiting forever at an exact RPM boundary.
        candidate = int(math.copysign(max(abs(previous),allowed),candidate))
    return candidate


class SpeedTrackingStream(NativeTrackingStream):
    """Visual bearing error -> signed RPM, with zero-speed handoff to position hold."""
    def __init__(self, driver, c):
        super().__init__(driver, c)
        self.command_velocity = [0., 0.]
        self.command_rpm = [0, 0]
        self.control_time = None
        self.stabilizer = SpeedTrackingStabilizer(c)
        self.diagnostics = {}

    def stop(self):
        # Zero speed must remain writable when feedback or the UI heartbeat is stale.
        for axis, mid in enumerate(self.c['motor_ids']):
            if self.engaged[axis]:
                self.driver._write(set_speed(mid, 0))
        self.command_velocity = [0., 0.]
        self.command_rpm = [0, 0]
        self.driver.stream_velocity = (0., 0.)
        self.control_time = None
        self.stabilizer.reset()
        self.diagnostics = {}

    def hold_gap(self):
        self.stop()
        self.gap_count += 1
        self.restore()

    def advance(self):
        d = self.driver
        d._heartbeat_check(); d._require_reference(fresh=True)
        self.sample_velocity()
        if not self.wanted():
            self.restore(); return
        with d.lock: message = self.message
        if message is None or d.response_context != 'TRACK':
            self.hold_gap(); return
        for axis, mid in enumerate(self.c['motor_ids']):
            if not self.engaged[axis]:
                d._write(set_speed(mid, 0))  # Clear the old position-mode speed before changing modes.
                self.engaged[axis] = True
                d.position_intent[axis] = None  # A speed command has no position corridor.
                d._write(select_mode(mid, MODE_SPEED))
                d._write(set_acceleration(mid, self.c['speed_tracking_driver_accel'][axis]))
                self.setup_count += 1
        now = time.monotonic()
        # Re-read the latest mailbox after UART setup, which may span visual frames.
        with d.lock: message = self.message
        if not self.wanted() or message is None or d.response_context != 'TRACK':
            self.hold_gap(); return
        if now - self.last_send < self.c['native_target_period_s']: return
        goal, stamp, active, offered_at, target_velocity = message
        if not 0 <= now - stamp <= self.c['vision_max_age']:
            self.hold_gap(); return
        dt = .02 if self.control_time is None else clip(now-self.control_time, .001, .1)
        self.control_time = now
        # Bounds are applied again after smoothing so a filtered goal cannot escape limits.
        goal,target_velocity = self.stabilizer.offer(goal,target_velocity,d.angles,stamp)
        mailbox_age = max(0.,now-offered_at)
        lead_s = min(mailbox_age,self.c['speed_tracking_mailbox_lead_s'])
        goal = tuple(q+ff*lead_s for q,ff in zip(goal,target_velocity))
        rates = []
        targets = []
        details = []
        for axis, (q, actual, moving, ff, cap, accel) in enumerate(zip(
                goal, d.angles, active, target_velocity, native_tracking_rates(self.c),
                self.c['speed_tracking_accel_deg_s2'])):
            brake = self.c['speed_tracking_brake_deg_s2'][axis]
            lo, hi = self.c['limits'][axis]
            if axis == 1 and self.c.get('overhead_tracking', False): lo = max(lo, -90.)
            q = clip(q, lo, hi)
            error = q-actual
            feedback_stamp = d.mechanical_feedback_times[axis]
            age = 0. if feedback_stamp is None else max(0.,now-feedback_stamp)
            requested = speed_tracking_request(self.c,axis,error,self.velocity[axis],ff,moving,age)
            motion = tracking_motion_weight(self.c,ff)
            near = (1.-clip(abs(error)/self.c['speed_tracking_smooth_radius_deg'],0.,1.))*(1.-motion)
            launch_accel = (accel+(self.c['speed_tracking_fast_accel_deg_s2'][axis]-accel)*motion)
            launch_accel *= 1.-near*(1.-self.c['speed_tracking_near_accel_ratio'])
            v = slew_tracking_speed(self.command_velocity[axis],requested,dt,launch_accel,brake)
            # Reserve feedback age and braking distance inside the pitch envelope.
            distance = (hi-actual if v > 0 else actual-lo)-self.c['speed_tracking_margin_deg']
            reaction = self.c['reference_max_age_s'] + self.c['native_target_period_s']
            safe = max(0., math.sqrt((brake*reaction)**2+2*brake*max(0.,distance))-brake*reaction)
            v = clip(v, -safe, safe)
            self.command_velocity[axis] = v
            details.append(dict(error_deg=error,target_velocity_deg_s=ff,measured_velocity_deg_s=self.velocity[axis],
                requested_deg_s=requested,bounded_deg_s=v,launch_accel_deg_s2=launch_accel,
                rpm_ceiling=abs(requested)>=cap-.01,soft_limit_clipped=abs(v)>=safe-.01,
                catchup_limited=(abs(ff)>=self.c['speed_tracking_motion_band_deg_s'][1] and (requested-ff)*ff>0
                    and abs(requested-ff)>=self.c['speed_tracking_catchup_deg_s'][axis]-.01),
                acceleration_limited=abs(v-requested)>.5))
            # Truncate toward zero: integer RPM must never round beyond a safety bound.
            rpm = stable_speed_rpm(self.c, axis, v, self.command_rpm[axis])
            d._write(set_speed(self.c['motor_ids'][axis], rpm))
            self.command_rpm[axis] = rpm
            d.programmed_rpm[axis] = rpm
            rates.append(speed_rpm_to_logical_rate(self.c, axis, rpm)); targets.append(q)
        if self.last_send:d.timings.add('speed_command_interval',(now-self.last_send)*1000)
        self.diagnostics = dict(axes=tuple(details),mailbox_age_ms=mailbox_age*1000,mailbox_lead_ms=lead_s*1000)
        self.last_send = d.last_send = now
        d.sent_target = d.stream_target = d.manual_target = tuple(targets)
        d.stream_velocity = tuple(rates)
        d.hold_target = None; d.hold_pending = False
        d.braking = d.soft_hold = False
        d.timings.add('native_goal_age_at_send', (time.monotonic()-stamp)*1000)
        d.timings.add('native_offer_to_write', (time.monotonic()-offered_at)*1000)

    def restore(self):
        d = self.driver
        if not any(self.engaged): return True
        self.stop()
        if d.armed:
            if (not d.reference_valid or any(d.feedback_issues)
                    or time.monotonic()-d.feedback_time >= self.c['reference_max_age_s']):
                return False
            self.sample_velocity()
            if max(abs(v) for v in self.velocity) > 8.: return False
        # Switch only after stopping. Preload a local position reference in speed mode;
        # this packet sets the next mode's target and does not command speed.
        for axis, mid in enumerate(self.c['motor_ids']):
            if not self.engaged[axis]: continue
            if d.armed: self.position(axis, d.angles[axis])
            d._write(select_mode(mid, self.c['position_mode']))
            if d.armed: self.position(axis, d.angles[axis])
            self.engaged[axis] = False
            d._write(set_acceleration(mid, self.c['motor_acceleration']))
            d.programmed_rpm[axis] = None
        d.stream_target = d.manual_target = d.sent_target = tuple(d.angles)
        d.stream_velocity = (0., 0.)
        self.accepted_aim = self.aim_candidate = None
        self.previous_pose = [None, None]; self.velocity = [0., 0.]
        self.gap_held = [False, False]; self.centered = [False, False]
        d.manual_ramp.reset(); d.tracking_ramp.reset(); d.hold_pending = False
        if d.armed:
            d._sync_driver_speed()
            if not any(d.joystick): d._latch_hold(d.sent_target)
        return True
