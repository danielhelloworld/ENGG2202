"""F32C mode 0 backend. No position commands, zero before enable/disable.

Angles remain observations/references for the camera's outer servo, never UART
position waypoints. One worker owns feedback, speed commands and all state.
"""


def velocity_limit(rate, angle, bounds, accel, latency, margin):
    lo,hi=bounds
    distance=hi-angle-margin if rate>0 else angle-lo-margin
    if not math.isfinite(distance):return rate
    if distance<=0:return 0.
    # Reserve travel during feedback/transport delay, then braking distance.
    speed=max(0.,math.sqrt((accel*latency)**2+2*accel*distance)-accel*latency)
    return math.copysign(min(abs(rate),speed),rate)


def moving_integral(value,error,feedforward,previous,dt,axis,c,allowed=True):
    # Correct sustained moving-target lag only. A stop, reversal, rapid slowdown,
    # coast, geometry relay, stale observation or rate saturation clears history.
    if (not allowed or abs(feedforward)<c['velocity_tracking_i_min_speed']
            or feedforward*previous<=0 or abs(feedforward)<.6*abs(previous)):
        return 0.
    if error*feedforward<0:return value*max(0.,1.-dt/.08)
    cap=c['velocity_tracking_i_cap'][axis]
    return clip(value+clip(error,-c['velocity_tracking_i_error_cap'],c['velocity_tracking_i_error_cap'])*c['velocity_tracking_ki'][axis]*dt,-cap,cap)


class VelocityMotorService(VisualMotorService):
    def __init__(self,c,transport_factory=None):
        super().__init__(c,transport_factory)
        self.velocity_message=None
        self.speed_command=[0.,0.];self.speed_rpm=[0,0]
        self.speed_last_write=[float('-inf')]*2
        self.speed_last_tick=None;self.speed_previous_pose=[None,None]
        self.speed_measured=[0.,0.];self.speed_integral=[0.,0.]
        ramp_config=dict(c,manual_response_s=c['velocity_response_s'],
                         manual_jerk_deg_s3=c['velocity_jerk_deg_s3'])
        self.speed_ramp=ManualRateRamp(ramp_config)
        self.tracking_stop_ramp=ManualRateRamp(dict(ramp_config,manual_response_s=c['velocity_tracking_brake_response_s']))
        self.velocity_hold=None;self.velocity_home_goal=None
        self.velocity_context=None;self.velocity_settled=None
        self.velocity_move_speed=None
        self.tracking_previous_ff=[0.,0.]
        self.tracking_brake_until=[0.,0.]
        self.home_direction_samples=[None,None];self.home_direction_checked=[False,False]

    def _write_impl(self,data):
        if (data[0]!=0x7A or data[-1]!=0x7B or bcc_xor(data[:-2])!=data[-2]
                or data[1] not in self.c['motor_ids']):raise ValueError('Invalid velocity bus frame')
        cmd=data[2]
        if cmd in (2,3,8,9,10):raise RuntimeError('Position/zero/Flash command forbidden in velocity backend')
        if cmd==FUNCTION_MODE and int.from_bytes(data[3:5],'big')!=MODE_SPEED:
            raise RuntimeError('Velocity backend requires mode 0')
        nonzero=cmd==FUNCTION_SPEED and int.from_bytes(data[3:5],'big',signed=True)!=0
        if nonzero and abs(int.from_bytes(data[3:5],'big',signed=True))>self.c['motor_rpm']:
            raise ValueError('RPM exceeds configured motor ceiling')
        if nonzero or cmd==FUNCTION_ENABLE:
            self._heartbeat_check();self._require_reference(fresh=False)
            if not self.arm_requested or any(self.feedback_issues) or time.monotonic()-self.feedback_time>self.c['reference_max_age_s']:
                raise FeedbackUnavailable('Fresh dual-axis feedback required for velocity drive')
        time.sleep(.002)
        # Recheck cancellation after yielding; zero/disable always bypass it.
        if nonzero or cmd==FUNCTION_ENABLE:self._heartbeat_check()
        if self.serial.write(data)!=len(data):raise RuntimeError('Incomplete velocity UART write')

    def _zero_speed(self):
        errors=[]
        if self.serial is not None:
            for mid in self.c['motor_ids']:
                try:self._write(set_speed(mid,0))
                except Exception as exc:errors.append(str(exc))
        self.speed_command=[0.,0.];self.speed_rpm=[0,0]
        self.speed_ramp.reset();self.speed_integral=[0.,0.]
        self.tracking_previous_ff=[0.,0.]
        self.tracking_brake_until=[0.,0.]
        return errors

    def _disable_all(self):
        errors=self._zero_speed()
        return errors+super()._disable_all()

    def _initialize(self):
        angles=super()._initialize()  # Saved mechanical reference; disabled reads only.
        for mid in self.c['motor_ids']:
            self._write(select_mode(mid,MODE_SPEED))
            self._write(set_acceleration(mid,self.c['velocity_driver_accel']))
        self._zero_speed()
        return angles

    def _arm_current(self):
        self._heartbeat_check();self._measure()
        if any(self.feedback_issues):raise FeedbackUnavailable('ARM needs both angles')
        for mid in self.c['motor_ids']:
            self._write(select_mode(mid,MODE_SPEED))
            self._write(set_acceleration(mid,self.c['velocity_driver_accel']))
        self._zero_speed()
        for mid in self.c['motor_ids']:self._write(enable(mid))
        self.velocity_hold=self.hold_target=tuple(self.angles)
        self.speed_last_tick=time.monotonic();self.speed_previous_pose=[None,None]
        self.speed_measured=[0.,0.];self.velocity_context=None
        if self.home_requested:self._velocity_begin_home()
        else:
            self.outside_hold_origin=(self.angles if any(not lo<=q<=hi for q,(lo,hi) in zip(self.angles,self.c['limits'])) else None)
        return tuple(self.angles)

    def _fixed_hold_active(self):
        return self.armed and self.velocity_hold is not None and self.response_context!='TRACK' and not self.home_requested and not any(self.joystick)

    def offer_tracking_goal(self,goal,stamp,active,velocity=(0.,0.),extrapolate=True):
        if goal is not None and (len(goal)!=2 or len(velocity)!=2 or len(active)!=2
                                or not math.isfinite(stamp) or any(not math.isfinite(v) for v in (*goal,*velocity))):
            raise ValueError('Invalid velocity goal observation')
        with self.lock:self.velocity_message=None if goal is None else (tuple(goal),stamp,tuple(velocity),bool(extrapolate))

    def _velocity_begin_home(self):
        self.velocity_home_goal=(self.angles[0]-signed_mechanical(self.angles[0]) if self.c['continuous_yaw'] else 0.,0.)
        self.recovery_origin=tuple(self.angles);self.recovery_active=True
        self.arm_homing=True;self.velocity_hold=self.hold_target=None
        self.velocity_settled=None;self.velocity_context=None
        self.home_direction_samples=[None,None];self.home_direction_checked=[False,False]

    def _check_home_direction(self,now):
        for axis,sample in enumerate(self.home_direction_samples):
            if sample is None or self.home_direction_checked[axis]:continue
            origin,started,direction=sample
            progress=(self.angles[axis]-origin)*direction
            if progress>=.3:self.home_direction_checked[axis]=True
            elif now-started>=.08 and progress<=-.8:
                self._zero_speed()
                raise MotionProtectionFault('SPEED_DIRECTION_MISMATCH: HOME axis %d moved %.2f deg against commanded direction' % (axis,-progress))

    def request_home(self):
        super().request_home()
        # Worker, not HTTP thread, captures the starting pose and begins home.

    def request_angle_move(self,yaw,pitch,speed,inertia_test=False,native_pitch=False,native_accel=3):
        if inertia_test or native_pitch:raise RuntimeError('Position-mode inertia experiments are unavailable in velocity mode')
        if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in (yaw,pitch,speed)):
            raise ValueError('Finite angle and speed required')
        if not 1<=speed<=min(self.c['manual_rate']):raise ValueError('Angle test speed outside range')
        if self.c['continuous_yaw']:yaw=self.angles[0]+signed_mechanical(yaw-self.angles[0])
        if any(not lo<=q<=hi for q,(lo,hi) in zip((yaw,pitch),self.c['limits'])):raise ValueError('Angle outside work limits')
        with self.lock:
            if not self.armed or self.fault or self.home_requested or self.outside_hold_origin is not None:
                raise RuntimeError('Enable and finish HOME first')
            if self.angle_move.active:raise RuntimeError('An angle move is already active')
            self.angle_move.__init__();self.angle_move.active=True;self.angle_move.state='MOVING'
            self.angle_move.profile='velocity_outer_loop';self.angle_move.goal=(yaw,pitch)
            self.angle_move.origin=tuple(self.angles);self.angle_move.accepted=self.angle_move.started=time.monotonic()
            self.velocity_move_speed=speed;self.velocity_hold=self.hold_target=None
            self.velocity_settled=None

    def _velocity_observe(self):
        for axis in (0,1):
            stamp=self.mechanical_feedback_times[axis];old=self.speed_previous_pose[axis]
            if stamp is None or (old and stamp<=old[1]):continue
            if old:
                dt=stamp-old[1]
                self.speed_measured[axis]=(self.speed_measured[axis]+dt/(.03+dt)*
                    (clip((self.angles[axis]-old[0])/dt,-self.c['motor_rpm']*9,self.c['motor_rpm']*9)-self.speed_measured[axis]) if .005<=dt<=.25 else 0.)
            self.speed_previous_pose[axis]=(self.angles[axis],stamp)

    def _velocity_tick(self):
        self._heartbeat_check();self._require_reference(fresh=False)
        now=time.monotonic();dt=min(.05,max(0.,now-(self.speed_last_tick or now)));self.speed_last_tick=now
        self._velocity_observe()
        if (any(self.feedback_issues) or self.motion_warning or now-self.feedback_time>self.c['reference_max_age_s']
                or now-self.command_time>self.c['control_pause_age_s']):
            self._zero_speed();self.motion_paused=True;return
        self.motion_paused=False
        if self.home_requested:self._check_home_direction(now)
        with self.lock:message=self.velocity_message;stick=tuple(self.joystick)
        if self.home_requested:
            if self.velocity_home_goal is None:self._velocity_begin_home()
            context='HOME';goal=self.velocity_home_goal;ff=(0.,0.);caps=self.c['home_rate']
            caps=tuple(cap if self.home_direction_checked[a] else min(cap,18.) for a,cap in enumerate(caps))
        elif self.angle_move.active:
            context='ANGLE';goal=self.angle_move.goal;ff=(0.,0.);caps=(self.velocity_move_speed,)*2
        elif self.response_context=='TRACK' and message and 0<=now-message[1]<=self.c['vision_max_age']:
            context='TRACK';goal,stamp,ff,extrapolate=message
            goal=tuple(q+(pursuit_shift(v,now-stamp,a,self.c) if extrapolate else 0.) for a,(q,v) in enumerate(zip(goal,ff)))
            goal=(goal[0],clip(goal[1],max(-90.,self.c['limits'][1][0]),self.c['limits'][1][1]))
            caps=self.c['velocity_tracking_rate']
        elif self.response_context=='MANUAL' and any(stick):
            context='MANUAL';goal=None;ff=(0.,0.);caps=self.c['manual_rate']
        else:
            context='HOLD';ff=(0.,0.);caps=self.c['hold_return_rate']
            if self.velocity_hold is None:
                # Decelerate first, then capture one landing pose, never chase
                # successive feedback positions while holding against a hand.
                if max(abs(v) for v in self.speed_command)>1 or max(abs(v) for v in self.speed_measured)>3:
                    context='BRAKE';goal=None
                else:self.velocity_hold=tuple(self.angles);goal=self.velocity_hold
            else:goal=self.velocity_hold
        if context!=self.velocity_context:
            self.speed_integral=[0.,0.];self.velocity_context=context
        if context in ('TRACK','MANUAL','ANGLE','HOME'):self.velocity_hold=self.hold_target=None
        elif context=='HOLD':self.hold_target=self.velocity_hold
        target=[];sent=[]
        for a in (0,1):
            error=0. if goal is None else goal[a]-self.angles[a]
            if context=='MANUAL':wanted=stick[a]*caps[a]
            elif context=='BRAKE':wanted=0.
            else:
                kp=(self.c['velocity_tracking_kp'][a] if context=='TRACK' else
                    self.c['velocity_home_kp'][a] if context in ('HOME','ANGLE') else self.c['velocity_hold_kp'][a])
                if context=='TRACK':
                    previous_ff=self.tracking_previous_ff[a]
                    if abs(previous_ff)>=self.c['velocity_tracking_i_min_speed'] and (ff[a]*previous_ff<=0 or abs(ff[a])<.6*abs(previous_ff)):
                        self.tracking_brake_until[a]=now+self.c['velocity_tracking_brake_s']
                    uncorrected=ff[a]+kp*error-self.c['velocity_damping']*(self.speed_measured[a]-ff[a])
                    distance=(self.c['limits'][a][1]-self.angles[a] if ff[a]>0 else self.angles[a]-self.c['limits'][a][0])
                    allowed=(extrapolate and now-stamp<=.2 and distance>2.
                             and now>=self.tracking_brake_until[a]
                             and abs(uncorrected)<caps[a]-self.c['velocity_tracking_i_cap'][a])
                    self.speed_integral[a]=moving_integral(self.speed_integral[a],error,ff[a],self.tracking_previous_ff[a],dt,a,self.c,allowed)
                if context=='HOLD' and abs(error)>self.c['velocity_hold_deadzone_deg']:
                    if abs(kp*error)<caps[a]:self.speed_integral[a]=clip(self.speed_integral[a]+error*dt*self.c['velocity_hold_ki'][a],-18.,18.)
                if abs(error)<=self.c['velocity_hold_deadzone_deg'] and abs(ff[a])<1.:
                    wanted=0.;self.speed_integral[a]=0.
                else:
                    damping=self.c['velocity_home_damping'] if context in ('HOME','ANGLE') else self.c['velocity_damping']
                    if context=='TRACK' and now<self.tracking_brake_until[a]:damping=self.c['velocity_tracking_stop_damping']
                    wanted=ff[a]+kp*error-damping*(self.speed_measured[a]-ff[a])+self.speed_integral[a]
                if context in ('HOME','ANGLE') and abs(self.speed_measured[a])<3.:
                    if abs(error)<=self.c['home_tolerance_deg']:wanted=0.
                    elif abs(wanted)<3.:wanted=math.copysign(6.,error)  # One RPM minimum above arrival tolerance.
            wanted=clip(wanted,-caps[a],caps[a])
            self.tracking_previous_ff[a]=ff[a] if context=='TRACK' else 0.
            bounds=self.c['limits'][a]
            if context=='HOME' and self.recovery_origin is not None:
                origin=self.recovery_origin[a];destination=goal[a]
                bounds=(min(bounds[0],origin,destination),max(bounds[1],origin,destination))
            # Out-of-range ARM allows inward speed only.
            bounded=velocity_limit(wanted,self.angles[a],bounds,self.c['velocity_accel_deg_s2'][a],
                                   max(0.,now-self.mechanical_feedback_times[a])+self.c['velocity_brake_latency_s'],
                                   self.c['velocity_limit_margin_deg'])
            ramp=self.tracking_stop_ramp if context=='TRACK' and now<self.tracking_brake_until[a] else self.speed_ramp
            if ramp is self.tracking_stop_ramp:ramp.acceleration[a]=self.speed_ramp.acceleration[a]
            v=ramp.advance(a,self.speed_command[a],bounded,self.c['velocity_accel_deg_s2'][a],dt,caps[a])
            if ramp is self.tracking_stop_ramp:self.speed_ramp.acceleration[a]=ramp.acceleration[a]
            v=velocity_limit(v,self.angles[a],bounds,self.c['velocity_accel_deg_s2'][a],
                             max(0.,now-self.mechanical_feedback_times[a])+self.c['velocity_brake_latency_s'],self.c['velocity_limit_margin_deg'])
            rpm=int(math.copysign(math.floor(abs(v)/6.+.5),v))
            # Rounding must not round the speed UP through a limit brake cap.
            safe=velocity_limit(rpm*6.,self.angles[a],bounds,self.c['velocity_accel_deg_s2'][a],
                                max(0.,now-self.mechanical_feedback_times[a])+self.c['velocity_brake_latency_s'],self.c['velocity_limit_margin_deg'])
            if abs(safe)<abs(rpm*6.)-1e-7:rpm=int(math.copysign(math.floor(abs(safe)/6.),safe))
            target.append(v);sent.append(rpm)
        self.speed_command=target;self.stream_velocity=tuple(r*6 for r in sent)
        self.sent_target=tuple(goal) if goal is not None else tuple(self.angles)
        self.stream_target=self.manual_target=self.sent_target
        for a,mid in enumerate(self.c['motor_ids']):
            if sent[a]!=self.speed_rpm[a] or now-self.speed_last_write[a]>=self.c['velocity_refresh_s']:
                raw_rpm=int(sent[a]*self.c['velocity_command_signs'][a]/self.c['motor_signs'][a])
                self._write(set_speed(mid,raw_rpm))
                self.speed_rpm[a]=sent[a];self.speed_last_write[a]=time.monotonic()
                if context=='HOME' and sent[a] and self.home_direction_samples[a] is None:
                    self.home_direction_samples[a]=(self.angles[a],time.monotonic(),1 if sent[a]>0 else -1)
        if context in ('HOME','ANGLE'):
            arrived=all(abs(q-p)<=self.c['home_tolerance_deg'] for q,p in zip(goal,self.angles)) and max(abs(v) for v in self.speed_measured)<3 and not any(sent)
            self.velocity_settled=(now if self.velocity_settled is None else self.velocity_settled) if arrived else None
            if self.velocity_settled is not None and now-self.velocity_settled>=self.c['home_settle_s']:
                self.velocity_hold=self.hold_target=tuple(goal)
                if context=='HOME':
                    self.home_requested=self.recovery_active=self.arm_homing=False
                    self.velocity_home_goal=None;self.recovery_origin=None;self.outside_hold_origin=None
                    self.startup_completed=self.home_holding=True;self.phase='HOME_HOLD'
                else:self.angle_move.active=False;self.angle_move.state='COMPLETE';self.angle_move.completed=now
        elif context=='HOLD':self._advance_fixed_hold_guard(now)
        self.last_send=now

    def _advance_fixed_hold_guard(self,now):
        # Reuse fixed-reference blockage observation without position writes.
        for a,(q,p) in enumerate(zip(self.velocity_hold,self.angles)):
            error=abs(q-p)
            if error<=self.c['hold_stall_error_deg']:self.hold_error_since[a]=self.hold_error_best[a]=None
            elif self.hold_error_best[a] is None or error<self.hold_error_best[a]-self.c['motion_progress_deg']:
                self.hold_error_since[a]=now;self.hold_error_best[a]=error
            elif now-self.hold_error_since[a]>=self.c['hold_block_timeout_s']:raise RuntimeError('SUSPECTED_BLOCK: velocity hold made no return progress')

    def _control_loop(self):
        while not self.stop_event.wait(self.c['motor_loop_period_s']):
            if self._recheck_requested():return
            try:
                self._measure()
                if self.armed and not self.arm_requested:self._disable_now();self.phase='DISABLED'
                if self.arm_requested and not self.armed:
                    self.manual_target=self.sent_target=self._arm_current();self.armed=True;self.phase='ARMED'
                if not self.armed:continue
                if self.angle_cancel:
                    self._cancel_angle('Cancelled');self.velocity_hold=None
                self._velocity_tick()
                if self.home_requested:self.phase='RETURNING'
                elif self.velocity_context=='HOLD':self.phase='HOME_HOLD' if self.home_holding else 'ARMED'
                else:self.phase='VELOCITY_'+self.velocity_context
            except UserDisarm:
                self._disable_now();self.phase='DISABLED'

    def _disable_now(self):
        super()._disable_now()
        self.velocity_hold=self.velocity_home_goal=self.velocity_message=None
        self.velocity_context=None;self.velocity_settled=None

    def snapshot(self):
        result=super().snapshot()
        result.update(drive_control='velocity',position_mode=MODE_SPEED,native_tracking_enabled=False,
                      velocity_command_deg_s=tuple(self.speed_command),velocity_sent_rpm=tuple(self.speed_rpm),
                      velocity_raw_sent_rpm=tuple(int(v*s/m) for v,s,m in zip(self.speed_rpm,self.c['velocity_command_signs'],self.c['motor_signs'])),
                      velocity_integral_deg_s=tuple(self.speed_integral),
                      velocity_command_signs=self.c['velocity_command_signs'],
                      home_direction_checked=tuple(self.home_direction_checked),
                      velocity_measured_deg_s=tuple(self.speed_measured),velocity_control_state=self.velocity_context,
                      velocity_tracking_rate=self.c['velocity_tracking_rate'],velocity_accel_deg_s2=self.c['velocity_accel_deg_s2'])
        return result
