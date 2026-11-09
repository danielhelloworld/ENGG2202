"""Independent wide-motion controller extension; appended to the frozen v41 bundle."""

WIDE_CONFIG={
    'app_version':'wide-motion-v42',
    'wide_error_exit_deg':(4.,3.), 'wide_error_enter_deg':(12.,9.),
    'wide_catchup_deg_s':(100.,70.), 'wide_gain':(5.4,5.6),
    'wide_innovation_band_deg':(.8,.6), 'wide_innovation_gain':.30,
    'wide_coast_s':.35, 'wide_coast_max_deg':(24.,8.),
    'wide_reacquire_s':.9, 'wide_reacquire_gate_deg':(12.,8.),
    'wide_reacquire_score':.65, 'wide_lock_release_s':2.,
    'wide_diagnostic_period_s':1.,
}
CONFIG.update(WIDE_CONFIG)

_previous_validate_visual_config=validate_visual_config
_previous_speed_tracking_request=speed_tracking_request
_PreviousClickTracker=ClickTracker
_PreviousStabilizer=SpeedTrackingStabilizer
_PreviousSpeedStream=SpeedTrackingStream
_PreviousVisualMotorService=VisualMotorService
_PreviousTrackingHub=TrackingHub


def validate_visual_config(c):
    _previous_validate_visual_config(c)
    for key in ('wide_error_exit_deg','wide_error_enter_deg','wide_catchup_deg_s','wide_gain',
                'wide_innovation_band_deg','wide_coast_max_deg','wide_reacquire_gate_deg'):
        if len(c[key])!=2 or any(not math.isfinite(v) or v<=0 for v in c[key]):
            raise ValueError('Invalid '+key)
    if any(lo>=hi for lo,hi in zip(c['wide_error_exit_deg'],c['wide_error_enter_deg'])):
        raise ValueError('Wide motion error band must increase')
    if any(v>c['motor_rpm']*6 for v in c['wide_catchup_deg_s']):
        raise ValueError('Wide catch-up exceeds the motor rate ceiling')
    if (not .1<=c['wide_innovation_gain']<=1. or not .15<=c['wide_coast_s']<=.5
            or not c['wide_coast_s']<=c['wide_reacquire_s']<=1.2
            or not c['wide_reacquire_s']<c['wide_lock_release_s']<=3.
            or not .6<=c['wide_reacquire_score']<=.9
            or not .5<=c['wide_diagnostic_period_s']<=5.):
        raise ValueError('Invalid wide motion observation/loss bounds')
    if c['wide_coast_max_deg'][0]>30. or c['wide_coast_max_deg'][1]>10.:
        raise ValueError('Blind continuation displacement too large')


def wide_error_weight(c,axis,error):
    lo,hi=c['wide_error_exit_deg'][axis],c['wide_error_enter_deg'][axis]
    return clip((abs(error)-lo)/(hi-lo),0.,1.)


def speed_tracking_request(c,axis,error,measured,feedforward,moving,feedback_age=0.):
    """A continuous large-error capture law; keep the previous fine-tracking law."""
    fine=_previous_speed_tracking_request(c,axis,error,measured,feedforward,moving,feedback_age)
    weight=wide_error_weight(c,axis,error)
    if weight<=0.:return fine
    relative=measured-feedforward
    brake=c['speed_tracking_brake_deg_s2'][axis]
    reaction=c['speed_tracking_brake_latency_s']+clip(feedback_age,0.,.1)
    safe=math.sqrt((brake*reaction)**2+2*brake*abs(error))-brake*reaction
    correction=c['wide_gain'][axis]*error-c['speed_tracking_kd'][axis]*relative
    correction=clip(correction,-safe,safe)
    # The safe closing-speed curve itself provides braking. Avoid toggling a
    # large correction abruptly to zero at its measured-velocity boundary.
    if abs(feedforward)>=c['speed_tracking_motion_band_deg_s'][1] and correction*feedforward>0:
        correction=math.copysign(min(abs(correction),c['wide_catchup_deg_s'][axis]),correction)
    requested=feedforward+correction
    if not moving and abs(feedforward)<1.:requested=0.
    cap=native_tracking_rates(c)[axis]
    return clip(fine+(requested-fine)*weight,-cap,cap)


class WideMotionStabilizer(_PreviousStabilizer):
    def offer(self,goal,feedforward,actual,stamp):
        previous,old_rates,old_stamp=self.goal,self.feedforward,self.stamp
        filtered,rates=super().offer(goal,feedforward,actual,stamp)
        if previous is None or stamp==old_stamp or not 0<stamp-old_stamp<=.2:
            return filtered,rates
        dt=stamp-old_stamp
        values=list(filtered)
        for axis,(q,ff,pose) in enumerate(zip(goal,feedforward,actual)):
            weight=wide_error_weight(self.c,axis,q-pose)
            if not weight:continue
            predicted=previous[axis]+old_rates[axis]*dt
            innovation=q-predicted
            band=self.c['wide_innovation_band_deg'][axis]
            alpha=self.c['wide_innovation_gain']+(1-self.c['wide_innovation_gain'])*clip(abs(innovation)/band-1.,0.,1.)
            corrected=predicted+alpha*innovation
            values[axis]=filtered[axis]+weight*(corrected-filtered[axis])
        self.goal=tuple(values)
        return self.goal,self.feedforward


class WideMotionClickTracker(_PreviousClickTracker):
    def __init__(self,c):
        super().__init__(c)
        self._reset_wide()

    def _reset_wide(self):
        self.last_box_area=None;self.reacquire_candidate=None
        self.wide_reacquired=0;self.loss_message='';self.last_measurement_score=0.

    def clear(self,arm_c=False):
        super().clear(arm_c)
        self._reset_wide()

    def _candidate_reacquire(self,tracks,stamp,pose):
        if self.last_seen is None or pose is None or self.last_tracked_goal is None:return None
        age=stamp-self.last_seen
        if not 0<age<=self.c['wide_reacquire_s']:return None
        if self.overhead.active:return self.overhead.reacquire(tracks,self.selected,pose,stamp)
        predicted=tuple(q+clip(v*age,-cap,cap) for q,v,cap in zip(
            self.last_tracked_goal,self.last_tracked_velocity,self.c['wide_coast_max_deg']))
        candidates=[]
        for tr in tracks:
            score=tr.get('score',0.)
            if tr['key'][0]!=self.selected[0] or not math.isfinite(score) or score<self.c['wide_reacquire_score']:continue
            box=tr.get('box',(0,0,0,0));area=max(0.,box[2]*box[3])
            if self.last_box_area and not .25<=area/self.last_box_area<=4.:continue
            ray=(tr['cx']-.5,.5-tr['cy'])
            bearing=tuple(q+sign*e*fov for q,sign,e,fov in zip(pose,self.c['joystick_signs'],ray,self.c['camera_fov']))
            if all(abs(a-b)<=gate for a,b,gate in zip(bearing,predicted,self.c['wide_reacquire_gate_deg'])):
                candidates.append(tr)
        if len(candidates)!=1:
            self.reacquire_candidate=None;return None
        tr=candidates[0];key=tuple(tr['key']);previous=self.reacquire_candidate
        self.reacquire_candidate=(key,stamp)
        if previous is not None and previous[0]==key and 0<stamp-previous[1]<=.15:return tr
        return None

    def _continue_or_hold(self,now):
        age=max(0.,now-self.last_seen) if self.last_seen is not None else float('inf')
        if age>=self.c['wide_lock_release_s']:
            self.clear();self.state='LOST'
            self.loss_message='TARGET_LOST: lock released; hold position; select target again (no automatic HOME)'
            return self.vector
        duration=self.c['wide_coast_s']
        reliable=(self.last_measurement_score>=.55 and self.last_tracked_goal is not None
                  and self.predictor.ready and any(abs(v)>=4. for v in self.last_tracked_velocity))
        if reliable and age<duration:
            fade=1.-age/duration
            integral=age-age*age/(2*duration)
            values=[];rates=[]
            for axis,(q,v,bound,limits) in enumerate(zip(self.last_tracked_goal,self.last_tracked_velocity,
                    self.c['wide_coast_max_deg'],self.c['limits'])):
                displacement=v*integral
                value=q+clip(displacement,-bound,bound)
                bounded=value if axis==0 and self.c['continuous_yaw'] else clip(value,*limits)
                values.append(bounded)
                rates.append(v*fade if abs(displacement)<bound and bounded==value else 0.)
            self.aim_goal=tuple(values)
            self.aim_velocity=tuple(rates)
            self.native_active_axes=tuple(abs(v)>0 for v in self.aim_velocity)
            self.vector=tuple(clip(v/r,-1.,1.) for v,r in zip(self.aim_velocity,self.c['max_rate']))
            norm=max(1.,math.hypot(*self.vector));self.vector=tuple(v/norm for v in self.vector)
            self.state='COAST';self.prediction_active=True
            self.loss_message='SHORT_LOSS: bounded decelerating continuation, waiting for visual confirmation'
        else:
            self.state='LOST';self.vector=(0.,0.);self.aim_goal=None
            self.aim_velocity=(0.,0.);self.native_active_axes=(False,False)
            self.prediction_active=False
            self.loss_message='TARGET_NOT_VISIBLE: stopped; waiting for same ID or unique confirmed candidate'
        return self.vector

    def drive(self,now,pose=None):
        if self.selected is not None and self.state in ('COAST','LOST'):
            return self._continue_or_hold(now)
        return super().drive(now,pose)

    def step(self,tracks,now,pose=None,latency=0.,current_pose=None):
        if self.selected is None:return super().step(tracks,now,pose,latency,current_pose)
        target=next((tr for tr in tracks if tuple(tr['key'])==self.selected),None)
        if target is None:
            target=self._candidate_reacquire(tracks,now,pose)
            if target is not None:
                self.selected=tuple(target['key']);self.wide_reacquired+=1
                self.reacquire_candidate=None
        if target is not None:
            result=super().step(tracks,now,pose,latency,current_pose)
            box=target.get('box',(0,0,0,0));self.last_box_area=max(0.,box[2]*box[3])
            self.last_measurement_score=target.get('score',0.)
            self.loss_message='';self.reacquire_candidate=None
            return result
        self.last_tick=now
        plan=self.overhead.missing(now,current_pose)
        if plan is not None:
            self.state='OVERHEAD';self.aim_goal=plan['goal'];self.aim_velocity=plan['velocity']
            self.native_active_axes=plan['active'];return None
        self._continue_or_hold(now)
        return None  # Loss never implicitly starts HOME in this independent branch.


class WideMotionSpeedStream(_PreviousSpeedStream):
    def advance(self):
        result=super().advance()
        for axis,detail in enumerate(self.diagnostics.get('axes',())):
            weight=wide_error_weight(self.c,axis,detail['error_deg'])
            detail['wide_error_weight']=weight
            detail['catchup_limit_deg_s']=self.c['speed_tracking_catchup_deg_s'][axis]+weight*(
                self.c['wide_catchup_deg_s'][axis]-self.c['speed_tracking_catchup_deg_s'][axis])
            detail['catchup_limited']=abs(detail['requested_deg_s']-detail['target_velocity_deg_s'])>=detail['catchup_limit_deg_s']-.1
        return result


class WideMotionMotorService(_PreviousVisualMotorService):
    def __init__(self,c,transport_factory=None):
        super().__init__(c,transport_factory)
        self.pause_counts={};self.pause_reason=None;self._last_pause_reason=None

    def _measure(self):
        try:return super()._measure()
        finally:
            reason=('FAULT' if self.fault else 'FEEDBACK_PAUSE' if any(self.feedback_issues)
                else 'MECHANICAL_LIMIT_PAUSE' if self.motion_warning
                else 'CONTROL_HEARTBEAT_PAUSE' if self.armed and time.monotonic()-self.command_time>self.c['control_pause_age_s']
                else None)
            if reason and reason!=self._last_pause_reason:self.pause_counts[reason]=self.pause_counts.get(reason,0)+1
            self.pause_reason=reason;self._last_pause_reason=reason

    def snapshot(self):
        result=super().snapshot()
        result['wide_pause_reason']=self.pause_reason
        result['wide_pause_counts']=dict(self.pause_counts)
        return result


class WideMotionHub(_PreviousTrackingHub):
    def __init__(self,driver,c,clock=time.monotonic):
        super().__init__(driver,c,clock)
        self._wide_last_log=None
        self._wide_loss_notice=None

    def tick(self):
        super().tick()
        if self.tracking.loss_message:
            self.notice=self.tracking.loss_message
            self._wide_loss_notice=self.notice
        elif self._wide_loss_notice is not None:
            if self.notice==self._wide_loss_notice:
                self.notice='TRACK_RESUMED: visual target confirmed' if self.tracking.state=='TRACK' else 'Ready for target selection'
            self._wide_loss_notice=None
        now=self.clock()
        if self._wide_last_log is None or now-self._wide_last_log>=self.c['wide_diagnostic_period_s']:
            motor=self.driver.snapshot();diagnostic=motor.get('speed_tracking_diagnostics',{})
            flags=[]
            if self.tracking.state in ('COAST','LOST'):flags.append('IDENTITY_OR_FOV_LOSS')
            if motor.get('wide_pause_reason'):flags.append(motor['wide_pause_reason'])
            if any(a.get('rpm_ceiling') for a in diagnostic.get('axes',())):flags.append('RPM_CEILING')
            planned=motor.get('planned_velocity_deg_s',(0.,0.))
            measured=motor.get('response',{}).get('measured_velocity_deg_s',(0.,0.))
            if any(abs(p)>36. and abs(v)<.35*abs(p) for p,v in zip(planned,measured)):
                flags.append('COMMAND_ACTUAL_SPEED_GAP')
            self.performance['wide_motion']=dict(flags=flags,visual_state=self.tracking.state,
                reacquired=self.tracking.wide_reacquired,pause_counts=motor.get('wide_pause_counts',{}))
            stages=self.driver.timings.snapshot() if hasattr(self.driver,'timings') else {}
            print('WIDE_V42 '+json.dumps(dict(t=round(now,3),state=self.tracking.state,
                selected=self.tracking.selected,actual_deg=motor.get('angles'),command_rpm=motor.get('tracking_command_rpm'),
                measured_deg_s=measured,flags=flags,pause_counts=motor.get('wide_pause_counts',{}),
                fps=self.performance.get('inference_fps_actual'),
                control_p95_ms=self.performance.get('capture_to_control_p95_ms'),
                uart_interval_ms=stages.get('uart_cycle_interval',{}).get('p95_ms'))),flush=True)
            self._wide_last_log=now


# Rebind only in this standalone module. Previous files remain byte-for-byte unchanged.
SpeedTrackingStabilizer=WideMotionStabilizer
SpeedTrackingStream=WideMotionSpeedStream
ClickTracker=WideMotionClickTracker
VisualMotorService=WideMotionMotorService
TrackingHub=WideMotionHub
WEB_HTML=WEB_HTML.replace('speed-track-v41','wide-motion-v42')
validate_visual_config(CONFIG)
