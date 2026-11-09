"""Latest absolute visual aim, retargeted inside F32C's single-turn planner."""


def velocity_damped_goal(goal, measured_velocity, target_velocity, c):
    correction=clip((measured_velocity-target_velocity)*c['native_velocity_damping_s'],
                    -c['native_damping_max_deg'],c['native_damping_max_deg'])
    return goal-correction,correction


def pursuit_shift(velocity, age, axis, c):
    # Stale observations never receive an ever-growing position extrapolation.
    horizon=clip(age,0.,c['native_prediction_age_s'])+c['native_pursuit_lead_s']
    cap=c['native_pursuit_max_deg'][axis]
    return clip(velocity*horizon,-cap,cap)


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
        self.pursuit=[0.,0.]

    def offer(self, goal, stamp, active, velocity=(0.,0.), extrapolate=True):
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
        self.message=(goal,stamp,tuple(bool(v) for v in active),self.offered_at,tuple(velocity),bool(extrapolate))

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
        self.pursuit=[0.,0.]
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
        goal,stamp,active,offered_at,target_velocity,extrapolate=message
        self.gap_held=[False,False]
        if now-self.last_send>=self.c['native_target_period_s']:
            for axis,(q,actual,(lo,hi),moving) in enumerate(zip(goal,d.angles,self.c['limits'],active)):
                if axis==1 and self.c.get('overhead_tracking',False):lo=max(lo,-90.)
                relative_velocity=self.velocity[axis]-target_velocity[axis]
                braking=self.c['native_velocity_damping_s']>0 and abs(relative_velocity)>3.
                if not moving and self.centered[axis] and not braking:continue
                self.pursuit[axis]=pursuit_shift(target_velocity[axis],time.monotonic()-stamp,axis,self.c) if extrapolate else 0.
                q+=self.pursuit[axis]
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
        self.pursuit=[0.,0.]
        d.manual_ramp.reset();d.tracking_ramp.reset();d.hold_pending=False
        if d.armed and not any(d.joystick):d._latch_hold(d.sent_target)
        return True
