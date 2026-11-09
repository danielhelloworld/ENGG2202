"""Capture-time line-of-sight geometry and a bounded, non-inverting yaw relay."""
def sight_vector(error, pose, c):
    yaw=math.radians(pose[0]);elev=math.radians(pose[1]*c['joystick_signs'][1])
    cy,sy,ce,se=math.cos(yaw),math.sin(yaw),math.cos(elev),math.sin(elev)
    u=2*error[0]*math.tan(math.radians(c['camera_fov'][0]/2))
    v=2*error[1]*math.tan(math.radians(c['camera_fov'][1]/2))
    right=(-sy*c['joystick_signs'][0],cy*c['joystick_signs'][0],0.)
    forward=(ce*cy,ce*sy,se);up=(-se*cy,-se*sy,ce)
    vector=tuple(f+u*r+v*t for f,r,t in zip(forward,right,up))
    length=math.sqrt(sum(q*q for q in vector))
    return tuple(q/length for q in vector)


class OverheadPlanner:
    def __init__(self,c):
        self.c=c;self.reset()

    def reset(self):
        self.state='NORMAL';self.entry_yaw=self.turn_yaw=self.deadline=None
        self.candidate=None;self.previous=None
        self.velocity=(0.,0.);self.turn_count=0;self.elevation=None
        self.last_turn_sign=1.
        self.last_vector=self.vector_previous=None
        self.vector_velocity=(0.,0.,0.)
        self.reacquire_until=None;self.reacquire_candidate=None

    def waiting(self,now):
        return self.reacquire_until is not None and now<=self.reacquire_until

    def reacquire(self,tracks,selected,pose,stamp):
        if not self.waiting(stamp) or pose is None or self.last_vector is None:return None
        age=max(0.,stamp-self.vector_previous[1])
        predicted=tuple(q+v*min(age,.3) for q,v in zip(self.last_vector,self.vector_velocity))
        length=math.sqrt(sum(q*q for q in predicted))
        predicted=tuple(q/length for q in predicted) if length>1e-9 else self.last_vector
        candidates=[]
        for track in tracks:
            if track['key'][0]!=selected[0] or track.get('score',0.)<self.c['new_track_confidence']:continue
            vector=sight_vector((track['cx']-.5,.5-track['cy']),pose,self.c)
            angle=math.degrees(math.acos(clip(sum(a*b for a,b in zip(predicted,vector)),-1.,1.)))
            if angle<=10.:candidates.append(track)
        if len(candidates)!=1:
            self.reacquire_candidate=None;return None
        candidate=candidates[0];key=tuple(candidate['key']);previous=self.reacquire_candidate
        self.reacquire_candidate=(key,stamp)
        if previous is not None and previous[0]==key and 0<stamp-previous[1]<=.2:
            self.reacquire_candidate=None;return candidate
        return None

    @property
    def active(self):return self.state!='NORMAL'

    def _turn_plan(self,pose):
        return dict(goal=(self.turn_yaw,88./self.c['joystick_signs'][1]),
                    velocity=(0.,0.),active=(True,True),state=self.state)

    def missing(self,now,pose):
        if self.state!='TURN' or pose is None:return None
        if now>self.deadline:
            self.state='POLE';return None
        if abs(self.turn_yaw-pose[0])<=8.:
            self.state='POLE';self.entry_yaw=self.turn_yaw;self.candidate=None
            return None
        return self._turn_plan(pose)

    def observe(self,error,capture_pose,current_pose,stamp,score=1.):
        if (not self.c['overhead_tracking'] or not self.c['continuous_yaw']
                or capture_pose is None or current_pose is None):return None
        if self.state=='NORMAL':
            upper=max(capture_pose[1]*self.c['joystick_signs'][1],current_pose[1]*self.c['joystick_signs'][1])
            if upper+abs(error[1])*self.c['camera_fov'][1]*1.2<70.:return None
        vector=sight_vector(error,capture_pose,self.c)
        if self.vector_previous is not None and .01<stamp-self.vector_previous[1]<=.25:
            dt=stamp-self.vector_previous[1];alpha=dt/(.1+dt)
            self.vector_velocity=tuple(v+alpha*((q-old)/dt-v) for v,q,old in
                zip(self.vector_velocity,vector,self.vector_previous[0]))
        else:self.vector_velocity=(0.,0.,0.)
        self.last_vector=vector;self.vector_previous=(vector,stamp)
        az=math.degrees(math.atan2(vector[1],vector[0]))
        elev=math.degrees(math.atan2(vector[2],math.hypot(*vector[:2])))
        self.elevation=elev
        yaw=current_pose[0]+signed_mechanical(az-current_pose[0])
        pitch=clip(elev/self.c['joystick_signs'][1],-90.,self.c['limits'][1][1])
        camera_elev=capture_pose[1]*self.c['joystick_signs'][1]
        if self.state=='NORMAL':
            if max(elev,camera_elev)<70.:return None
            self.state='GEOMETRY'
        if self.state=='TURN':
            plan=self.missing(stamp,current_pose)
            if plan is not None:return plan
        crossed_between_frames=(camera_elev>=75. and elev>=70. and abs(signed_mechanical(az-current_pose[0]))>100.)
        if self.state=='GEOMETRY' and (elev>=84. or crossed_between_frames):
            self.state='POLE';self.entry_yaw=current_pose[0];self.candidate=None
        if self.state=='POLE':
            entry=math.radians(self.entry_yaw)
            behind=vector[0]*math.cos(entry)+vector[1]*math.sin(entry)
            delta=signed_mechanical(az-self.entry_yaw)
            crossing=behind < -math.sin(math.radians(2.)) and abs(delta)>100. and score>=.5
            if crossing:
                before=self.candidate
                self.candidate=stamp
                if before is not None and 0<stamp-before<=.2:
                    if abs(delta)>=170.:delta=180.*self.last_turn_sign
                    self.last_turn_sign=1. if delta>0 else -1.
                    self.turn_yaw=self.entry_yaw+delta
                    self.deadline=stamp+self.c['overhead_turn_timeout_s']
                    self.reacquire_until=self.deadline+.5
                    self.turn_count+=1;self.state='TURN';self.previous=None
                    return self._turn_plan(current_pose)
            else:self.candidate=None
            if elev<78.:
                self.state='GEOMETRY';self.previous=None
            else:
                return dict(goal=(self.entry_yaw,88./self.c['joystick_signs'][1]),
                            velocity=(0.,0.),active=(True,True),state=self.state)
        if self.state=='GEOMETRY' and elev<60. and camera_elev<65.:
            self.state='NORMAL';self.previous=None;self.velocity=(0.,0.);return None
        previous=self.previous
        self.previous=(yaw,pitch,stamp)
        if previous is not None and .01<stamp-previous[2]<=.25:
            dt=stamp-previous[2];alpha=dt/(.1+dt)
            measured=(signed_mechanical(yaw-previous[0])/dt,(pitch-previous[1])/dt)
            cap=self.c['motor_rpm']*6
            self.velocity=tuple(v+alpha*(clip(q,-cap,cap)-v) for v,q in zip(self.velocity,measured))
        else:self.velocity=(0.,0.)
        return dict(goal=(yaw,pitch),velocity=self.velocity,active=(True,True),state=self.state)

    def snapshot(self):
        return dict(state=self.state,turn_yaw_deg=self.turn_yaw,turn_count=self.turn_count,
                    deadline=self.deadline,elevation_deg=self.elevation,reacquire_until=self.reacquire_until)
