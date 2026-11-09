"""One-shot mechanical-angle experiment. All trajectory updates run on the UART worker."""


class AngleMove:
    def __init__(self):
        self.active = False
        self.state = "IDLE"
        self.goal = self.origin = None
        self.accepted = self.started = self.first_write = self.first_motion = self.completed = None
        self.duration = self.progress = 0.
        self.settled = None
        self.speed = None
        self.reason = None
        self.profile = 'software_quintic'

    def begin(self, origin, goal, speed, accepted, now, c):
        self.__init__()
        self.active = True
        self.state = "MOVING"
        self.origin, self.goal, self.speed = tuple(origin), tuple(goal), speed
        self.accepted, self.started = accepted, now
        # Quintic position interpolation has zero velocity and acceleration at
        # both ends; these are its exact derivative maxima, with a small margin.
        self.duration = max([.1]+[max(1.875*abs(b-a)/min(speed,cap),
                                       (5.774*abs(b-a)/accel)**.5,
                                       (60.*abs(b-a)/jerk)**(1./3.))
                                   for a,b,cap,accel,jerk in zip(origin,goal,c['manual_rate'],
                                                                c['manual_accel_deg_s2'],c['manual_jerk_deg_s3'])])

    def waypoint(self, dt, actual, lead):
        progress=min(self.duration,self.progress+dt)
        u=progress/self.duration
        blend=u*u*u*(10.+u*(-15.+6.*u))
        target=tuple(a+(b-a)*blend for a,b in zip(self.origin,self.goal))
        if any(abs(q-a)>cap for q,a,cap in zip(target,actual,lead)):
            return None  # Pause trajectory time until feedback catches up.
        self.progress=progress
        velocity=tuple((b-a)*30.*u*u*(1.-u)*(1.-u)/self.duration for a,b in zip(self.origin,self.goal))
        return target,velocity

    def observe(self, now, actual, velocity, tolerance, allow_timeout=True):
        if self.first_motion is None and max(abs(a-b) for a,b in zip(actual,self.origin))>=.3:
            self.first_motion=now
        if self.progress>=self.duration:
            self.state='SETTLING'
            at_goal=(all(abs(a-b)<=tolerance for a,b in zip(actual,self.goal))
                     and max(abs(v) for v in velocity)<3.)
            self.settled=(now if self.settled is None else self.settled) if at_goal else None
            if self.settled is not None and now-self.settled>=.25:
                self.active=False;self.state='COMPLETE';self.completed=now
        if allow_timeout and self.active and now-self.started>self.duration*3.+5.:
            raise RuntimeError('ANGLE_MOVE_TIMEOUT: target did not settle')

    def cancel(self, reason):
        if self.active:
            self.active=False;self.state='CANCELLED';self.reason=reason

    def snapshot(self, pending=None):
        latency=lambda t:None if t is None or self.accepted is None else (t-self.accepted)*1000.
        return dict(active=self.active or pending is not None,state='PENDING' if pending else self.state,profile=self.profile,
                    target_deg=pending[:2] if pending else self.goal,speed_deg_s=pending[2] if pending else self.speed,
                    planned_duration_s=self.duration,progress_s=self.progress,
                    queue_ms=latency(self.started),first_write_ms=latency(self.first_write),
                    first_motion_ms=latency(self.first_motion),completed_ms=latency(self.completed),reason=self.reason)
