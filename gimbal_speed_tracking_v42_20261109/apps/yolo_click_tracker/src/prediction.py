"""Short-horizon image-motion estimate compensated with measured gimbal pose."""


class MotionPredictor:
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self):
        self.previous = None
        self.velocity = (0., 0.)
        self.samples = 0

    @property
    def ready(self):
        return self.samples >= self.c["prediction_min_samples"]

    @property
    def motion_velocity(self):
        return tuple(v if abs(v) >= self.c["prediction_min_velocity"] else 0. for v in self.velocity)

    def observe(self, error, pose, stamp, score=1.):
        if (not self.c["prediction_enabled"] or pose is None or len(pose) != 2
                or not math.isfinite(score) or score < .55 or not all(math.isfinite(v) for v in (*error, *pose, stamp))):
            self.reset()
            return
        current = (tuple(error), tuple(pose), stamp)
        previous = self.previous
        self.previous = current
        if previous is None:
            self.samples = 1
            return
        before, before_pose, before_time = previous
        dt = stamp-before_time
        if not .02 <= dt <= self.c["prediction_max_gap_s"]:
            self.velocity, self.samples = (0., 0.), 1
            return
        # error directions: right/up. input signs map them to mechanical angles.
        displacement = tuple(e-b+s*(p-bp)/f for e,b,s,p,bp,f in zip(
            error, before, self.c["joystick_signs"], pose, before_pose, self.c["camera_fov"]))
        if max(abs(d) for d in displacement) > self.c["prediction_jump_gate"]:
            self.velocity, self.samples = (0., 0.), 1
            return
        alpha = 1-(1-self.c["prediction_velocity_alpha"])**(dt/.1)
        velocity = []
        for delta, old in zip(displacement, self.velocity):
            measured = clip(delta/dt, -self.c["prediction_max_velocity"], self.c["prediction_max_velocity"])
            if measured*old < 0:
                old = 0.  # Do not extrapolate the old direction after a reversal.
            if abs(measured) < self.c["prediction_min_velocity"]/2:
                old *= .3
            v = old+alpha*(measured-old)
            velocity.append(v)
        self.velocity = tuple(velocity)
        self.samples += 1

    def lead(self, error, latency=0.):
        if not self.ready:
            return tuple(error)
        horizon = self.c["prediction_lead_s"]+clip(latency, 0., self.c["prediction_latency_cap_s"])
        limit = self.c["prediction_max_shift"]
        return tuple(clip(e+clip(v*horizon, -limit, limit), -.5, .5)
                     for e,v in zip(error, self.motion_velocity))


class FastMotionPredictor(MotionPredictor):
    """Speed-mode estimator: preserve quiet response, accelerate consistent fast observations."""
    def reset(self):
        super().reset()
        self.fast_counts=[0,0]
        self.last_rates=[0.,0.]
        self.rate_limit_hits=[0,0]

    def observe(self,error,pose,stamp,score=1.):
        if (not self.c['prediction_enabled'] or pose is None or len(pose)!=2
                or not math.isfinite(score) or score<.55 or not all(math.isfinite(v) for v in (*error,*pose,stamp))):
            self.reset();return
        current=(tuple(error),tuple(pose),stamp)
        previous=self.previous;self.previous=current
        if previous is None:self.samples=1;return
        before,before_pose,before_time=previous
        dt=stamp-before_time
        if not .008<=dt<=self.c['prediction_max_gap_s']:
            self.reset();self.previous=current;self.samples=1;return
        displacement=tuple(e-b+sign*(q-old)/fov for e,b,sign,q,old,fov in zip(
            error,before,self.c['joystick_signs'],pose,before_pose,self.c['camera_fov']))
        if max(abs(v) for v in displacement)>self.c['prediction_jump_gate']:
            self.reset();self.previous=current;self.samples=1;return
        result=[]
        for axis,(delta,old,fov) in enumerate(zip(displacement,self.velocity,self.c['camera_fov'])):
            raw=delta/dt
            cap=self.c['speed_prediction_rate_deg_s'][axis]/fov
            measured=clip(raw,-cap,cap)
            if abs(raw)>cap:self.rate_limit_hits[axis]+=1
            was_fast=self.fast_counts[axis]>=2
            fast=abs(measured)*fov>=self.c['speed_prediction_fast_deg_s'][axis]
            self.fast_counts[axis]=(min(8,self.fast_counts[axis]+1)
                if fast and measured*self.last_rates[axis]>0 else 1 if fast else 0)
            self.last_rates[axis]=measured
            slowing=abs(old)*fov>=self.c['speed_prediction_fast_deg_s'][axis] and abs(measured)<abs(old)
            reference_alpha=(self.c['speed_prediction_fast_alpha'] if self.fast_counts[axis]>=2 or slowing
                             else self.c['prediction_velocity_alpha'])
            alpha=1-(1-reference_alpha)**(dt/.1)
            if measured*old<0:old=0.
            if abs(measured)<self.c['prediction_min_velocity']/2:old*=0. if was_fast else .3
            result.append(old+alpha*(measured-old))
        self.velocity=tuple(result);self.samples+=1

    def lead(self,error,latency=0.):
        if not self.ready:return tuple(error)
        horizon=self.c['prediction_lead_s']+clip(latency,0.,self.c['prediction_latency_cap_s'])
        return tuple(clip(e+clip(v*horizon,-limit,limit),-.5,.5)
            for e,v,limit in zip(error,self.motion_velocity,
                (self.c['speed_prediction_max_shift'] if n>=2 else self.c['prediction_max_shift'] for n in self.fast_counts)))

    def diagnostics(self):
        return dict(samples=self.samples,ready=self.ready,velocity_deg_s=tuple(v*f for v,f in zip(self.motion_velocity,self.c['camera_fov'])),
                    fast_axes=tuple(n>=2 for n in self.fast_counts),rate_limit_hits=tuple(self.rate_limit_hits),
                    rate_limit_deg_s=self.c['speed_prediction_rate_deg_s'])
