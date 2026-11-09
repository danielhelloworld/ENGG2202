"""Short-horizon image-motion estimate compensated with measured gimbal pose."""


class MotionPredictor:
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self):
        self.previous = None
        self.velocity = (0., 0.)
        self.samples = 0
        self.previous_measured = (0., 0.)

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
            self.previous_measured = (0., 0.)
            return
        before, before_pose, before_time = previous
        dt = stamp-before_time
        if not .02 <= dt <= self.c["prediction_max_gap_s"]:
            self.velocity, self.samples = (0., 0.), 1
            self.previous_measured = (0., 0.)
            return
        # error directions: right/up. input signs map them to mechanical angles.
        displacement = tuple(e-b+s*(p-bp)/f for e,b,s,p,bp,f in zip(
            error, before, self.c["joystick_signs"], pose, before_pose, self.c["camera_fov"]))
        if max(abs(d) for d in displacement) > self.c["prediction_jump_gate"]:
            self.velocity, self.samples = (0., 0.), 1
            self.previous_measured = (0., 0.)
            return
        alpha = 1-(1-self.c["prediction_velocity_alpha"])**(dt/.1)
        velocity = []
        measurements = []
        for delta, old, before_rate in zip(displacement, self.velocity, self.previous_measured):
            measured = clip(delta/dt, -self.c["prediction_max_velocity"], self.c["prediction_max_velocity"])
            measurements.append(measured)
            fast = (measured*before_rate>0 and min(abs(measured),abs(before_rate))>=self.c['prediction_fast_min_velocity'])
            axis_alpha = max(alpha,dt/(self.c['prediction_fast_tau_s']+dt)) if fast else alpha
            if measured*old < 0:
                old = 0.  # Do not extrapolate the old direction after a reversal.
            if abs(measured) < self.c["prediction_min_velocity"]/2:
                old *= .3
            v = old+axis_alpha*(measured-old)
            velocity.append(v)
        self.velocity = tuple(velocity)
        self.previous_measured = tuple(measurements)
        self.samples += 1

    def lead(self, error, latency=0.):
        if not self.ready:
            return tuple(error)
        horizon = self.c["prediction_lead_s"]+clip(latency, 0., self.c["prediction_latency_cap_s"])
        limit = self.c["prediction_max_shift"]
        return tuple(clip(e+clip(v*horizon, -limit, limit), -.5, .5)
                     for e,v in zip(error, self.motion_velocity))
