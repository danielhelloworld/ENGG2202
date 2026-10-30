"""Bounded software response adjustment from real position error; no torque API."""


class ManualRateRamp:
    """Critically damped velocity response, bounded acceleration and jerk.

    Substeps keep the ramp stable with varying UART/control loop intervals.
    Explicit hold, feedback loss and hard limit handling may reset immediately.
    """
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self, axis=None):
        if axis is None:
            self.acceleration = [0., 0.]
        else:
            self.acceleration[axis] = 0.

    def advance(self, axis, velocity, desired, accel, dt, cap):
        if dt <= 0:
            return velocity
        steps = max(1, int(math.ceil(dt/.005)))
        h, omega = dt/steps, 2./self.c["manual_response_s"]
        a, jerk_cap = self.acceleration[axis], self.c["manual_jerk_deg_s3"][axis]
        for _ in range(steps):
            jerk = clip(omega*omega*(desired-velocity)-2*omega*a, -jerk_cap, jerk_cap)
            a = clip(a+jerk*h, -accel, accel)
            proposed = velocity+a*h
            velocity = clip(proposed, -cap, cap)
            if velocity != proposed:
                a = 0.  # The speed ceiling takes priority over smoothing.
        self.acceleration[axis] = a
        return velocity


class ResponseTuner:
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self):
        self.previous = None
        self.velocity = self.error = self.error_square = (0., 0.)
        self.scale = (1., 1.)
        self.accel = tuple(clip(.85*a, lo, a) for a,lo in zip(self.c["command_accel_deg_s2"], self.c["adaptive_accel_min"]))
        self.correction = (0., 0.)
        self.status = ("IDLE", "IDLE")

    def update(self, now, actual, commanded, requested, enabled, healthy=True):
        if not enabled or not healthy or commanded is None:
            self.reset()
            return
        error = tuple(q-a for q,a in zip(commanded, actual))
        self.error = error
        previous, self.previous = self.previous, (now, tuple(actual))
        if previous is None:
            return
        dt = now-previous[0]
        if not .005 <= dt <= .25:
            self.velocity = self.correction = (0., 0.)
            return
        alpha = 1-math.exp(-dt/.12)
        velocity = tuple(v+alpha*((a-b)/dt-v) for v,a,b in zip(self.velocity, actual, previous[1]))
        scales, accels, corrections, states = [], [], [], []
        squares = []
        for axis,(e,request,speed,scale,accel,position,limits) in enumerate(zip(error,requested,velocity,
                self.scale,self.accel,actual,self.c["limits"])):
            lo,hi = self.c["adaptive_scale_bounds"]
            minimum,maximum = self.c["adaptive_accel_min"][axis], self.c["command_accel_deg_s2"][axis]
            target_scale, target_accel, correction, status = 1., .85*maximum, 0., "STABLE"
            direction = 1 if request >= 0 else -1
            progress, lag = speed*direction, e*direction
            near_limit = min(position-limits[0], limits[1]-position) < self.c["manual_lead_deg"]
            if not self.c["adaptive_response"]:
                target_accel, status = maximum, "FIXED"
            elif abs(request) <= .5 or near_limit:
                status = "HOLD" if abs(request) <= .5 else "LIMIT_APPROACH"
            elif lag < -self.c["adaptive_overshoot_deg"] or progress > abs(request)*1.3+1:
                target_scale, target_accel, status = lo, minimum, "DAMPING"
                correction = clip(e*self.c["adaptive_correction_kp"], -self.c["adaptive_correction_cap_deg_s"], self.c["adaptive_correction_cap_deg_s"])
            elif lag > self.c["adaptive_lag_deg"]:
                if progress >= self.c["adaptive_min_progress_deg_s"]:
                    target_scale, target_accel, status = hi, maximum, "LAG_COMPENSATING"
                    correction = clip(e*self.c["adaptive_correction_kp"], -self.c["adaptive_correction_cap_deg_s"], self.c["adaptive_correction_cap_deg_s"])
                else:
                    # Lack of progress is not permission to push harder into an obstruction.
                    target_scale, target_accel, status = min(scale, 1.), minimum, "NO_PROGRESS"
            scale += clip(target_scale-scale, -self.c["adaptive_gain_slew_s"]*dt, self.c["adaptive_gain_slew_s"]*dt)
            accel += clip(target_accel-accel, -maximum*dt, maximum*dt)
            scales.append(clip(scale, lo, hi))
            accels.append(clip(accel, minimum, maximum))
            corrections.append(correction)
            states.append(status)
            squares.append(self.error_square[axis]+alpha*(e*e-self.error_square[axis]))
        self.velocity, self.scale, self.accel = velocity, tuple(scales), tuple(accels)
        self.correction, self.status, self.error_square = tuple(corrections), tuple(states), tuple(squares)

    def snapshot(self):
        return dict(error_deg=self.error, error_rms_deg=tuple(math.sqrt(max(0.,v)) for v in self.error_square),
                    measured_velocity_deg_s=self.velocity, drive_scale=self.scale,
                    software_accel_deg_s2=self.accel, rate_correction_deg_s=self.correction, status=self.status,
                    torque_feedback_available=False, driver_pid_modified=False)
