"""1101 mainweb semantics: same ID during 2s loss; timeout clears and homes."""


class ClickTracker:
    def __init__(self, c):
        self.c = c
        self.selected = self.last_seen = self.last_tick = None
        self.filtered = [0., 0.]
        self.vector = (0., 0.)
        self.state = "DETECT"
        self.c_next_home = False
        self.predictor = MotionPredictor(c)
        self.prediction_point = None
        self.prediction_active = False

    def clear(self, arm_c=False):
        self.selected = self.last_seen = self.last_tick = None
        self.filtered = [0., 0.]
        self.vector = (0., 0.)
        self.state = "DETECT"
        self.c_next_home = arm_c
        self.predictor.reset()
        self.prediction_point = None
        self.prediction_active = False

    def select(self, key, tracks, now):
        key = tuple(key)
        if not any(tuple(t["key"]) == key for t in tracks):
            raise RuntimeError("Target is no longer visible")
        self.clear()
        self.selected, self.last_seen, self.state = key, now, "TRACK"

    def drive(self, now):
        if self.state == "COAST":
            age = max(0., now-self.last_seen)
            if age >= self.c["prediction_coast_s"]:
                self.state, self.vector = "LOST", (0., 0.)
                self.prediction_point, self.prediction_active = None, False
            else:
                fade = 1-age/self.c["prediction_coast_s"]
                self.vector = tuple(clip(v*f, -cap, cap)*fade/rate for v,f,cap,rate in zip(
                    self.predictor.motion_velocity, self.c["camera_fov"], self.c["prediction_coast_rate"], self.c["max_rate"]))
        return self.vector

    def step(self, tracks, now, pose=None, latency=0.):
        dt = min(.2, max(.02, now-self.last_tick)) if self.last_tick is not None else .1
        self.last_tick = now
        if self.selected is None:
            self.vector = (0., 0.)
            return None
        target = next((t for t in tracks if tuple(t["key"]) == self.selected), None)
        if target is None:
            self.vector = (0., 0.)
            self.filtered = [0., 0.]
            self.state = "LOST"
            if now-self.last_seen > self.c["lost_timeout"]:
                self.clear()
                self.state = "RETURNING"
                return "home"
            if self.predictor.ready and any(self.predictor.motion_velocity) and now-self.last_seen < self.c["prediction_coast_s"]:
                self.state = "COAST"
                self.prediction_active = True
                self.drive(now)
            else:
                self.prediction_point, self.prediction_active = None, False
            return None
        self.last_seen, self.state = now, "TRACK"
        error = (target["cx"]-.5, .5-target["cy"])
        self.predictor.observe(error, pose, now, target.get("score", 1.))
        predicted = self.predictor.lead(error, latency)
        self.prediction_active = self.predictor.ready and any(self.predictor.motion_velocity)
        self.prediction_point = (.5+predicted[0], .5-predicted[1]) if self.prediction_active else None
        vector = []
        for axis, e in enumerate(predicted):
            if abs(e) < self.c["deadzone"][axis]:
                self.filtered[axis] = 0.
            else:
                angular = e*self.c["camera_fov"][axis]
                blend = clip(abs(angular)/self.c["tracking_blend_deg"][axis], 0., 1.)
                gain = self.c["tracking_near_kp"][axis]+blend*(self.c["tracking_far_kp"][axis]-self.c["tracking_near_kp"][axis])
                value = angular*gain*.1*self.c["visual_gain"][axis]/.7
                if value*self.filtered[axis] < 0:
                    self.filtered[axis] = 0.  # Never retain the old push after crossing center.
                # Reference EMA and step caps are specified at 10Hz.
                alpha = 1-(1-self.c["ema"][axis])**(dt/.1)
                self.filtered[axis] += alpha*(value-self.filtered[axis])
            feedforward = self.predictor.motion_velocity[axis]*self.c["camera_fov"][axis]*self.c["prediction_feedforward"] if self.predictor.ready else 0.
            cap = min(self.c["step_cap"][axis]/.1, self.c["tracking_rate"][axis])
            rate = clip((self.filtered[axis]/.1+feedforward)*self.c["tracking_speed_multiplier"], -cap, cap)
            vector.append(rate/self.c["max_rate"][axis])
        n = max(1., math.hypot(*vector))
        self.vector = tuple(v/n for v in vector)
        return None


def contain_rect(iw, ih, viewport=(0, 40, 480, 360)):
    x, y, w, h = viewport
    scale = min(w/iw, h/ih)
    sw, sh = int(iw*scale), int(ih*scale)
    return x+(w-sw)//2, y+(h-sh)//2, sw, sh


def screen_to_normalized(x, y, rect):
    rx, ry, rw, rh = rect
    if not rx <= x < rx+rw or not ry <= y < ry+rh:
        return None
    return (x-rx)/rw, (y-ry)/rh
