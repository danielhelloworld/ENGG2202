"""1101 mainweb semantics: same ID during 2s loss; timeout clears and homes."""


class ClickTracker:
    def __init__(self, c):
        self.c = c
        self.selected = self.last_seen = self.last_tick = None
        self.filtered = [0., 0.]
        self.vector = (0., 0.)
        self.state = "DETECT"
        self.c_next_home = False
        self.predictor = (FastMotionPredictor if c['tracking_motor_control']=='speed' and c['native_tracking'] else MotionPredictor)(c)
        self.prediction_point = None
        self.prediction_active = False
        self.settled = [False,False]
        self.tuning_offset = (0., 0.)
        self.control_error = (0., 0.)
        self.aim_goal = None
        self.aim_velocity=(0.,0.)
        self.native_active_axes=(False,False)
        self.last_tracked_goal=None
        self.last_tracked_velocity=(0.,0.)
        self.overhead=OverheadPlanner(c)
        self.overhead_reacquired=0

    def clear(self, arm_c=False):
        self.selected = self.last_seen = self.last_tick = None
        self.filtered = [0., 0.]
        self.vector = (0., 0.)
        self.state = "DETECT"
        self.c_next_home = arm_c
        self.predictor.reset()
        self.prediction_point = None
        self.prediction_active = False
        self.settled = [False,False]
        self.tuning_offset = (0., 0.)
        self.aim_goal = None
        self.aim_velocity=(0.,0.);self.native_active_axes=(False,False)
        self.last_tracked_goal=None;self.last_tracked_velocity=(0.,0.)
        self.overhead.reset()
        self.overhead_reacquired=0

    def select(self, key, tracks, now):
        key = tuple(key)
        if not any(tuple(t["key"]) == key for t in tracks):
            raise RuntimeError("Target is no longer visible")
        self.clear()
        self.selected, self.last_seen, self.state = key, now, "TRACK"

    def drive(self, now, pose=None):
        if self.state=='OVERHEAD':
            plan=self.overhead.missing(now,pose)
            if plan is None:
                self.state='LOST';self.aim_goal=None;self.native_active_axes=(False,False)
            else:self.aim_goal=plan['goal'];self.native_active_axes=plan['active']
        if self.state == "COAST":
            age = max(0., now-self.last_seen)
            if age >= self.c["prediction_coast_s"]:
                self.state, self.vector = "LOST", (0., 0.)
                self.prediction_point, self.prediction_active = None, False
                self.aim_goal=None;self.native_active_axes=(False,False)
            else:
                fade = 1-age/self.c["prediction_coast_s"]
                self.vector = tuple(clip(v*f, -cap, cap)*fade/rate for v,f,cap,rate in zip(
                    self.predictor.motion_velocity, self.c["camera_fov"], self.c["prediction_coast_rate"], self.c["max_rate"]))
                if self.c['native_tracking'] and self.last_tracked_goal is not None:
                    distance_time=age-age*age/(2*self.c['prediction_coast_s'])
                    self.aim_goal=tuple(q+clip(v*distance_time,-cap,cap) for q,v,cap in
                        zip(self.last_tracked_goal,self.last_tracked_velocity,self.c['native_coast_max_deg']))
                    self.aim_velocity=tuple(v*fade for v in self.last_tracked_velocity)
                    self.native_active_axes=tuple(abs(v)>0 for v in self.last_tracked_velocity)
        return self.vector

    def step(self, tracks, now, pose=None, latency=0., current_pose=None):
        dt = min(.2, max(.02, now-self.last_tick)) if self.last_tick is not None else .1
        self.last_tick = now
        if self.selected is None:
            self.vector = (0., 0.)
            return None
        target = next((t for t in tracks if tuple(t["key"]) == self.selected), None)
        if target is None:
            target=self.overhead.reacquire(tracks,self.selected,pose,now)
            if target is not None:
                self.selected=tuple(target['key']);self.overhead_reacquired+=1;self.predictor.reset()
        if target is None:
            self.aim_goal = None
            self.vector = (0., 0.)
            self.filtered = [0., 0.]
            self.state = "LOST"
            plan=self.overhead.missing(now,current_pose)
            if plan is not None:
                self.state='OVERHEAD';self.aim_goal=plan['goal'];self.aim_velocity=(0.,0.)
                self.native_active_axes=plan['active'];return None
            if now-self.last_seen > self.c["lost_timeout"] and not self.overhead.waiting(now):
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
        control_error = error
        if pose is not None and current_pose is not None:
            control_error = tuple(e-clip(sign*(q-old)/f,-.2,.2) for e,sign,q,old,f in zip(
                error,self.c["joystick_signs"],current_pose,pose,self.c["camera_fov"]))
        control_error = tuple(e+offset for e, offset in zip(control_error, self.tuning_offset))
        self.control_error = control_error
        predicted = self.predictor.lead(control_error, latency)
        self.prediction_active = self.predictor.ready and any(self.predictor.motion_velocity)
        # Draw on the capture-time image, not on the newer compensated pose.
        display_prediction=self.predictor.lead(tuple(e+offset for e,offset in zip(error,self.tuning_offset)),latency)
        self.prediction_point = (.5+display_prediction[0], .5-display_prediction[1]) if self.prediction_active else None
        vector = []
        for axis, e in enumerate(predicted):
            zone=self.c['native_center_deadzone'] if self.c['native_tracking'] else self.c['deadzone']
            ratio=self.c['native_center_exit_ratio'] if self.c['native_tracking'] else self.c['tracking_settle_ratio']
            threshold = zone[axis]*(ratio if self.settled[axis] else 1.)
            self.settled[axis] = abs(e) < threshold
            if self.settled[axis]:
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
            cap = min(cap,max(0.,abs(e)-self.c["deadzone"][axis])*self.c["camera_fov"][axis]/self.c["tracking_stop_horizon_s"])
            rate = clip((self.filtered[axis]/.1+feedforward)*self.c["tracking_speed_multiplier"], -cap, cap)
            if self.settled[axis]:
                rate = 0.
            vector.append(rate/self.c["max_rate"][axis])
        n = max(1., math.hypot(*vector))
        self.vector = tuple(v/n for v in vector)
        self.aim_goal=(tuple(q+sign*e*f*gain*(visual/.7) for q,sign,e,f,gain,visual in
                             zip(current_pose,self.c['joystick_signs'],predicted,self.c['camera_fov'],
                                 self.c['native_aim_gain'],self.c['visual_gain'])) if current_pose is not None else None)
        self.aim_velocity=tuple(sign*v*f for sign,v,f in zip(self.c['joystick_signs'],self.predictor.motion_velocity,self.c['camera_fov']))
        self.native_active_axes=tuple(not settled or (self.predictor.ready and abs(v)>0) for settled,v in zip(self.settled,self.predictor.motion_velocity))
        plan=self.overhead.observe(error,pose,current_pose,now,target.get('score',1.)) if self.c['native_tracking'] else None
        if plan is not None:
            self.aim_goal=plan['goal'];self.aim_velocity=plan['velocity'];self.native_active_axes=plan['active']
            self.prediction_point=None  # Flat-image prediction does not describe zenith geometry.
        self.last_tracked_goal=self.aim_goal;self.last_tracked_velocity=self.aim_velocity
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
