"""Onboard visual-loop tuning around a stationary selected target."""
import json
import os


class SoftwareAutoTuner:
    KEYS = ("tracking_near_kp", "tracking_far_kp", "ema", "tracking_accel_deg_s2")

    def __init__(self, hub):
        self.hub = hub
        self.axis = None
        self.session = AutoTuneExperiment(self._apply, self._offset)
        self.origin = None
        self.selected = None
        self.last_pose = None
        self.saved_path = None

    @property
    def active(self):
        return self.session.active

    def _apply(self, profile):
        if set(profile) != set(self.KEYS):
            raise ValueError("Invalid software profile keys")
        proposed = dict(self.hub.c, **profile)
        validate_visual_config(proposed)
        for config in (self.hub.c, self.hub.driver.c):
            for key in self.KEYS:
                config[key] = tuple(profile[key])
        self.hub.tracking.filtered = [0., 0.]
        self.hub.tracking.settled = [False, False]

    def _offset(self, degrees):
        offset = [0., 0.]
        offset[self.axis] = degrees/self.hub.c["camera_fov"][self.axis]
        self.hub.tracking.tuning_offset = tuple(offset)
        self.hub.tracking.settled[self.axis] = False

    def start(self, axis):
        if isinstance(self.hub.driver,VelocityMotorService):
            raise RuntimeError('旧位置控制整定不适用于速度控制，请使用速度模式参数')
        if type(axis) is not int or axis not in (0, 1):
            raise ValueError("Choose one tuning axis")
        motor = self.hub.driver.snapshot()
        if (not self.hub.fresh() or self.hub.tracking.selected is None or not motor["armed"]
                or motor["fault"] or motor.get("home_requested") or motor.get("outside_hold")
                or any(self.hub.manual) or not motor["reference_valid"] or motor.get("feedback_degraded")
                or motor["age"] is None or motor["age"] > self.hub.c["pose_max_age_s"]):
            raise RuntimeError("Select a stationary target with fresh feedback and released controls")
        if self.active:
            raise RuntimeError("Software auto tune already running")
        selected = next((track for track in self.hub.latest["tracks"]
                         if tuple(track["key"]) == self.hub.tracking.selected), None)
        if selected is None or any(abs(error*fov) > 4 for error, fov in zip(
                (selected["cx"]-.5, .5-selected["cy"]), self.hub.c["camera_fov"])):
            raise RuntimeError("Wait until the stationary target is within 4 degrees of image center")
        self.axis = axis
        self.saved_path = None
        self.origin = tuple(motor["angles"])
        for angle, limits in zip(self.origin, self.hub.c["limits"]):
            if not limits[0]+6 <= angle <= limits[1]-6:
                raise RuntimeError("Move at least 6 degrees inside working limits before auto tune")
        self.selected = self.hub.tracking.selected
        self.last_pose = None
        self.session.amplitude = 4.
        self.session.tolerance = min(3., self.hub.c["deadzone"][axis]*self.hub.c["camera_fov"][axis]*self.hub.c["tracking_settle_ratio"]+.2)
        baseline = {key: tuple(self.hub.c[key]) for key in self.KEYS}
        candidates = []
        for factor in (.85, 1.15):
            candidate = copy.deepcopy(baseline)
            for key in ("tracking_near_kp", "tracking_far_kp"):
                values = list(candidate[key])
                values[axis] *= factor
                candidate[key] = tuple(values)
            candidates.append(candidate)
        for change in (-.1, .1):
            candidate = copy.deepcopy(baseline)
            values = list(candidate["ema"])
            values[axis] = max(.2, min(.8, values[axis]+change))
            candidate["ema"] = tuple(values)
            candidates.append(candidate)
        candidate = copy.deepcopy(baseline)
        values = list(candidate["tracking_accel_deg_s2"])
        values[axis] *= .85
        candidate["tracking_accel_deg_s2"] = tuple(values)
        candidates.append(candidate)
        self.session.start(baseline, candidates, self.hub.clock())

    def observe(self, frame, motor):
        if not self.active:
            return
        now = frame["stamp"]
        if now <= self.session.last_stamp:
            return  # Discard the in-flight double-buffer frame from before START.
        valid = (motor["armed"] and not motor["fault"] and motor["reference_valid"]
                 and not motor.get("feedback_degraded") and not motor.get("home_requested")
                 and motor["age"] is not None and motor["age"] <= self.hub.c["pose_max_age_s"]
                 and self.hub.tracking.selected == self.selected
                 and self.hub.tracking.state == "TRACK"
                 and all(abs(a-b) <= 8 for a, b in zip(motor["angles"], self.origin)))
        if not valid:
            self.abort("Target or mechanical feedback lost during software auto tune")
            return
        error = self.hub.tracking.control_error[self.axis]*self.hub.c["camera_fov"][self.axis]
        pose = motor["angles"][self.axis]
        speed = 0.
        if self.last_pose is not None:
            speed = (pose-self.last_pose[1])/(now-self.last_pose[0])
        self.last_pose = (now, pose)
        self.session.observe(now, error, speed)
        if not self.active:
            self._offset(0.)
            if self.session.state == "FAILED":
                self.abort(self.session.reason)

    def abort(self, reason="software auto tune cancelled"):
        if not hasattr(self.session, "baseline"):
            return
        self.session.active = False
        self.session.state = "CANCELLED"
        self.session.reason = reason
        self._apply(self.session.baseline)
        self._offset(0.)
        self.session.restore_pending = False
        self.hub.tracking.clear()
        self.hub.manual = (0., 0.)

    def export(self):
        result = self.session.snapshot()
        result.update(axis=self.axis, selected=self.selected, origin=self.origin,
                      saved_path=self.saved_path,
                      sensor_basis="visual error and encoder motion; IMU not enabled")
        return result

    def save(self):
        if self.active or self.session.state != "COMPLETE":
            raise RuntimeError("Finish verification before saving the software profile")
        path = self.hub.c["software_autotune_profile"]
        directory = os.path.dirname(path)
        os.makedirs(directory, exist_ok=True)
        profile = dict(schema=1, model_path=self.hub.c["model_path"],
                       camera_fov=self.hub.c["camera_fov"], motor_signs=self.hub.c["motor_signs"],
                       joystick_signs=self.hub.c["joystick_signs"], limits=self.hub.c["limits"],
                       parameters=self.session.best, experiment=self.export())
        with open(path+".tmp", "w", encoding="utf-8") as output:
            json.dump(profile, output, ensure_ascii=False, allow_nan=False)
        os.replace(path+".tmp", path)
        self.saved_path = path
        return path

    def load(self):
        path = self.hub.c["software_autotune_profile"]
        try:
            with open(path, encoding="utf-8") as source:
                saved = json.load(source)
        except FileNotFoundError:
            return False
        if saved.get("schema") != 1:
            raise ValueError("Unsupported saved software profile")
        for key in ("model_path", "camera_fov", "motor_signs", "joystick_signs", "limits"):
            if json.dumps(saved.get(key)) != json.dumps(self.hub.c[key]):
                raise ValueError("Saved profile configuration differs: " + key)
        self._apply(saved["parameters"])
        return True
