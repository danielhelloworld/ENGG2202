"""Bounded, observation-driven response experiments. No hardware imports."""
import copy
import math


class AutoTuneExperiment:
    """One axis, matched +/- steps, baseline repeat and winner verification.

    apply(profile) runs only after centering has settled. set_offset(degrees)
    changes either a visual setpoint or a guarded local mechanical target.
    All callbacks execute in the caller's control loop, never an extra UART thread.
    """
    def __init__(self, apply, set_offset, amplitude=1.5, segment_s=1.5, tolerance=.3, amplitude_limit=4.):
        # Larger physical trials require an explicit caller limit and a callback
        # that ramps each waypoint through its own per-axis working limits.
        if (not math.isfinite(amplitude_limit) or not .5 <= amplitude_limit <= 15
                or not .5 <= amplitude <= amplitude_limit or not 1 <= segment_s <= 3 or not .1 <= tolerance <= 3):
            raise ValueError("Invalid bounded experiment settings")
        self.apply = apply
        self.set_offset = set_offset
        self.amplitude = amplitude
        self.segment_s = segment_s
        self.tolerance = tolerance
        self.active = False
        self.state = "IDLE"
        self.results = []
        self.best = None
        self.reason = None
        self.restore_pending = False

    def start(self, baseline, candidates, now):
        if self.active:
            raise RuntimeError("Auto tune already active")
        if not math.isfinite(now) or not 1 <= len(candidates) <= 8:
            raise ValueError("Invalid bounded candidate search")
        self.baseline = copy.deepcopy(baseline)
        self.candidates = [copy.deepcopy(baseline)] + copy.deepcopy(candidates)
        self.index = 0
        self.kind = "search"
        self.best = copy.deepcopy(baseline)
        self.best_score = None
        self.results = []
        self.reason = None
        self.restore_pending = False
        self.started = self.last_stamp = now
        self.active = True
        self._center(now)

    def _center(self, now):
        self.state = "CENTERING"
        self.phase_started = now
        self.stable_since = None
        self.set_offset(0.)

    def cancel(self, now, reason="user cancelled"):
        if self.active:
            self.reason = reason
            self.state = "CANCELLING"
            self.phase_started = now
            self.stable_since = None
            self.restore_pending = True
            try:
                self.set_offset(0.)
            except Exception as exc:
                self.fail("Return-to-center failed: " + str(exc))

    def fail(self, reason):
        # A feedback/transport failure cannot establish that rollback succeeded.
        self.active = False
        self.state = "FAILED"
        self.reason = reason
        self.restore_pending = True

    def observe(self, now, error_deg, velocity_deg_s=0., valid=True, gyro_rms=None):
        if not self.active:
            return
        if (not valid or not all(math.isfinite(v) for v in (now, error_deg, velocity_deg_s))
                or now <= self.last_stamp or now-self.last_stamp > .25):
            self.fail("Missing, stale or invalid observation; restore pending")
            return
        dt = now-self.last_stamp
        self.last_stamp = now
        if now-self.started > 120 and self.state != "CANCELLING":
            self.cancel(now, "experiment timeout")
        if self.state in ("CENTERING", "CANCELLING"):
            stable = abs(error_deg) <= self.tolerance and abs(velocity_deg_s) <= 1.
            self.stable_since = (now if self.stable_since is None else self.stable_since) if stable else None
            if self.stable_since is not None and now-self.stable_since >= .4:
                try:
                    if self.state == "CANCELLING":
                        self.apply(copy.deepcopy(self.baseline))
                        self.restore_pending = False
                        self.state, self.active = "CANCELLED", False
                        return
                    if self.kind == "finish":
                        self.apply(copy.deepcopy(self.best))
                        self.state, self.active = "COMPLETE", False
                        return
                    self.profile = copy.deepcopy(self.candidates[self.index] if self.kind == "search" else
                                                 self.baseline if self.kind == "baseline_repeat" else self.best)
                    self.apply(self.profile)
                    self.segment = 0
                    self.area = self.duration = self.peak = self.tail_area = self.tail_duration = 0.
                    self.gyro_area = self.gyro_duration = 0.
                    self.position_min = self.position_max = 0.
                    self._step(now)
                except Exception as exc:
                    self.fail("Profile/target application failed: " + str(exc))
                return
            if now-self.phase_started > 8:
                if self.state == "CANCELLING":
                    self.fail("Centering for rollback timed out")
                else:
                    self.cancel(now, "centering did not settle")
            return
        if self.state != "MEASURING":
            return
        self.area += error_deg*error_deg*dt
        inferred_position = (self.amplitude, 0., -self.amplitude, 0.)[self.segment]-error_deg
        self.position_min = min(self.position_min, inferred_position)
        self.position_max = max(self.position_max, inferred_position)
        self.duration += dt
        self.peak = max(self.peak, abs(error_deg))
        if now-self.phase_started >= self.segment_s-.5:
            self.tail_area += error_deg*error_deg*dt
            self.tail_duration += dt
            if gyro_rms is not None:
                if not math.isfinite(gyro_rms) or gyro_rms < 0:
                    self.fail("Invalid gyro observation")
                    return
                self.gyro_area += gyro_rms*gyro_rms*dt
                self.gyro_duration += dt
        if now-self.phase_started < self.segment_s:
            return
        self.segment += 1
        if self.segment < 4:
            try:
                self._step(now)
            except Exception as exc:
                self.fail("Step command failed: " + str(exc))
            return
        rms = math.sqrt(self.area/self.duration)
        tail = math.sqrt(self.tail_area/self.tail_duration)
        record = dict(kind=self.kind, profile=copy.deepcopy(self.profile), rmse_deg=rms,
                      peak_error_deg=self.peak, tail_rmse_deg=tail,
                      gyro_rms=math.sqrt(self.gyro_area/self.gyro_duration) if self.gyro_duration else None,
                      score=rms+.5*tail+.1*self.peak, duration_s=self.duration)
        self.results.append(record)
        if self.position_max-self.position_min < .3:
            self.cancel(now, "Insufficient measured response for calibration")
            return
        self._next(now, record)

    def _step(self, now):
        # Always return to center between +/- steps, preserving the local path.
        offset = (self.amplitude, 0., -self.amplitude, 0.)[self.segment]
        self.set_offset(offset)
        self.state = "MEASURING"
        self.phase_started = now

    def _next(self, now, record):
        score = record["score"]
        if self.kind == "search":
            if self.index == 0:
                self.initial_score = score
            if self.best_score is None or score < self.best_score:
                self.best_score, self.best = score, copy.deepcopy(self.profile)
            self.index += 1
            if self.index >= len(self.candidates):
                self.kind = "baseline_repeat"
        elif self.kind == "baseline_repeat":
            self.repeat_score = score
            if abs(score-self.initial_score) > max(.1, .35*self.initial_score):
                self.cancel(now, "Baseline drift; trials are not comparable")
                return
            self.kind = "verify"
        else:
            accepted = score < self.repeat_score*.95
            self.best = copy.deepcopy(self.best if accepted else self.baseline)
            self.reason = "Verified improvement" if accepted else "Baseline retained after verification"
            self.kind = "finish"
            self._center(now)
            return
        self._center(now)

    def snapshot(self):
        return dict(active=self.active, state=self.state, kind=getattr(self, "kind", None),
                    candidate=getattr(self, "index", 0), reason=self.reason,
                    restore_pending=self.restore_pending, best=copy.deepcopy(self.best),
                    trials=copy.deepcopy(self.results))
