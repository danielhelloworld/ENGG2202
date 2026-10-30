"""Faster manual/home profile and smoother rates: PC motor model only."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m = app.m


class FastManualTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable

    def move(self, driver, vector, count=60, dt=.02):
        self.clock.pump = lambda:driver.input(*vector)
        driver.set_response_context("MANUAL")
        driver.phase = "MANUAL"
        rows = []
        for _ in range(count):
            self.clock.sleep(dt)
            driver._measure()
            driver._advance(driver.angles, driver.c["max_rate"])
            rows.append((tuple(driver.angles), tuple(driver.stream_target), tuple(driver.stream_velocity)))
        return rows

    def test_generated_profile_validates_but_rejects_unbounded_speed_acceleration_jerk(self):
        m.validate_visual_config(m.CONFIG)
        for change in (dict(motor_rpm=81), dict(command_accel_deg_s2=(2001.,1500.)),
                       dict(manual_jerk_deg_s3=(40001.,20000.)), dict(manual_response_s=.001),
                       dict(manual_rate=(241.,180.))):
            with self.subTest(change=change), self.assertRaises(ValueError):
                m.validate_visual_config(dict(m.CONFIG, **change))

    def test_arm_programs_the_new_driver_speed_without_zero_or_save_commands(self):
        driver, bus = self.make_driver((0.,0.))
        self.enable(driver)
        speeds = [int.from_bytes(p[3:5],"big",signed=True) for p in bus.writes if p[2] == 1]
        self.assertEqual(speeds, [40,30])
        self.assertFalse(any(p[2] in (2,8,9,10) for p in bus.writes))

    def test_full_stick_travels_faster_than_old_profile_on_each_axis(self):
        for axis, previous in enumerate((30.,24.)):
            distances = []
            for cap in (previous, m.CONFIG["manual_rate"][axis]):
                rates = list(m.CONFIG["manual_rate"])
                rates[axis] = cap
                driver, bus = self.make_driver((0.,0.), manual_rate=tuple(rates))
                self.enable(driver)
                vector = (1.,0.) if axis == 0 else (0.,1.)
                self.move(driver,vector,count=60)
                distances.append(abs(bus.physical[axis+1]))
                driver._disable_all()
            with self.subTest(axis=axis):
                self.assertGreater(distances[1], distances[0]*1.25)

    def test_start_and_reverse_have_bounded_acceleration_and_jerk_with_irregular_cycles(self):
        for axis, cap in enumerate(m.CONFIG["manual_rate"]):
            ramp = m.ManualRateRamp(m.CONFIG)
            velocity, last_a, now, positive, negative = 0.,0.,0.,[],[]
            for i in range(220):
                dt = (.012,.028,.06,.018)[i%4]
                desired = cap if now < 1. else -cap
                velocity = ramp.advance(axis,velocity,desired,m.CONFIG["command_accel_deg_s2"][axis],dt,cap)
                a = ramp.acceleration[axis]
                self.assertLessEqual(abs(a), m.CONFIG["command_accel_deg_s2"][axis]+1e-8)
                self.assertLessEqual(abs(a-last_a), m.CONFIG["manual_jerk_deg_s3"][axis]*dt+1e-8)
                self.assertLessEqual(abs(velocity),cap+1e-8)
                (positive if desired > 0 else negative).append(velocity)
                now, last_a = now+dt,a
            self.assertGreater(max(positive), .95*cap)
            self.assertLess(negative[-1], -.95*cap)

    def test_ramp_is_crisp_without_an_instant_full_speed_step(self):
        ramp = m.ManualRateRamp(m.CONFIG)
        velocity = ramp.advance(0,0.,240.,765.,.02,240.)
        self.assertGreater(velocity,0.)
        self.assertLess(velocity,10.)
        for _ in range(29):
            velocity = ramp.advance(0,velocity,240.,765.,.02,240.)
        self.assertGreater(velocity,240.*.95)  # At least 95% of target within 600ms.

    def test_normal_limit_approach_brakes_and_never_sends_an_outside_target(self):
        for pose, vector, axis, bound in (((-138.,0.),(1.,0.),0,-150.),
                                        ((0.,18.),(0.,-1.),1,30.)):
            driver, bus = self.make_driver(pose)
            self.enable(driver)
            rows = self.move(driver,vector,count=120)
            self.assertGreater(max(abs(r[2][axis]) for r in rows),5.)
            self.assertLess(abs(rows[-1][2][axis]),1.)
            self.assertLess(abs(bus.physical[axis+1]-bound),.6)
            for _,targets,_ in rows:
                self.assertTrue(all(lo <= q <= hi for q,(lo,hi) in zip(targets,driver.c["limits"])))
            driver._disable_all()

    def test_lead_saturation_does_not_repeat_zero_speed_restarts(self):
        driver, bus = self.make_driver((0.,0.))
        self.enable(driver)
        bus.stuck = True
        rows = self.move(driver,(1.,0.),count=40)
        self.assertTrue(all(abs(target[0]-actual[0]) <= driver.c["manual_position_lead_deg"][0]+1e-8 for actual,target,_ in rows))
        self.assertGreater(abs(rows[-1][2][0]),200.)
        self.assertTrue(driver.armed)

    def test_stuck_motor_still_faults_with_the_faster_ramp(self):
        driver, bus = self.make_driver((0.,0.))
        self.enable(driver)
        bus.stuck = True
        with self.assertRaisesRegex(RuntimeError,"SUSPECTED_BLOCK"):
            self.move(driver,(1.,0.),count=240)

    def test_release_still_holds_immediately_without_a_smoothing_tail(self):
        driver, bus = self.make_driver((0.,0.))
        self.enable(driver)
        self.move(driver,(1.,0.),count=25)
        driver._measure()
        actual = driver.angles
        driver.input(0.,0.)
        driver.set_response_context("HOLD")
        driver._release_hold()
        driver._advance(actual, driver.c["max_rate"])
        self.assertEqual(driver.stream_velocity,(0.,0.))
        self.assertEqual(driver.manual_ramp.acceleration,[0.,0.])
        self.assertEqual(driver.stream_target,actual)

    def test_disarm_and_feedback_pause_clear_the_ramp(self):
        driver, bus = self.make_driver((0.,0.))
        self.enable(driver)
        self.move(driver,(1.,0.),count=5)
        self.assertTrue(any(driver.manual_ramp.acceleration))
        driver.feedback_issues[0] = "temporary missing reply"
        count = len(bus.writes)
        driver._advance(driver.angles,driver.c["max_rate"])
        self.assertEqual(len(bus.writes),count)
        self.assertEqual(driver.manual_ramp.acceleration,[0.,0.])
        driver._disable_now()
        self.assertFalse(any(bus.enabled.values()))

    def test_home_profile_is_faster_but_still_arrives_and_holds_enabled(self):
        durations = []
        for rate in ((24.,20.),m.CONFIG["home_rate"]):
            driver, bus = self.make_driver((70.,-65.),home_rate=rate)
            self.enable(driver)
            driver.request_home()
            start = self.clock.now
            driver._home_and_clear()
            durations.append(self.clock.now-start)
            self.assertEqual(driver.phase,"HOME_HOLD")
            self.assertTrue(driver.armed)
            self.assertTrue(all(abs(q) <= .5 for q in driver.angles))
            driver._disable_all()
        self.assertLess(durations[1],durations[0]*.85)

    def test_tracking_and_home_caps_remain_distinct_from_faster_manual(self):
        self.assertEqual(m.CONFIG["tracking_rate"],(150.,120.))
        self.assertEqual(m.CONFIG["home_rate"],(108.,90.))
        driver, bus = self.make_driver((0.,0.))
        self.enable(driver)
        driver.phase = "MANUAL"  # Shared worker name for a continuous rate stream.
        driver.set_response_context("TRACK")
        self.clock.pump = lambda:driver.input(1.,0.)
        for _ in range(50):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance(driver.angles,driver.c["max_rate"])
            self.assertLessEqual(abs(driver.stream_velocity[0]),150.+1e-8)


if __name__ == "__main__":
    unittest.main()
