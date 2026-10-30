"""Requested 3/5/10 profile ratios and protection at high rates: PC only."""
import unittest
import test_adapter as adapters
import test_tracker as app
import test_separate_controls as controls

m = app.m


class ContinuousMotorModel(adapters.fixtures.MechanicalMotorModel):
    """Follow held positions over simulated time, instead of teleporting on writes."""
    def __init__(self, clock, positions):
        super().__init__(positions)
        self.clock, self.sim_stamp = clock, clock.now
        self.applied_rpm = {1:80,2:80}

    def write(self, data):
        dt = self.clock.now-self.sim_stamp
        self.sim_stamp = self.clock.now
        for mid in self.physical:
            if self.enabled[mid] and not self.stuck:
                delta = m.signed_mechanical(self.target[mid]-self.physical[mid])
                rate = self.applied_rpm[mid]*6
                self.physical[mid] += m.clip(delta,-rate*dt,rate*dt)
        if data[2] == 1:
            self.applied_rpm[data[1]] = abs(int.from_bytes(data[3:5],"big",signed=True))
        old_stuck = self.stuck
        if data[2] == 3:
            self.stuck = True  # Only suppress this fixture's instantaneous move.
        try:
            return super().write(data)
        finally:
            self.stuck = old_stuck


class SpeedMultiplierTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable

    def test_rates_match_user_revised_profile(self):
        self.assertEqual(m.CONFIG["manual_rate"],tuple(.5*v for v in (480.,360.)))
        self.assertEqual(m.CONFIG["home_rate"],tuple(3*v for v in (36.,30.)))
        self.assertEqual(m.CONFIG["startup_home_rate"],m.CONFIG["home_rate"])
        self.assertEqual(m.CONFIG["tracking_rate"],tuple(5*v for v in (30.,24.)))
        self.assertEqual(m.CONFIG["motor_rpm"]*6,max(m.CONFIG["manual_rate"]))

    def test_tracking_multiplier_changes_unsaturated_requested_motion(self):
        target = app.track(box=(300,210,80,60))
        rates = []
        for multiplier in (1.,5.):
            core = m.ClickTracker(dict(m.CONFIG,prediction_enabled=False,tracking_speed_multiplier=multiplier))
            core.select((0,1),[target],1.)
            core.step([target],1.1)
            rates.append(core.vector[0]*m.CONFIG["max_rate"][0])
        self.assertGreater(rates[0],0.)
        self.assertAlmostEqual(rates[1],rates[0]*5.,places=6)

    def test_each_motion_mode_uses_its_own_bounded_horizon_and_acceleration(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        driver.input(1.,0.)
        driver.set_response_context("MANUAL")
        self.assertEqual(driver._position_lead_deg(0),12.)
        self.assertLessEqual(driver._effective_accel()[0],900.)
        driver.set_response_context("TRACK")
        self.assertEqual(driver._position_lead_deg(0),10.)
        self.assertLessEqual(driver._effective_accel()[0],900.)
        driver.request_home()
        driver._begin_home()
        self.assertEqual(driver._position_lead_deg(0),8.)
        self.assertLessEqual(driver._effective_accel()[0],540.)
        driver._disable_all()

    def test_final_uart_gate_rejects_targets_beyond_the_active_horizon(self):
        for context,lead in (("MANUAL",12.),("TRACK",10.),("COAST",3.)):
            driver,bus = self.make_driver((0.,0.))
            self.enable(driver)
            driver.set_response_context(context)
            self.clock.pump = lambda:driver.input(1.,0.)
            driver.input(1.,0.)
            before = len(bus.writes)
            with self.assertRaisesRegex(RuntimeError,"Unsafe single-turn step"):
                driver._write(m.set_single_turn_position(1,lead+.2))
            self.assertEqual(len(bus.writes),before)
            driver._write(m.set_single_turn_position(1,lead))
            self.assertAlmostEqual(driver.position_intent[0],lead)
            driver._disable_all()

    def test_manual_outside_work_limits_keeps_three_degree_inward_steps(self):
        driver,bus = self.make_driver((165.,116.2))
        self.enable(driver)
        driver.set_response_context("MANUAL")
        driver.input(1.,1.)
        self.assertEqual(driver._position_lead_deg(0),3.)
        with self.assertRaisesRegex(RuntimeError,"local inward step"):
            driver._write(m.set_single_turn_position(1,161.8))
        driver._write(m.set_single_turn_position(1,162.))
        driver._disable_all()

    def test_coast_cannot_inherit_a_fast_manual_or_tracking_velocity(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        driver.phase = "MANUAL"
        driver.stream_velocity = (-400.,0.)
        driver.set_response_context("COAST")
        self.clock.pump = lambda:driver.input(1.,0.)
        self.clock.sleep(.02)
        driver._measure()
        driver._advance(driver.angles,driver.c["max_rate"])
        self.assertLessEqual(abs(driver.stream_velocity[0]),4.)

    def test_high_speed_stream_with_continuous_held_target_feedback_stays_trusted(self):
        bus = ContinuousMotorModel(self.clock,(100.,0.))
        driver = m.VisualMotorService(dict(m.CONFIG,startup_auto_home=False),lambda:bus)
        driver.serial,driver.stop_event = bus,adapters.fixtures.Event(self.clock)
        driver.sent_target = driver.manual_target = driver._initialize()
        driver.ready,driver.phase = True,"DISABLED"
        self.clock.pump = lambda:driver.input(1.,0.)
        driver.input(1.,0.)
        self.enable(driver)
        driver.set_response_context("MANUAL")
        driver.phase = "MANUAL"
        samples = []
        for _ in range(50):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance(driver.angles,driver.c["max_rate"])
            self.assertFalse(any(driver.feedback_issues))
            self.assertTrue(driver.armed)
            samples.append((self.clock.now,driver.angles[0]))
        peak = max(abs(b[1]-a[1])/(b[0]-a[0]) for a,b in zip(samples,samples[1:]))
        self.assertGreater(peak,48.*3)
        self.assertTrue(all(-150 <= q <= 150 for _,q in samples))

    def test_stall_fault_occurs_after_1_8_seconds_without_real_progress(self):
        self.assertAlmostEqual(m.CONFIG["motion_grace_s"]+m.CONFIG["motion_window_s"],1.8)
        guard = m.TrackingMotionGuard(dict(m.CONFIG))
        guard.update(1.,(0.,0.),(3.,0.))
        guard.update(2.79,(0.,0.),(3.,0.))
        with self.assertRaisesRegex(RuntimeError,"SUSPECTED_BLOCK"):
            guard.update(2.81,(0.,0.),(3.,0.))
        guard.reset()
        guard.check_request(1.,(0.,0.),(3.,0.),(100.,0.))
        guard.check_request(2.79,(0.,0.),(3.,0.),(100.,0.))
        with self.assertRaisesRegex(RuntimeError,"SUSPECTED_BLOCK"):
            guard.check_request(2.81,(0.,0.),(3.,0.),(100.,0.))

    def test_driver_speed_follows_mode_without_an_enable_or_zero_command(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        self.assertEqual(driver.programmed_rpm,[40,30])
        before = len(bus.writes)
        driver.set_response_context("TRACK")
        driver._sync_driver_speed()
        self.assertEqual(driver.programmed_rpm,[25,20])
        driver.request_home()
        driver._begin_home()
        driver._sync_driver_speed()
        self.assertEqual(driver.programmed_rpm,[18,15])
        self.assertFalse(any(p[2] in (6,8,9,10) for p in bus.writes[before:]))


if __name__ == "__main__":
    unittest.main()
