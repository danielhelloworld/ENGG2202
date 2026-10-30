"""Home inertia/zero-crossing tolerance and persistent faults in PC models."""
import unittest
import test_separate_controls as controls
import test_tracker as app

m = app.m


class HomeToleranceTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable

    def home_driver(self):
        driver,bus = self.make_driver((20.,-10.))
        self.enable(driver)
        driver.request_home()
        driver._begin_home()
        driver.phase = "RETURNING"
        return driver,bus

    def test_startup_inertia_of_one_degree_does_not_disable_homing(self):
        driver,bus = self.home_driver()
        bus.physical[1] = 21.
        driver._measure()
        driver._advance((0.,0.),driver.c["home_rate"])
        self.assertTrue(driver.armed)
        self.assertIsNone(driver.motion_warning)
        self.assertLessEqual(bus.physical[1],21.)

    def test_small_zero_crossing_keeps_enable_and_can_command_zero(self):
        driver,bus = self.make_driver((.1,0.))
        self.enable(driver)
        driver.request_home()
        driver._begin_home()
        driver.phase = "RETURNING"
        bus.physical[1] = -.7
        driver._measure()
        driver._advance((0.,0.),driver.c["home_rate"])
        self.assertTrue(driver.armed)
        self.assertIsNone(driver.fault)
        self.assertGreaterEqual(bus.physical[1],-.7)
        for _ in range(5):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance((0.,0.),driver.c["home_rate"])
        self.assertAlmostEqual(bus.physical[1],0.,delta=.1)

    def test_brief_two_to_five_degree_deviation_pauses_without_disabling(self):
        driver,bus = self.home_driver()
        bus.physical[1] = 22.4
        driver._measure()
        self.assertIsNotNone(driver.motion_warning)
        before = len([p for p in bus.writes if p[2] == 3])
        driver._advance((0.,0.),driver.c["home_rate"])
        self.assertEqual(len([p for p in bus.writes if p[2] == 3]),before)
        self.assertTrue(driver.armed)
        self.clock.sleep(.15)
        bus.physical[1] = 21.
        driver._measure()
        driver._advance((0.,0.),driver.c["home_rate"])
        self.assertTrue(driver.armed)
        self.assertIsNone(driver.motion_warning)

    def test_persistent_path_escape_still_disables_and_is_not_labeled_zero_loss(self):
        driver,bus = self.home_driver()
        bus.physical[1] = 22.4
        driver._measure()
        self.clock.sleep(.35)
        with self.assertRaisesRegex(m.MotionProtectionFault,"MOTION_PATH_PERSISTENT"):
            driver._measure()
        self.assertFalse(driver.armed)
        self.assertFalse(any(bus.enabled.values()))
        self.assertNotIn("ZERO_REFERENCE_LOST",driver.fault)

    def test_large_path_escape_is_still_an_immediate_fault(self):
        driver,bus = self.home_driver()
        self.clock.sleep(.2)
        bus.physical[1] = 25.6
        with self.assertRaisesRegex(m.MotionProtectionFault,"MOTION_HARD_LIMIT"):
            driver._measure()
        self.assertFalse(driver.armed)

    def test_short_feedback_dropout_can_recover_before_new_500ms_deadline(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        bus.drop_feedback = {1}
        for _ in range(4):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance(driver.angles,driver.c["max_rate"])
        self.assertTrue(driver.armed)
        bus.drop_feedback.clear()
        driver._measure()
        self.assertTrue(driver.armed)
        self.assertFalse(driver.snapshot()["feedback_degraded"])

    def test_delayed_mechanical_start_under_one_second_is_not_a_stall(self):
        driver,bus = self.make_driver((20.,-10.))
        self.enable(driver)
        driver.request_home()
        bus.stuck = True
        started = self.clock.now
        def pump():
            driver.input(0.,0.)
            if self.clock.now-started > .9 and bus.stuck:
                bus.stuck = False
                # A real motor continues toward its held target without another
                # UART packet; this fixture otherwise moves only on writes.
                for mid in bus.enabled:
                    if bus.enabled[mid]:
                        bus.physical[mid] += m.signed_mechanical(bus.target[mid]-bus.physical[mid])
        self.clock.pump = pump
        driver._home_and_clear()
        self.assertEqual(driver.phase,"HOME_HOLD")
        self.assertTrue(driver.armed)
        self.assertIsNone(driver.fault)

    def test_delayed_control_loop_holds_before_hard_watchdog(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        driver.phase = "MANUAL"
        driver.input(1.,0.)
        self.clock.pump = None
        self.clock.sleep(.6)
        driver._measure()
        count = len([p for p in bus.writes if p[2] == 3])
        driver._advance((0.,0.),driver.c["max_rate"])
        self.assertTrue(driver.armed)
        self.assertTrue(driver.motion_paused)
        self.assertEqual(len([p for p in bus.writes if p[2] == 3]),count)
        self.clock.pump = lambda:driver.input(.3,0.)
        self.clock.sleep(.02)
        driver._measure()
        driver._advance((0.,0.),driver.c["max_rate"])
        self.assertTrue(driver.armed)
        self.assertFalse(driver.motion_paused)

    def test_hard_watchdog_still_stops_control_after_long_loss(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        self.clock.pump = None
        self.clock.sleep(1.25)
        with self.assertRaisesRegex(RuntimeError,"heartbeat expired") as caught:
            driver._advance((0.,0.),driver.c["max_rate"])
        driver._record_fault(caught.exception)
        self.assertFalse(driver.armed)
        self.assertFalse(any(bus.enabled.values()))


if __name__ == "__main__":
    unittest.main()
