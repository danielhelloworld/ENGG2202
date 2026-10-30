"""ARM holds, HOME moves, faster stream remains constrained: motor MODEL only."""
import contextlib
import io
import unittest
from unittest.mock import patch
import test_adapter as adapters
import test_tracker as app

m, fixtures = app.m, adapters.fixtures


class SeparateControlTests(unittest.TestCase):
    def setUp(self):
        self.clock = fixtures.Clock()
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(m.time, "monotonic", self.clock.monotonic))
        self.stack.enter_context(patch.object(m.time, "sleep", self.clock.sleep))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def make_driver(self, positions=(20., -10.), **settings):
        bus = fixtures.MechanicalMotorModel(positions)
        driver = m.VisualMotorService(dict(m.CONFIG, startup_auto_home=False, **settings), lambda:bus)
        driver.serial = bus
        driver.stop_event = fixtures.Event(self.clock)
        driver.sent_target = driver.manual_target = driver._initialize()
        driver.ready, driver.phase = True, "DISABLED"
        driver.last_send = self.clock.now
        self.clock.pump = lambda:driver.input(0., 0.)
        driver.input(0., 0.)  # Represent the live application's first heartbeat.
        driver.confirm_reference()
        return driver, bus

    def enable(self, driver):
        driver.request_arm(True)
        driver.sent_target = driver.manual_target = driver._arm_current()
        driver.armed = True
        driver.phase = "ARMED"
        driver.last_send = self.clock.now

    def test_arm_only_holds_current_pose_then_home_is_explicit(self):
        driver, bus = self.make_driver()
        self.enable(driver)
        self.assertFalse(driver.home_requested)
        self.assertFalse(driver.recovery_active)
        self.assertEqual(tuple(bus.physical.values()), (20., -10.))
        driver.request_home()
        driver._home_and_clear()
        self.assertEqual(driver.phase, "HOME_HOLD")
        self.assertTrue(driver.armed)
        self.assertTrue(all(abs(v) <= .5 for v in driver.angles))
        driver._disable_all()

    def test_outside_arm_is_hold_only_and_home_can_recover(self):
        driver, bus = self.make_driver((165., 116.2))
        self.enable(driver)
        self.assertTrue(driver.snapshot()["outside_hold"])
        self.assertFalse(driver.home_requested)
        count = len(bus.writes)
        with self.assertRaisesRegex(RuntimeError, "local inward step"):
            driver._write(m.set_single_turn_position(2, 117.2))
        self.assertEqual(len(bus.writes), count)
        driver.request_home()
        driver._home_and_clear()
        self.assertFalse(driver.snapshot()["outside_hold"])
        self.assertTrue(all(abs(v) <= .5 for v in driver.angles))
        driver._disable_all()

    def test_home_does_not_enable_from_disabled_and_requires_separate_arm(self):
        driver, bus = self.make_driver()
        with self.assertRaisesRegex(RuntimeError,"HOME 不会自动使能"):
            driver.request_home()
        self.assertFalse(driver.arm_requested or driver.home_requested)
        self.assertFalse(any(bus.enabled.values()))
        self.enable(driver)
        driver.request_home()
        driver._home_and_clear()
        self.assertTrue(all(abs(v) <= .5 for v in driver.angles))
        driver._disable_all()

    def test_worker_arm_stays_at_current_pose_without_requesting_home(self):
        for positions in ((20., -10.), (165., 116.2)):
            with self.subTest(positions=positions):
                driver, bus = self.make_driver(positions)
                driver.request_arm(True)
                armed_at = [None]

                def schedule():
                    if driver.armed:
                        if armed_at[0] is None:
                            armed_at[0] = self.clock.now
                        if self.clock.now-armed_at[0] > .6:
                            driver.stop_event.set()
                    if driver.fault:
                        driver.stop_event.set()

                driver.stop_event.callback = schedule
                driver._control_loop()
                self.assertIsNone(driver.fault)
                self.assertTrue(driver.armed)
                self.assertFalse(driver.home_requested)
                self.assertFalse(driver.recovery_active)
                self.assertEqual(tuple(bus.physical.values()), positions)
                driver._disable_all()

    def test_disabled_home_cannot_enable_with_stale_vision(self):
        driver, bus = self.make_driver()
        hub = m.TrackingHub(driver, dict(m.CONFIG), self.clock.monotonic)
        hub.offer_frame([], 640, 480, self.clock.now)
        self.clock.sleep(m.CONFIG["vision_max_age"]+.01)
        with self.assertRaisesRegex(RuntimeError, "Fresh camera/inference"):
            hub.local_action("home")
        self.assertFalse(driver.arm_requested)
        self.assertFalse(driver.home_requested)
        self.assertFalse(any(bus.enabled.values()))

    def test_faster_home_reduces_model_duration(self):
        durations = []
        for rate in ((10., 8.), m.CONFIG["home_rate"]):
            driver, bus = self.make_driver(home_rate=rate)
            self.enable(driver)
            driver.request_home()
            before = self.clock.now
            driver._home_and_clear()
            durations.append(self.clock.now-before)
            driver._disable_all()
        self.assertLess(durations[1], durations[0])

    def test_velocity_stream_reaches_rate_without_repeated_waypoint_braking(self):
        driver, bus = self.make_driver((0., 0.))
        self.enable(driver)
        self.clock.pump = lambda:driver.input(1., 0.)
        driver.phase = "MANUAL"
        before = bus.physical[1]
        for _ in range(60):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance((0., 0.), driver.c["max_rate"])
        self.assertGreater(abs(bus.physical[1]-before), 25.)
        self.assertLessEqual(abs(driver.stream_velocity[0]), driver.c["max_rate"][0])
        self.assertTrue(-150 <= bus.physical[1] <= 150)
        driver._disable_all()

