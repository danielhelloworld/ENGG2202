"""Regression for real-board v1 failure: healthy 150-200ms scheduling gaps."""
import unittest
import test_adapter as adapters
import test_separate_controls as controls
import test_tracker as app

m, fixtures = app.m, adapters.fixtures


class SchedulingTests(unittest.TestCase):
    setUp = controls.SeparateControlTests.setUp
    make_driver = controls.SeparateControlTests.make_driver
    enable = controls.SeparateControlTests.enable

    def test_healthy_manual_control_survives_150_to_200ms_between_queries(self):
        driver,bus = self.make_driver((0., 0.))
        self.enable(driver)
        driver.phase = "MANUAL"
        self.clock.pump = lambda:driver.input(.3, 0.)
        for delay in (.15,.2,.18,.16,.2,.15):
            self.clock.sleep(delay)
            driver._measure()  # Must query before applying any old-age test.
            driver._advance((0.,0.),driver.c["max_rate"])
            self.assertTrue(driver.armed)
            self.assertIsNone(driver.fault)
            self.assertEqual(driver.feedback_missing_since,[None,None])

    def test_motion_gate_refreshes_old_but_healthy_feedback_instead_of_latching_fault(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        self.clock.sleep(.3)
        before = driver.feedback_time
        driver._require_reference(fresh=True)
        self.assertGreater(driver.feedback_time,before)
        self.assertTrue(driver.armed)
        self.assertIsNone(driver.fault)

    def test_initial_auto_home_survives_150ms_worker_scheduling_gaps(self):
        bus = fixtures.MechanicalMotorModel((12.,117.4))
        driver = m.VisualMotorService(dict(m.CONFIG),lambda:bus)
        event = fixtures.Event(self.clock)
        driver.stop_event = event
        original_wait = event.wait
        held = []
        def wait(seconds):
            self.clock.sleep(.13)
            return original_wait(seconds)
        def schedule():
            if driver.phase == "HOME_HOLD":
                held.append(driver.snapshot())
                event.set()
            if driver.phase == "FAULT" or self.clock.now > 40:
                event.set()
        event.wait, event.callback = wait, schedule
        self.clock.pump = lambda:driver.input(0.,0.)
        driver._run()
        self.assertIsNone(driver.fault)
        self.assertTrue(held)
        self.assertTrue(held[0]["startup_completed"])
        self.assertTrue(held[0]["armed"])
        self.assertTrue(all(abs(v) <= .5 for v in held[0]["angles"]))

    def test_25ms_serial_response_is_accepted_and_keeps_initialization_ready(self):
        bus = fixtures.MechanicalMotorModel((12.,117.4))
        driver = m.VisualMotorService(dict(m.CONFIG,startup_auto_home=False),lambda:bus)
        driver.serial, driver.stop_event = bus, fixtures.Event(self.clock)
        original_write, original_read = bus.write,bus.read
        ready_at = [0.]
        def write(data):
            result = original_write(data)
            if data[2] == m.FUNCTION_FEEDBACK:
                ready_at[0] = self.clock.now+.025
            return result
        def read(size,timeout):
            if bus.pending and self.clock.now < ready_at[0]:
                self.clock.sleep(.002)
                return b""
            return original_read(size,timeout)
        bus.write,bus.read = write,read
        driver.sent_target = driver.manual_target = driver._initialize()
        driver.ready,driver.phase = True,"DISABLED"
        self.clock.pump = lambda:driver.input(0.,0.)
        driver.input(0.,0.)
        self.enable(driver)
        self.assertTrue(driver.armed)
        self.assertEqual(driver.angles,(12.,117.4))
        self.assertEqual(driver.feedback_rejects,[0,0])

    def test_arm_outside_range_holds_without_home_and_accepts_small_encoder_settling(self):
        driver,bus = self.make_driver((12.,119.8))
        self.enable(driver)
        self.assertFalse(driver.home_requested)
        self.assertEqual(tuple(bus.physical.values()),(12.,119.8))
        bus.physical[2] += .4
        driver._measure()
        driver._advance(driver.angles,driver.c["max_rate"])
        self.assertTrue(driver.armed)
        self.assertIsNone(driver.fault)

    def test_manual_from_outside_can_only_move_inward_and_release_holds(self):
        driver,bus = self.make_driver((12.,35.))
        self.enable(driver)
        driver.phase = "MANUAL"
        self.clock.pump = lambda:driver.input(0.,-1.)  # Outward mechanical pitch.
        for _ in range(10):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance(driver.angles,driver.c["max_rate"])
        self.assertAlmostEqual(bus.physical[2],35.)
        self.clock.pump = lambda:driver.input(0.,1.)
        for _ in range(35):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance(driver.angles,driver.c["max_rate"])
        self.assertLess(bus.physical[2],30.)
        self.assertIsNone(driver.outside_hold_origin)
        self.clock.pump = lambda:driver.input(0.,0.)
        driver._release_hold()
        self.clock.sleep(.02)
        driver._measure()
        driver._advance(driver.manual_target,driver.c["max_rate"])
        self.assertTrue(driver.armed)
        self.assertFalse(any(p[2] in (8,9,10) for p in bus.writes))

    def test_outside_target_cannot_jump_to_boundary_or_move_farther_outward(self):
        driver,bus = self.make_driver((165.,120.))
        self.enable(driver)
        for angle in (121.,30.,-100.):
            before = len(bus.writes)
            with self.assertRaisesRegex(RuntimeError,"local inward step"):
                driver._write(m.set_single_turn_position(2,angle))
            self.assertEqual(len(bus.writes),before)
        driver._write(m.set_single_turn_position(2,119.))
        self.assertTrue(driver.armed)
        self.assertFalse(driver.home_requested)

    def test_arm_accepts_signed_single_turn_positions_without_replacing_zero(self):
        for pose in ((179.9,-179.9),(-179.9,119.8),(0.,30.1)):
            with self.subTest(pose=pose):
                driver,bus = self.make_driver(pose)
                self.enable(driver)
                self.assertTrue(driver.armed)
                self.assertFalse(driver.home_requested)
                self.assertEqual(tuple(bus.physical.values()),pose)
                self.assertFalse(any(p[2] in (8,9,10) for p in bus.writes))

    def test_release_outside_holds_new_pose_without_chasing_original_arm_pose(self):
        driver,bus = self.make_driver((12.,119.8))
        self.enable(driver)
        driver.phase = "MANUAL"
        self.clock.pump = lambda:driver.input(0.,1.)
        for _ in range(25):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance(driver.angles,driver.c["max_rate"])
        self.assertLess(bus.physical[2],115.)
        self.assertGreater(bus.physical[2],30.)
        self.clock.pump = lambda:driver.input(0.,0.)
        self.clock.sleep(.02)
        driver._measure()
        driver._release_hold()
        driver.phase = "ARMED"
        driver._advance(driver.manual_target,driver.c["max_rate"])
        held = bus.physical[2]
        for _ in range(100):
            self.clock.sleep(.02)
            driver._measure()
            driver._advance(driver.manual_target,driver.c["max_rate"])
        self.assertTrue(driver.armed)
        self.assertAlmostEqual(bus.physical[2],held)
        self.assertFalse(driver.home_requested)

    def test_encoder_settling_outside_does_not_make_outward_key_press_fault(self):
        driver,bus = self.make_driver((12.,119.8))
        self.enable(driver)
        bus.physical[2] = 120.2
        self.clock.pump = lambda:driver.input(0.,-1.)
        self.clock.sleep(.02)
        driver._measure()
        driver.phase = "MANUAL"
        driver._advance(driver.angles,driver.c["max_rate"])
        self.assertAlmostEqual(bus.physical[2],120.2)
        self.assertTrue(driver.armed)

    def test_stop_during_initial_feedback_capture_cannot_be_overwritten_by_auto_home(self):
        bus = fixtures.MechanicalMotorModel((12.,119.8))
        driver = m.VisualMotorService(dict(m.CONFIG),lambda:bus)
        driver.serial, driver.stop_event = bus,fixtures.Event(self.clock)
        original_write = bus.write
        stopped = [False]
        def write(data):
            if data[2] == m.FUNCTION_FEEDBACK and not stopped[0]:
                stopped[0] = True
                driver.request_arm(False)
            return original_write(data)
        bus.write = write
        driver._initialize()
        self.assertTrue(driver.startup_initialized)
        self.assertTrue(driver.startup_cancelled)
        self.assertFalse(driver.arm_requested or driver.home_requested)
        self.assertFalse(any(p[2] in (3,6,8,9,10) for p in bus.writes))

    def test_final_motion_refresh_failure_never_sends_a_cached_position(self):
        driver,bus = self.make_driver((0.,0.))
        self.enable(driver)
        self.clock.sleep(.3)
        bus.drop_feedback = {1}
        count = len([p for p in bus.writes if p[2] == 3])
        with self.assertRaises(RuntimeError):
            driver._write(m.set_single_turn_position(1,1.))
        self.assertEqual(len([p for p in bus.writes if p[2] == 3]),count)
        self.assertFalse(driver.armed)
        self.assertIsNotNone(driver.last_feedback_fault)


if __name__ == "__main__":
    unittest.main()
