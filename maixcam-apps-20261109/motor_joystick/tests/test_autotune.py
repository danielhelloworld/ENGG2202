"""Independent first-order plant and protocol models, never real hardware."""
import math
import unittest
import test_web
import test_tuning

w = test_web.w


class SearchTests(unittest.TestCase):
    def run_plant(self, cancel=False, drift=False):
        now, position, offset = 1., 0., [0.]
        profile = {"tau": .3}
        applied = []

        def apply(value):
            profile.update(value)
            applied.append(dict(value))

        session = w.AutoTuneExperiment(apply, lambda value: offset.__setitem__(0, value))
        session.start({"tau": .3}, [{"tau": .15}, {"tau": .6}], now)
        cancelled = False
        for _ in range(6000):
            now += .02
            tau = 1.2 if drift and session.kind == "baseline_repeat" else profile["tau"]
            previous = position
            position += (offset[0]-position)*(1-math.exp(-.02/tau))
            session.observe(now, offset[0]-position, (position-previous)/.02)
            if cancel and session.state == "MEASURING" and not cancelled:
                session.cancel(now)
                cancelled = True
            if not session.active:
                break
        return session, applied, offset[0]

    def test_search_verifies_faster_response_before_accepting(self):
        session, applied, offset = self.run_plant()
        self.assertEqual(session.state, "COMPLETE")
        self.assertEqual(session.best, {"tau": .15})
        self.assertEqual(applied[-1], session.best)
        self.assertEqual(offset, 0)
        self.assertEqual([r["kind"] for r in session.results][-2:], ["baseline_repeat", "verify"])

    def test_cancel_recenters_and_restores_the_original_profile(self):
        session, applied, offset = self.run_plant(cancel=True)
        self.assertEqual(session.state, "CANCELLED")
        self.assertEqual(applied[-1], {"tau": .3})
        self.assertFalse(session.restore_pending)
        self.assertEqual(offset, 0)

    def test_baseline_drift_rejects_incomparable_candidate_results(self):
        session, applied, _ = self.run_plant(drift=True)
        self.assertEqual(session.state, "CANCELLED")
        self.assertIn("drift", session.reason)
        self.assertEqual(applied[-1], {"tau": .3})

    def test_feedback_gap_reports_unconfirmed_restore(self):
        session = w.AutoTuneExperiment(lambda p: None, lambda offset: None)
        session.start({"kp": 10}, [{"kp": 12}], 1.)
        session.observe(1.3, 0.)
        self.assertEqual(session.state, "FAILED")
        self.assertTrue(session.restore_pending)


class PiRunnerTests(unittest.TestCase):
    setUp = test_tuning.TuningTests.setUp
    arm = test_tuning.TuningTests.arm
    initialize = test_tuning.TuningTests.initialize
    commands = test_tuning.TuningTests.commands

    def run_until_finished(self, control=None, cancel=False):
        cancelled = False
        for _ in range(6000):
            self.clock.sleep(.02)
            if control:
                control.poll()
            self.s._measure()
            self.s._advance(self.s.manual_target, self.c["max_rate"])
            if cancel and self.s.pi_autotune.session.state == "MEASURING" and not cancelled:
                self.s.cancel_pi_autotune()
                cancelled = True
            if not self.s.autotune_active:
                return
        self.fail("PI experiment did not terminate")

    def test_pi_search_runs_on_existing_owner_and_retains_torque(self):
        self.arm()
        origin = self.s.angles
        self.s.request_pi_autotune(1, 10, 10)
        self.run_until_finished()
        self.assertEqual(self.s.pi_autotune.session.state, "COMPLETE")
        self.assertEqual(self.s.pi_last_sent[1], [10, 10])
        self.assertEqual(self.s.angles, origin)
        self.assertTrue(all(self.bus.enabled.values()))
        self.assertFalse(self.commands(8)+self.commands(9)+self.commands(10))
        self.assertTrue(all(p[1] == 2 for p in self.commands(0x0F)+self.commands(0x10)))

    def test_cancel_restores_baseline_at_test_origin(self):
        self.arm()
        origin = self.s.angles
        self.s.request_pi_autotune(0, 12, 9)
        self.run_until_finished(cancel=True)
        self.assertEqual(self.s.pi_autotune.session.state, "CANCELLED")
        self.assertEqual(self.s.pi_last_sent[0], [12, 9])
        self.assertEqual(self.s.angles, origin)
        self.assertTrue(all(self.bus.enabled.values()))

    def test_browser_timeout_hands_local_experiment_over_without_disarming(self):
        self.arm()
        self.s.startup_waiting_owner = self.s.home_holding = True
        self.s.phase = "HOME_HOLD"
        control = w.BrowserControl(self.s, self.clock.monotonic)
        token = control.connect()["token"]
        control.command(dict(token=token, seq=1, x=0, y=0, action="pi_autotune", axis=1, kp=10, ki=10))
        self.run_until_finished(control)
        self.assertIsNone(control.token)
        self.assertTrue(self.s.startup_waiting_owner)
        self.assertTrue(self.s.armed and self.s.arm_requested)
        self.s.phase = self.s._manual_phase(False)
        self.assertTrue(control.connect()["token"])

    def test_feedback_failure_does_not_claim_success_or_write_saved_parameters(self):
        self.arm()
        self.s.request_pi_autotune(1, 10, 10)
        self.clock.sleep(.02)
        self.s._advance(self.s.manual_target, self.c["max_rate"])
        self.s._record_fault(RuntimeError("feedback missing"))
        report = self.s.pi_autotune.export()
        self.assertFalse(report["active"])
        self.assertEqual(report["state"], "FAILED")
        self.assertTrue(report["restore_pending"])
        self.assertFalse(self.commands(8))


if __name__ == "__main__":
    unittest.main()
