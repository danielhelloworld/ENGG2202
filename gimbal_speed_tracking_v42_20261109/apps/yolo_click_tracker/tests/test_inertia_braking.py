"""Trajectory braking contracts with synthetic encoder feedback, not inertia identification."""
import unittest
import test_tracker as app
import test_separate_controls as controls
import test_fast_manual as fast

m=app.m


class InertiaBrakingTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    make_driver=controls.SeparateControlTests.make_driver
    enable=controls.SeparateControlTests.enable
    move=fast.FastManualTests.move

    def release(self,d,context='HOLD'):
        d.was_moving=True
        self.clock.pump=lambda:d.input(0.,0.)
        d.input(0.,0.);d.set_response_context(context);d._release_hold()
        d.phase=d._manual_phase(False)

    def tick(self,d,dt=.02):
        self.clock.sleep(dt);d._measure();d._advance(d.angles,d.c['max_rate'])

    def test_each_axis_and_direction_brakes_without_retracting_or_disabling(self):
        for vector in ((-.2,0.),(.2,0.),(0.,-.2),(0.,.2)):
            with self.subTest(vector=vector):
                d,bus=self.make_driver((0.,0.));self.enable(d)
                self.move(d,vector,count=20)
                axis=0 if vector[0] else 1
                direction=1 if d.stream_velocity[axis]>0 else -1
                previous=d.sent_target[axis]
                self.release(d)
                self.assertTrue(d.braking)
                started=self.clock.now
                while d.braking and self.clock.now-started<2.:
                    self.tick(d)
                    self.assertGreaterEqual((d.sent_target[axis]-previous)*direction,-1e-8)
                    self.assertGreaterEqual(d.stream_velocity[axis]*direction,0.)
                    previous=d.sent_target[axis]
                self.assertTrue(d.soft_hold)
                self.assertLess(self.clock.now-started,d.c['brake_timeout_s'])
                landing=d.sent_target
                for _ in range(10):self.tick(d)
                self.assertEqual(d.sent_target,landing)
                self.assertTrue(d.armed);self.assertIsNone(d.fault)
                d._disable_all()

    def test_full_rate_stop_completes_before_timeout_and_keeps_targets_in_limits(self):
        d,bus=self.make_driver((0.,0.));self.enable(d)
        self.move(d,(-1.,0.),count=15)
        # Isolate a full-rate release at the center without travelling to a limit first.
        d.stream_velocity=(240.,0.);d.manual_ramp.acceleration=[400.,0.]
        d.response.velocity=(240.,0.)
        self.release(d);started=self.clock.now
        while d.braking and self.clock.now-started<2.:
            self.tick(d)
            self.assertTrue(all(lo<=q<=hi for q,(lo,hi) in zip(d.sent_target,d.c['limits'])))
        self.assertTrue(d.soft_hold)
        self.assertLess(self.clock.now-started,d.c['brake_timeout_s'])

    def test_tracking_loss_uses_tracking_speed_and_lead_during_brake(self):
        d,bus=self.make_driver((0.,0.));self.enable(d)
        d.phase='MANUAL';d.set_response_context('TRACK')
        self.clock.pump=lambda:d.input(-.4,0.)
        for _ in range(20):self.tick(d)
        self.release(d,'LOST')
        self.assertTrue(d.braking);self.assertEqual(d.release_source,'TRACK')
        self.assertEqual(d._position_lead_deg(0),d.c['tracking_position_lead_deg'][0])
        for _ in range(60):self.tick(d)
        self.assertTrue(d.armed);self.assertTrue(d.soft_hold)

    def test_home_and_explicit_disarm_interrupt_braking(self):
        for action in ('home','disarm'):
            with self.subTest(action=action):
                d,bus=self.make_driver((0.,0.));self.enable(d)
                self.move(d,(-.2,0.),count=20);self.release(d)
                self.assertEqual(d.phase,'BRAKING')
                if action=='home':
                    d.request_home();self.assertTrue(d.home_requested)
                else:
                    d._disable_now();self.assertFalse(any(bus.enabled.values()))
                self.assertFalse(d.braking or d.soft_hold)

    def test_new_input_resumes_without_reversing_the_old_velocity_in_one_tick(self):
        d,bus=self.make_driver((0.,0.));self.enable(d)
        self.move(d,(-.3,0.),count=20);self.release(d)
        self.tick(d);v=d.stream_velocity[0]
        d.set_response_context('MANUAL');d.input(.3,0.)
        d.phase='MANUAL';self.clock.pump=lambda:d.input(.3,0.)
        self.assertFalse(d.braking or d.soft_hold)
        self.tick(d)
        self.assertGreater(d.stream_velocity[0]*v,0.)
        self.assertTrue(d.armed)
