"""Regressions for lead saturation and benign rapid operator actions."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m=app.m


class SmoothStreamTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    make_driver=controls.SeparateControlTests.make_driver
    enable=controls.SeparateControlTests.enable

    def test_tracking_lead_saturation_keeps_ramp_running(self):
        d,bus=self.make_driver((0.,0.));self.enable(d)
        d.phase='MANUAL';d.set_response_context('TRACK');bus.stuck=True
        self.clock.pump=lambda:d.input(-1.,0.)
        d.input(-1.,0.)
        for _ in range(35):
            self.clock.sleep(.02);d._measure();d._advance(d.angles,d.c['max_rate'])
            self.assertLessEqual(d.stream_target[0],d.c['tracking_position_lead_deg'][0]+1e-8)
        self.assertGreater(d.stream_velocity[0],50.)
        self.assertTrue(d.armed)

    def test_same_direction_tracking_does_not_retract_on_feedback_retreat(self):
        d,bus=self.make_driver((0.,0.));self.enable(d)
        d.phase='MANUAL';d.set_response_context('TRACK');d.input(-1.,0.)
        self.clock.pump=lambda:d.input(-1.,0.)
        bus.stuck=True
        d._write(m.set_single_turn_position(1,6.))
        d.sent_target=d.stream_target=(6.,0.);d.position_units=[60,0]
        d.stream_velocity=(30.,0.);bus.physical[1]=-.2
        self.clock.sleep(.02);d._measure()
        before=len([f for f in bus.writes if f[2]==3])
        d._advance(d.angles,d.c['max_rate'])
        self.assertEqual(d.stream_target[0],6.)
        self.assertEqual(len([f for f in bus.writes if f[2]==3]),before)
        self.assertGreater(d.stream_velocity[0],30.)

    def test_manual_rate_ignores_diagnostic_error_corrections(self):
        final=[]
        for scale,correction in ((1.,0.),(.8,-3.),(1.15,3.)):
            d,bus=self.make_driver((0.,0.));self.enable(d)
            d.phase='MANUAL';d.set_response_context('MANUAL')
            self.clock.pump=lambda:d.input(-.12,0.)
            d.input(-.12,0.)
            for _ in range(20):
                self.clock.sleep(.02);d._measure()
                d.response.scale=(scale,scale);d.response.correction=(correction,correction)
                d._advance(d.angles,d.c['max_rate'])
            final.append(d.stream_velocity[0])
        self.assertAlmostEqual(min(final),max(final),places=6)


class RapidActionTests(unittest.TestCase):
    setUp=app.HubTests.setUp
    send=app.HubTests.send
    arm=app.HubTests.arm
    def test_duplicate_home_is_rejected_without_disabling(self):
        self.arm();self.send('home')
        with self.assertRaises(m.ControlError):self.send('home')
        self.assertTrue(self.driver.armed)
        self.assertEqual(self.hub.token,self.token)
        self.assertIsNotNone(self.hub.last_control_rejection)
        self.assertTrue(self.send()['armed'])

    def test_button_with_stick_does_not_drop_hold(self):
        self.arm();self.seq+=1
        with self.assertRaises(m.ControlError):
            self.hub.command(dict(token=self.token,seq=self.seq,x=.5,y=0.,action='home'))
        self.assertTrue(self.driver.armed)
        self.assertEqual(self.driver.inputs[-1],(0.,0.))
        self.assertTrue(self.send()['armed'])
