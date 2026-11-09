"""Isolated speed-loop model with feedback lag and command transport lag.

The plant uses a bounded acceleration, no inertia/load identification. Results
compare controller profiles, and are not measurements of the physical gimbal.
"""
import unittest
import test_tracker as app
import test_speed_tracking as speed

m=app.m


def simulate(profile, scenario, axis):
    dt=.02
    position=velocity=command=estimated=previous=0.
    pending=[0.]*3  # 60ms command delay.
    feedback=[0.]*3  # 40ms-old pose at the start of each step.
    samples=[]
    for i in range(250):
        stamp=i*dt
        target=(20. if scenario=='step' else
                30.*min(stamp,1.) if scenario=='stop' else
                20. if stamp<1. else -20.)
        ff=30. if scenario=='stop' and stamp<1. else 0.
        measured=feedback[0]
        estimated+=dt/(.03+dt)*((measured-previous)/dt-estimated)
        previous=measured
        error=target-measured
        if profile=='v38':
            requested=max(-240.,min(240.,(2.4,2.6)[axis]*error+ff-.15*(estimated-ff)))
            command+=max(-60.*dt,min(60.*dt,requested-command))
            drive_accel=60.
        else:
            requested=m.speed_tracking_request(m.CONFIG,axis,error,estimated,ff,True,.04)
            command=m.slew_tracking_speed(command,requested,dt,
                m.CONFIG['speed_tracking_accel_deg_s2'][axis],m.CONFIG['speed_tracking_brake_deg_s2'][axis])
            drive_accel=m.CONFIG['speed_tracking_driver_accel'][axis]*6.
        pending.append(int(command/6)*6)
        requested=pending.pop(0)
        velocity+=max(-drive_accel*dt,min(drive_accel*dt,requested-velocity))
        position+=velocity*dt
        feedback.append(position);feedback.pop(0)
        samples.append((stamp,position,target))
    return dict(
        rise_s=next((t for t,p,q in samples if scenario=='step' and abs(p-q)<2.5),None),
        stop_overshoot_deg=max(max(0.,p-q) for t,p,q in samples if t>=1.),
        mean_error_deg=sum(abs(p-q) for t,p,q in samples)/len(samples),
        final_error_deg=abs(samples[-1][1]-samples[-1][2]))


class SpeedResponseTests(unittest.TestCase):
    def test_step_reaches_target_band_faster_without_extra_overshoot(self):
        for axis in (0,1):
            before,after=simulate('v38','step',axis),simulate('v39','step',axis)
            self.assertLess(after['rise_s'],before['rise_s']*.8)
            self.assertLessEqual(after['stop_overshoot_deg'],.5)

    def test_moving_target_stop_brakes_instead_of_coasting_across_target(self):
        for axis in (0,1):
            before,after=simulate('v38','stop',axis),simulate('v39','stop',axis)
            self.assertLess(after['stop_overshoot_deg'],before['stop_overshoot_deg']*.5)
            self.assertLess(after['stop_overshoot_deg'],4.)
            self.assertLess(after['final_error_deg'],2.)

    def test_target_reversal_reduces_error_without_sustained_oscillation(self):
        for axis in (0,1):
            before,after=simulate('v38','reverse',axis),simulate('v39','reverse',axis)
            self.assertLess(after['mean_error_deg'],before['mean_error_deg']*.8)
            self.assertLess(after['final_error_deg'],2.5)

    def test_measured_closing_speed_triggers_braking_before_crossing(self):
        for axis in (0,1):
            self.assertEqual(m.speed_tracking_request(m.CONFIG,axis,1.,80.,0.,True),0.)
            self.assertEqual(m.speed_tracking_request(m.CONFIG,axis,-1.,-80.,0.,True),0.)
            # Moving target at image center must retain matched angular velocity.
            self.assertEqual(m.speed_tracking_request(m.CONFIG,axis,0.,30.,30.,True),30.)

    def test_reversal_brakes_to_zero_before_launching_the_other_direction(self):
        self.assertAlmostEqual(m.slew_tracking_speed(20.,-50.,.02,100.,400.),12.)
        self.assertAlmostEqual(m.slew_tracking_speed(4.,-50.,.02,100.,400.),-1.)
        self.assertAlmostEqual(m.slew_tracking_speed(0.,50.,.02,100.,400.),2.)

    def test_invalid_launch_and_brake_profiles_rejected(self):
        for settings in ({'speed_tracking_accel_deg_s2': (0.,200.)},
                         {'speed_tracking_driver_accel': (0,60)},
                         {'speed_tracking_driver_accel': (70.5,60)},
                         {'speed_tracking_brake_deg_s2': (100.,100.)},
                         {'speed_tracking_accel_deg_s2': (500.,240.)},
                         {'speed_tracking_brake_latency_s': float('nan')}):
            with self.assertRaises(ValueError):m.validate_visual_config(dict(m.CONFIG,**settings))


class SpeedDriverProfileTests(unittest.TestCase):
    setUp=speed.SpeedTrackingTests.setUp
    enable=speed.SpeedTrackingTests.enable
    driver=speed.SpeedTrackingTests.driver
    aim=speed.SpeedTrackingTests.aim
    tick=speed.SpeedTrackingTests.tick

    def test_tracking_programs_its_own_acceleration_and_restores_shared_value(self):
        d,bus=self.driver()
        before=len(bus.writes)
        self.aim(d)
        packets=[p for p in bus.writes[before:] if p[2]==m.FUNCTION_ACCELERATION]
        self.assertEqual([(p[1],int.from_bytes(p[3:5],'big')) for p in packets],[(1,70),(2,60)])
        d.set_response_context('MANUAL')
        for _ in range(25):self.tick(d)
        packets=[p for p in bus.writes if p[2]==m.FUNCTION_ACCELERATION]
        self.assertEqual([int.from_bytes(p[3:5],'big') for p in packets[-2:]],
                         [d.c['motor_acceleration']]*2)
        self.assertFalse(any(d.native_tracker.engaged))


if __name__=='__main__':unittest.main()
