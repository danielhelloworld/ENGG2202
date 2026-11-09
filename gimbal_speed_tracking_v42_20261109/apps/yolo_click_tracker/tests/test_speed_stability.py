"""Noisy-observation controller comparison; synthetic plant, not hardware evidence."""
import math
import unittest
import test_tracker as app
import test_speed_tracking as speed

m=app.m


def simulate_stability(profile, scenario, axis):
    dt=.02
    c=m.CONFIG
    stabilizer=m.SpeedTrackingStabilizer(c)
    position=velocity=command=estimated=previous=0.
    previous_rpm=0
    pending=[0.]*3
    feedback=[0.]*3
    goal=ff=stamp=0.
    samples=[]
    for i in range(300):
        now=i*dt
        target=(20. if scenario=='step' else 12.*min(now,2.) if scenario=='moving' else 0.)
        true_ff=12. if scenario=='moving' and now<2. else 0.
        if i%2==0:
            # 25Hz visual observations with bounded centroid / velocity noise.
            noise=0. if scenario=='step' else .9*math.sin(2*math.pi*9*now)+.3*math.sin(2*math.pi*4*now)
            goal=target+noise
            ff=true_ff+(0. if scenario=='step' else 8.*math.sin(2*math.pi*7*now))
            stamp=now
        measured=feedback[0]
        estimated+=dt/(.03+dt)*((measured-previous)/dt-estimated)
        previous=measured
        if profile=='v40':
            goals,rates=stabilizer.offer((goal,goal),(ff,ff),(measured,measured),stamp)
            q,target_rate=goals[axis],rates[axis]
        else:q,target_rate=goal,ff
        error=q-measured
        request=m.speed_tracking_request(c,axis,error,estimated,target_rate,True,.04)
        accel=c['speed_tracking_accel_deg_s2'][axis]
        if profile=='v40':
            near=1.-m.clip(abs(error)/c['speed_tracking_smooth_radius_deg'],0.,1.)
            accel*=1.-near*(1.-c['speed_tracking_near_accel_ratio'])
        command=m.slew_tracking_speed(command,request,dt,accel,c['speed_tracking_brake_deg_s2'][axis])
        rpm=(m.stable_speed_rpm(c,axis,command,previous_rpm) if profile=='v40' else
             m.logical_rate_to_speed_rpm(c,axis,command))
        previous_rpm=rpm
        rate=m.speed_rpm_to_logical_rate(c,axis,rpm)
        pending.append(rate);request=pending.pop(0)
        drive=c['speed_tracking_driver_accel'][axis]*6.
        velocity+=m.clip(request-velocity,-drive*dt,drive*dt)
        position+=velocity*dt
        feedback.append(position);feedback.pop(0)
        samples.append((now,position,target,rpm,velocity))
    tail=[s for s in samples if s[0]>=3.]
    return dict(
        peak_to_peak_deg=max(s[1] for s in tail)-min(s[1] for s in tail),
        rpm_switches=sum(a[3]!=b[3] for a,b in zip(tail,tail[1:])),
        rms_velocity_deg_s=math.sqrt(sum(s[4]**2 for s in tail)/len(tail)),
        mean_error_deg=sum(abs(s[1]-s[2]) for s in samples)/len(samples),
        rise_s=next((s[0] for s in samples if scenario=='step' and abs(s[1]-s[2])<2.5),None),
        final_error_deg=abs(samples[-1][1]-samples[-1][2]))


class SpeedStabilityTests(unittest.TestCase):
    def test_stationary_noisy_target_reduces_motion_and_rpm_switching(self):
        for axis in (0,1):
            old,new=simulate_stability('v39','stationary',axis),simulate_stability('v40','stationary',axis)
            self.assertLess(new['rpm_switches'],old['rpm_switches']*.75)
            self.assertLess(new['rms_velocity_deg_s'],old['rms_velocity_deg_s']*.8)
            self.assertLessEqual(new['peak_to_peak_deg'],old['peak_to_peak_deg'])

    def test_large_step_retains_fast_response(self):
        for axis in (0,1):
            old,new=simulate_stability('v39','step',axis),simulate_stability('v40','step',axis)
            self.assertLessEqual(new['rise_s'],old['rise_s']+.12)
            self.assertLess(new['final_error_deg'],2.5)

    def test_moving_target_stays_close_while_settled_motion_is_quieter(self):
        for axis in (0,1):
            old,new=simulate_stability('v39','moving',axis),simulate_stability('v40','moving',axis)
            self.assertLessEqual(new['mean_error_deg'],old['mean_error_deg']+.5)
            self.assertLess(new['rms_velocity_deg_s'],old['rms_velocity_deg_s'])

    def test_repeated_observation_is_not_filtered_multiple_times(self):
        f=m.SpeedTrackingStabilizer(m.CONFIG)
        f.offer((0.,0.),(0.,0.),(0.,0.),1.)
        first=f.offer((1.,1.),(4.,4.),(0.,0.),1.04)
        self.assertEqual(first,f.offer((1.,1.),(4.,4.),(.2,.2),1.04))

    def test_far_target_bypasses_smoothing_and_reversal_drops_old_feedforward(self):
        f=m.SpeedTrackingStabilizer(m.CONFIG)
        f.offer((0.,0.),(8.,8.),(0.,0.),1.)
        q,v=f.offer((20.,-20.),(30.,-30.),(0.,0.),1.04)
        self.assertEqual(q,(20.,-20.));self.assertEqual(v,(30.,-30.))
        f.offer((0.,0.),(8.,8.),(0.,0.),2.)
        q,v=f.offer((-.5,-.5),(-8.,-8.),(0.,0.),2.04)
        self.assertTrue(all(rate<0 for rate in v))

    def test_quantizer_reduces_boundary_toggle_without_exceeding_speed_bounds(self):
        c=m.CONFIG
        previous=0
        outputs=[]
        for rate in (5.9,6.1,5.8,6.2,5.9,6.3):
            previous=m.stable_speed_rpm(c,0,rate,previous);outputs.append(previous)
        self.assertEqual(outputs,[0]*6)
        self.assertEqual(m.stable_speed_rpm(c,0,7.2,0),-1)
        self.assertEqual(m.stable_speed_rpm(c,0,6.1,-1),-1)
        self.assertEqual(m.stable_speed_rpm(c,0,5.9,-1),0)
        self.assertEqual(m.stable_speed_rpm(c,0,-20.,-1),0)
        previous=0
        for rate in (6.,12.,18.,24.,30.):
            previous=m.stable_speed_rpm(c,0,rate,previous)
        self.assertEqual(previous,-5)  # Whole-RPM acceleration steps must not stall.
        self.assertEqual(m.stable_speed_rpm(c,0,240.,-39),-40)
        for rate in range(-240,241):
            for previous in (-10,-1,0,1,10):
                rpm=m.stable_speed_rpm(c,0,rate,previous)
                self.assertLessEqual(abs(rpm*6),abs(rate))
                self.assertGreaterEqual(m.speed_rpm_to_logical_rate(c,0,rpm)*rate,0.)

    def test_invalid_smoothing_profiles_rejected(self):
        for key,value in (('speed_tracking_goal_tau_s',-.1),('speed_tracking_feedforward_tau_s',.5),
                          ('speed_tracking_smooth_radius_deg',0.),('speed_tracking_near_accel_ratio',.1),
                          ('speed_tracking_rpm_hysteresis',float('nan'))):
            with self.assertRaises(ValueError):m.validate_visual_config(dict(m.CONFIG,**{key:value}))


class SpeedStabilityStopTests(unittest.TestCase):
    setUp=speed.SpeedTrackingTests.setUp
    enable=speed.SpeedTrackingTests.enable
    driver=speed.SpeedTrackingTests.driver
    aim=speed.SpeedTrackingTests.aim
    tick=speed.SpeedTrackingTests.tick

    def test_stop_resets_filter_and_bypasses_rpm_hysteresis(self):
        d,bus=self.driver()
        for _ in range(8):self.aim(d)
        self.assertIsNotNone(d.native_tracker.stabilizer.goal)
        d.native_tracker.stop()
        self.assertEqual(bus.rates,{1:0,2:0})
        self.assertIsNone(d.native_tracker.stabilizer.goal)
        self.assertEqual(d.native_tracker.command_rpm,[0,0])


if __name__=='__main__':unittest.main()
