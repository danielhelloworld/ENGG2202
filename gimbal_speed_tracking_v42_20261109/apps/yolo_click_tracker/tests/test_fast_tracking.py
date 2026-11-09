"""Fast-motion pipeline comparison against the saved v40 standalone entry."""
import importlib.util
import math
from pathlib import Path
import unittest
import test_tracker as app
import test_speed_tracking as speed

m=app.m
spec=importlib.util.spec_from_file_location('fast_motion_v40_baseline',Path(__file__).resolve().parents[1]/'backups/speed-track-v40/main.py')
baseline=importlib.util.module_from_spec(spec);spec.loader.exec_module(baseline)


def fast_pipeline(module,axis=0,rate=110.,noise=False):
    c=dict(module.CONFIG)
    tracker=module.ClickTracker(c)
    track=dict(key=(0,1),box=(250,180,80,80),cx=.5,cy=.5,score=.9)
    tracker.select((0,1),[track],0.)
    smoother=module.SpeedTrackingStabilizer(c)
    dt=.02;pos=vel=command=estimated=previous=0.;rpm=0
    feedback=[0.]*2;pending=[0.]*2;frames=[];mailbox=None;samples=[];lost=0
    for i in range(150):
        now=i*dt
        truth=rate*min(now,1.)
        actual=feedback[0]
        estimated+=dt/(.03+dt)*((actual-previous)/dt-estimated);previous=actual
        if i%2==0:
            jitter=(.9*math.sin(2*math.pi*9*now)+.3*math.sin(2*math.pi*4*now)) if noise else 0.
            ray=(truth+jitter-pos)/(c['joystick_signs'][axis]*c['camera_fov'][axis])
            tr=dict(track)
            if axis==0:tr['cx']=.5+ray
            else:tr['cy']=.5-ray
            pose=[0.,0.];pose[axis]=pos
            frames.append((i+3,now,tr,tuple(pose),abs(ray)<=.5))
        if frames and frames[0][0]<=i:
            _,stamp,tr,pose,visible=frames.pop(0)
            current=[0.,0.];current[axis]=actual
            tracker.step([tr] if visible else [],stamp,pose,latency=.06,current_pose=tuple(current))
            if not visible:lost+=1
            if tracker.aim_goal is not None:
                mailbox=(tracker.aim_goal,tracker.aim_velocity,stamp,now,tracker.native_active_axes)
            else:mailbox=None
        if mailbox:
            goal,ff,stamp,offered,active=mailbox
            pose=[0.,0.];pose[axis]=actual
            goals,rates=smoother.offer(goal,ff,pose,stamp)
            q=goals[axis];f=rates[axis]
            if 'speed_tracking_mailbox_lead_s' in c:q+=f*min(now-offered,c['speed_tracking_mailbox_lead_s'])
            error=q-actual
            request=module.speed_tracking_request(c,axis,error,estimated,f,active[axis],.02)
            near=1.-module.clip(abs(error)/c['speed_tracking_smooth_radius_deg'],0.,1.)
            motion=module.tracking_motion_weight(c,f) if hasattr(module,'tracking_motion_weight') else 0.
            near*=1.-motion
            base=c['speed_tracking_accel_deg_s2'][axis]
            fast=c.get('speed_tracking_fast_accel_deg_s2',c['speed_tracking_accel_deg_s2'])[axis]
            accel=(base+(fast-base)*motion)*(1.-near*(1.-c['speed_tracking_near_accel_ratio']))
            command=module.slew_tracking_speed(command,request,dt,accel,c['speed_tracking_brake_deg_s2'][axis])
            rpm=module.stable_speed_rpm(c,axis,command,rpm)
        else:command=0.;rpm=0
        pending.append(module.speed_rpm_to_logical_rate(c,axis,rpm));desired=pending.pop(0)
        drive=c['speed_tracking_driver_accel'][axis]*6.
        vel+=module.clip(desired-vel,-drive*dt,drive*dt);pos+=vel*dt
        feedback.append(pos);feedback.pop(0)
        samples.append((now,abs(truth-pos),pos,truth,vel))
    chase=[s for s in samples if .3<=s[0]<=1.]
    tail=[s for s in samples if s[0]>=2.]
    return dict(chase_mean_error_deg=sum(s[1] for s in chase)/len(chase),
                chase_max_error_deg=max(s[1] for s in chase),lost_frames=lost,
                final_error_deg=samples[-1][1],
                stop_overshoot_deg=max(max(0.,(s[2]-s[3])*(1 if rate>=0 else -1)) for s in samples if s[0]>=1.),
                settled_rms_velocity_deg_s=math.sqrt(sum(s[4]**2 for s in tail)/len(tail)),
                settled_peak_to_peak_deg=max(s[2] for s in tail)-min(s[2] for s in tail))


class FastMotionTests(unittest.TestCase):
    def test_fast_target_estimate_exceeds_old_normalized_velocity_cap(self):
        for axis,rate in ((0,110.),(1,80.)):
            p=m.FastMotionPredictor(m.CONFIG)
            for i in range(8):
                e=[0.,0.];e[axis]=rate*(i*.04)/(m.CONFIG['camera_fov'][axis])
                p.observe(tuple(e),(0.,0.),1.+i*.04)
            self.assertGreater(p.motion_velocity[axis]*m.CONFIG['camera_fov'][axis],rate*.85)
            self.assertTrue(p.diagnostics()['fast_axes'][axis])
            self.assertGreater(abs(p.lead((0.,0.),.06)[axis]),m.CONFIG['prediction_max_shift'])

    def test_single_fast_noise_sample_does_not_enable_fast_prediction(self):
        p=m.FastMotionPredictor(m.CONFIG)
        p.observe((0.,0.),(0.,0.),1.)
        p.observe((.04,.04),(0.,0.),1.04)
        self.assertEqual(p.fast_counts,[1,1])
        p.observe((0.,0.),(0.,0.),1.08)
        self.assertEqual(p.fast_counts,[1,1])

    def test_fast_predictor_preserves_own_motion_compensation_and_reversal(self):
        p=m.FastMotionPredictor(m.CONFIG)
        for i in range(10):p.observe((i*4./87.,0.),(i*4.,0.),1.+i*.04)
        self.assertEqual(p.motion_velocity,(0.,0.))
        for i in range(5):p.observe((.1+i*.04,0.),(0.,0.),2.+i*.04)
        p.observe((.22,0.),(0.,0.),2.20)
        self.assertLess(p.motion_velocity[0],0.)

    def test_pipeline_fast_chase_reduces_error_without_losing_frames(self):
        for axis,rate in ((0,110.),(1,-75.)):
            old,new=fast_pipeline(baseline,axis,rate),fast_pipeline(m,axis,rate)
            self.assertLess(new['chase_mean_error_deg'],old['chase_mean_error_deg']*.85)
            self.assertEqual(new['lost_frames'],0)
            self.assertLess(new['final_error_deg'],3.)

    def test_fast_stop_stays_within_delayed_braking_model_envelope(self):
        for axis,rate in ((0,110.),(1,-75.)):
            result=fast_pipeline(m,axis,rate)
            envelope=abs(rate)*.1+rate*rate/(2*m.CONFIG['speed_tracking_brake_deg_s2'][axis])+3.
            self.assertLess(result['stop_overshoot_deg'],envelope)
            self.assertLess(result['final_error_deg'],3.)

    def test_full_visual_predictor_does_not_worsen_stationary_noise(self):
        for axis in (0,1):
            before,after=fast_pipeline(baseline,axis,0.,True),fast_pipeline(m,axis,0.,True)
            self.assertLessEqual(after['settled_rms_velocity_deg_s'],before['settled_rms_velocity_deg_s']+.1)
            self.assertLessEqual(after['settled_peak_to_peak_deg'],before['settled_peak_to_peak_deg']+.1)

    def test_fast_sample_period_and_rate_ceiling_remain_bounded(self):
        p=m.FastMotionPredictor(m.CONFIG)
        for i in range(10):p.observe((i*.04,i*.04),(0.,0.),1.+i*.01)
        self.assertTrue(p.ready)
        self.assertTrue(all(v>0 for v in p.rate_limit_hits))
        self.assertTrue(all(abs(v)*f<=cap for v,f,cap in zip(p.motion_velocity,m.CONFIG['camera_fov'],m.CONFIG['speed_prediction_rate_deg_s'])))
        p.observe((.41,.41),(0.,0.),1.1,score=.5)
        self.assertFalse(p.ready)
        self.assertEqual(p.fast_counts,[0,0])

    def test_fast_catchup_is_bounded_and_stop_drops_old_velocity(self):
        for axis in (0,1):
            result=m.speed_tracking_request(m.CONFIG,axis,20.,100.,100.,True)
            self.assertLessEqual(result-100.,m.CONFIG['speed_tracking_catchup_deg_s'][axis])
        p=m.FastMotionPredictor(m.CONFIG)
        for i in range(8):p.observe((i*.04,0.),(0.,0.),1.+i*.04)
        self.assertTrue(p.fast_counts[0]>=2)
        p.observe((7*.04,0.),(0.,0.),1.32)
        self.assertEqual(p.motion_velocity[0],0.)

    def test_fast_target_bypasses_near_filter_and_launch_slowdown(self):
        f=m.SpeedTrackingStabilizer(m.CONFIG)
        for i in range(5):q,v=f.offer((i*.1,0.),(100.,0.),(0.,0.),1.+i*.04)
        self.assertAlmostEqual(q[0],.4)
        self.assertAlmostEqual(v[0],100.)
        self.assertEqual(m.tracking_motion_weight(m.CONFIG,100.),1.)
        self.assertEqual(m.tracking_motion_weight(m.CONFIG,8.),0.)

    def test_invalid_fast_prediction_settings_rejected(self):
        for values in ({'speed_prediction_rate_deg_s':(241.,140.)},
                       {'speed_prediction_fast_deg_s':(180.,140.)},
                       {'speed_prediction_fast_alpha':.99},
                       {'speed_prediction_max_shift':.5},
                       {'speed_tracking_motion_band_deg_s':(60.,18.)},
                       {'speed_tracking_mailbox_lead_s':.5}):
            with self.assertRaises(ValueError):m.validate_visual_config(dict(m.CONFIG,**values))


class FastMotionDriverTests(unittest.TestCase):
    setUp=speed.SpeedTrackingTests.setUp
    enable=speed.SpeedTrackingTests.enable
    driver=speed.SpeedTrackingTests.driver
    aim=speed.SpeedTrackingTests.aim
    tick=speed.SpeedTrackingTests.tick

    def test_motor_mailbox_extrapolation_is_bounded_and_reported(self):
        d,bus=self.driver()
        for _ in range(8):self.aim(d,(20.,0.),velocity=(100.,0.))
        diag=d.snapshot()['speed_tracking_diagnostics']
        self.assertGreater(diag['mailbox_lead_ms'],0.)
        self.assertLessEqual(diag['mailbox_lead_ms'],40.)
        self.assertEqual(len(diag['axes']),2)
        self.assertGreater(diag['axes'][0]['target_velocity_deg_s'],50.)
        self.assertIn('speed_command_interval',d.timings.snapshot())
        d.native_tracker.stop()
        self.assertEqual(d.native_tracker.diagnostics,{})


if __name__=='__main__':unittest.main()
