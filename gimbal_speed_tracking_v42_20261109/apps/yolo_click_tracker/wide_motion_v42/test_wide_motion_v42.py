"""Standalone v42 tests; all previous source files remain untouched."""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parent
APP=ROOT.parent
sys.path.insert(0,str(APP/'tests'))
import test_tracker as previous
import test_adapter as adapters
from native_model import make_native_model
spec=importlib.util.spec_from_file_location('wide_motion_v42_bundle',APP/'main_wide_motion_v42.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
old=previous.m
fixtures=adapters.fixtures


def person(bearing=0.,pose=0.,key=(0,1),score=.9):
    return dict(key=key,box=(250,160,80,120),cx=.5+(bearing-pose)/(m.CONFIG['joystick_signs'][0]*87.),
                cy=.5,score=score,label='person',trail=[])


def seeded_core(module=m,velocity=0.,start=0.):
    core=module.ClickTracker(dict(module.CONFIG))
    core.select((0,1),[person()],1.)
    for i in range(9):
        stamp=1.+i*.04;bearing=start+velocity*(stamp-1.)
        core.step([person(bearing,bearing-2.)],stamp,(bearing-2.,0.),latency=.06,current_pose=(bearing-2.,0.))
    return core


def pipeline(module,rate=75.,travel_s=2.,start=0.,noise=False):
    c=dict(module.CONFIG);core=module.ClickTracker(c)
    core.select((0,1),[person(start,start)],0.)
    filter=module.SpeedTrackingStabilizer(c)
    dt=.02;pos=start;v=command=estimated=0.;previous=start;rpm=0
    feedback=[start]*2;pending=[0.]*2;frames=[];message=None;rows=[];lost=homes=0
    for i in range(300):
        now=i*dt;truth=start+rate*min(now,travel_s);measured=feedback[0]
        estimated+=dt/(.03+dt)*((measured-previous)/dt-estimated);previous=measured
        if i%2==0:
            jitter=(.8*math.sin(2*math.pi*9*now)) if noise else 0.
            tr=person(truth+jitter,pos)
            frames.append((i+3,now,tr,pos,0<=tr['cx']<=1))
        if frames and frames[0][0]<=i:
            _,stamp,tr,pose,visible=frames.pop(0)
            result=core.step([tr] if visible else [],stamp,(pose,0.),latency=.06,current_pose=(measured,0.))
            if result=='home':homes+=1
            if not visible:lost+=1
            message=(core.aim_goal,core.aim_velocity,stamp,now,core.native_active_axes) if core.aim_goal else None
        core.drive(now,(measured,0.))
        if message and core.aim_goal is not None:
            goal,ff,stamp,offered,active=message
            if core.state=='COAST':goal,ff=core.aim_goal,core.aim_velocity
            goals,rates=filter.offer(goal,ff,(measured,0.),stamp)
            q=goals[0]+rates[0]*min(max(0.,now-offered),c['speed_tracking_mailbox_lead_s'])
            error=q-measured;f=rates[0]
            request=module.speed_tracking_request(c,0,error,estimated,f,active[0],.02)
            motion=module.tracking_motion_weight(c,f)
            near=(1.-module.clip(abs(error)/c['speed_tracking_smooth_radius_deg'],0.,1.))*(1.-motion)
            base=c['speed_tracking_accel_deg_s2'][0];fast=c['speed_tracking_fast_accel_deg_s2'][0]
            accel=(base+(fast-base)*motion)*(1.-near*(1.-c['speed_tracking_near_accel_ratio']))
            command=module.slew_tracking_speed(command,request,dt,accel,c['speed_tracking_brake_deg_s2'][0])
            rpm=module.stable_speed_rpm(c,0,command,rpm)
        else:command=0.;rpm=0
        pending.append(module.speed_rpm_to_logical_rate(c,0,rpm));desired=pending.pop(0)
        drive=c['speed_tracking_driver_accel'][0]*6.
        v+=module.clip(desired-v,-drive*dt,drive*dt);pos+=v*dt
        feedback.append(pos);feedback.pop(0)
        rows.append((now,pos,truth,v))
    chase=[r for r in rows if .3<=r[0]<=travel_s]
    tail=[r for r in rows if r[0]>=travel_s+2.]
    return dict(total_target_travel_deg=rate*travel_s,actual_travel_deg=pos-start,
        chase_mean_error_deg=sum(abs(r[1]-r[2]) for r in chase)/len(chase),
        chase_max_error_deg=max(abs(r[1]-r[2]) for r in chase),
        stop_overshoot_deg=max(max(0.,(r[1]-r[2])*(1 if rate>=0 else -1)) for r in rows if r[0]>=travel_s),
        final_error_deg=abs(pos-rows[-1][2]),lost_frames=lost,home_requests=homes,
        settled_peak_to_peak_deg=max(r[1] for r in tail)-min(r[1] for r in tail))


class WideControllerTests(unittest.TestCase):
    def test_small_error_keeps_previous_fine_tracking_law(self):
        for error in (-2.,-.5,0.,.5,2.):
            for ff in (0.,12.,80.):
                for velocity in (-20.,0.,80.):
                    for axis in (0,1):
                        self.assertEqual(m.speed_tracking_request(m.CONFIG,axis,error,velocity,ff,True),
                                         old.speed_tracking_request(old.CONFIG,axis,error,velocity,ff,True))

    def test_large_error_can_use_more_capture_speed_without_exceeding_ceiling(self):
        for error,ff in ((25.,90.),(-25.,-90.)):
            current=m.speed_tracking_request(m.CONFIG,0,error,ff,ff,True)
            previous=old.speed_tracking_request(old.CONFIG,0,error,ff,ff,True)
            self.assertGreater(abs(current),abs(previous)+30.)
            self.assertLessEqual(abs(current),m.CONFIG['motor_rpm']*6)

    def test_large_moving_reference_filters_noise_without_adding_constant_velocity_lag(self):
        f=m.SpeedTrackingStabilizer(m.CONFIG)
        raw=[];filtered=[]
        for i in range(40):
            stamp=1.+i*.04;truth=20.+60.*i*.04
            noisy=truth+(.5 if i%2 else -.5)
            q,_=f.offer((noisy,0.),(60.,0.),(truth-20.,0.),stamp)
            raw.append(abs(noisy-truth));filtered.append(abs(q[0]-truth))
        self.assertLess(sum(filtered[10:]),sum(raw[10:])*.6)

    def test_coast_extends_bounded_window_without_home(self):
        c=seeded_core(velocity=80.,start=170.)
        last=c.last_seen;goal=c.last_tracked_goal
        self.assertIsNone(c.step([],last+.20,(190.,0.),current_pose=(190.,0.)))
        self.assertEqual(c.state,'COAST')
        self.assertGreater(c.aim_goal[0],180.)
        self.assertLessEqual(abs(c.aim_goal[0]-goal[0]),24.)
        c.drive(last+.36,(195.,0.))
        self.assertEqual(c.state,'LOST');self.assertIsNone(c.aim_goal)
        self.assertIsNone(c.step([],last+2.1,(195.,0.),current_pose=(195.,0.)))
        self.assertIsNone(c.selected);self.assertEqual(c.state,'LOST')
        self.assertIn('no automatic HOME',c.loss_message)

    def test_stationary_or_unreliable_loss_never_starts_blind_continuation(self):
        c=seeded_core();last=c.last_seen
        c.step([],last+.04,(0.,0.),current_pose=(0.,0.))
        self.assertEqual(c.state,'LOST');self.assertIsNone(c.aim_goal)
        c=seeded_core(velocity=80.)
        c.last_measurement_score=.4
        c.step([],c.last_seen+.04,(0.,0.),current_pose=(0.,0.))
        self.assertEqual(c.state,'LOST')

    def test_new_id_requires_two_unique_geometrically_consistent_frames(self):
        c=seeded_core();last=c.last_seen
        tr=person(0.,-2.,key=(0,8))
        c.step([tr],last+.04,(-2.,0.),current_pose=(-2.,0.))
        self.assertEqual(c.selected,(0,1))
        c.step([tr],last+.08,(-2.,0.),current_pose=(-2.,0.))
        self.assertEqual(c.selected,(0,8));self.assertEqual(c.wide_reacquired,1)
        self.assertEqual(c.state,'TRACK')

    def test_ambiguous_wrong_class_low_score_and_far_candidates_are_not_reacquired(self):
        cases=([person(0.,-2.,(0,8)),person(.5,-2.,(0,9))],
               [person(0.,-2.,(2,8))],[person(0.,-2.,(0,8),.5)],
               [person(40.,-2.,(0,8))])
        for tracks in cases:
            c=seeded_core();last=c.last_seen
            for i in (1,2):c.step(tracks,last+i*.04,(-2.,0.),current_pose=(-2.,0.))
            self.assertEqual(c.selected,(0,1));self.assertNotEqual(c.state,'TRACK')

    def test_large_continuous_travel_and_wraps_do_not_request_home(self):
        for rate,start in ((75.,0.),(-75.,0.),(75.,160.),(-75.,-160.)):
            result=pipeline(m,rate,2.,start)
            self.assertGreater(abs(result['actual_travel_deg']),100.)
            self.assertEqual(result['home_requests'],0)
            self.assertEqual(result['lost_frames'],0)
            self.assertLess(result['final_error_deg'],3.)

    def test_large_motion_chase_improves_with_bounded_settled_motion(self):
        before,after=pipeline(old,90.,2.),pipeline(m,90.,2.)
        self.assertLess(after['chase_mean_error_deg'],before['chase_mean_error_deg']*.85)
        self.assertLess(after['final_error_deg'],3.)
        self.assertLess(after['settled_peak_to_peak_deg'],1.)

    def test_small_motion_pipeline_remains_identical_to_previous_version(self):
        before,after=pipeline(old,12.,2.),pipeline(m,12.,2.)
        self.assertEqual(before,after)

    def test_prediction_velocity_stops_at_displacement_cap(self):
        c=seeded_core(velocity=180.,start=170.)
        last=c.last_seen
        c.last_tracked_velocity=(180.,0.)
        c.drive(last+.30)
        # drive() needs an explicit loss state, as normal tracking ignores coast.
        c.state='COAST';c.drive(last+.30)
        self.assertLessEqual(abs(c.aim_goal[0]-c.last_tracked_goal[0]),24.+1e-8)
        self.assertEqual(c.aim_velocity[0],0.)

    def test_reacquire_window_expires_and_long_loss_never_selects_a_new_person(self):
        c=seeded_core();last=c.last_seen
        tr=person(0.,-2.,key=(0,8))
        for age in (1.,1.04):c.step([tr],last+age,(-2.,0.),current_pose=(-2.,0.))
        self.assertEqual(c.selected,(0,1))
        self.assertEqual(c.wide_reacquired,0)

    def test_large_capture_request_is_continuous_at_braking_and_mode_boundaries(self):
        c=m.CONFIG
        for error in (4.,12.,25.):
            brake=c['speed_tracking_brake_deg_s2'][0]
            t=c['speed_tracking_brake_latency_s']
            boundary=math.sqrt((brake*t)**2+2*brake*error)-brake*t
            if error>=12.:
                a=m.speed_tracking_request(c,0,error,90.+boundary-.001,90.,True)
                b=m.speed_tracking_request(c,0,error,90.+boundary+.001,90.,True)
                self.assertLess(abs(a-b),.01)
            a=m.speed_tracking_request(c,0,error-.000001,90.,90.,True)
            b=m.speed_tracking_request(c,0,error+.000001,90.,90.,True)
            self.assertLess(abs(a-b),.01)

    def test_invalid_config_rejected(self):
        for changes in ({'wide_coast_s':5.},{'wide_coast_max_deg':(120.,20.)},
                        {'wide_error_enter_deg':(2.,2.)},{'wide_reacquire_score':.2}):
            with self.assertRaises(ValueError):m.validate_visual_config(dict(m.CONFIG,**changes))


class WideMotorTests(unittest.TestCase):
    def setUp(self):
        self.clock=fixtures.Clock();self.stack=contextlib.ExitStack();self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(m.time,'monotonic',self.clock.monotonic))
        self.stack.enter_context(patch.object(m.time,'sleep',self.clock.sleep))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def driver(self,start=(0.,0.)):
        bus=make_native_model(fixtures.MechanicalMotorModel,self.clock,start)
        d=m.VisualMotorService(dict(m.CONFIG,startup_auto_home=False),lambda:bus)
        d.serial=bus;d.stop_event=fixtures.Event(self.clock)
        d.sent_target=d.manual_target=d._initialize();d.ready=True;d.phase='DISABLED'
        self.clock.pump=lambda:d.input(0.,0.)
        d.input(0.,0.);d.confirm_reference();d.request_arm(True)
        d.sent_target=d.manual_target=d._arm_current();d.armed=True;d.phase='ARMED'
        d.last_send=self.clock.now;d.set_response_context('TRACK')
        return d,bus

    def tick(self,d,goal=(30.,0.),ff=(70.,0.)):
        d.input(0.,0.);d.offer_tracking_goal(goal,self.clock.now,(True,True),ff)
        self.clock.sleep(.02);d._measure();d._advance(d.manual_target,d.c['max_rate'])

    def test_real_generated_driver_crosses_wrap_and_stops_on_disarm(self):
        d,bus=self.driver((170.,0.))
        for _ in range(24):self.tick(d,(220.,0.))
        self.assertGreater(d.angles[0],180.)
        self.assertGreater(d.stream_velocity[0],0.)
        self.assertEqual(d.snapshot()['wide_pause_counts'],{})
        d._disable_now()
        self.assertFalse(any(bus.enabled.values()))

    def test_feedback_pause_is_diagnosed_and_still_sends_zero(self):
        d,bus=self.driver()
        for _ in range(8):self.tick(d)
        bus.drop_feedback.add(1)
        d._measure()
        self.assertEqual(d.snapshot()['wide_pause_reason'],'FEEDBACK_PAUSE')
        self.assertEqual(bus.rates,{1:0,2:0})
        self.assertTrue(d.armed)
        bus.drop_feedback.clear();d._measure()
        self.assertIsNone(d.pause_reason)
        self.assertEqual(d.pause_counts['FEEDBACK_PAUSE'],1)

    def test_pitch_outward_speed_still_rejected_with_confirmed_direction(self):
        d,bus=self.driver((0.,29.5))
        self.tick(d,(0.,29.5),(0.,0.))
        with self.assertRaisesRegex(RuntimeError,'Unsafe tracking speed'):d._write(m.set_speed(2,-1))
        d._write(m.set_speed(2,1))

    def test_blocked_motor_still_triggers_existing_fault_protection(self):
        d,bus=self.driver();bus.stuck=True
        error=None
        for _ in range(150):
            try:self.tick(d,(30.,0.),(0.,0.))
            except RuntimeError as exc:error=exc;break
        self.assertIsNotNone(error)
        self.assertIn('SUSPECTED_BLOCK',str(error))
        d._record_fault(error)  # Same handling as the UART worker's exception path.
        self.assertFalse(any(bus.enabled.values()))
        self.assertIsNotNone(d.fault)

    def test_explicit_home_is_available_after_a_continuous_turn(self):
        d,bus=self.driver((170.,0.))
        for _ in range(24):self.tick(d,(210.,0.),(0.,0.))
        d.set_response_context('HOLD');d.input(0.,0.);d.request_home()
        d._home_and_clear()
        self.assertEqual(d.phase,'HOME_HOLD')
        self.assertTrue(d.armed)
        self.assertLess(abs(d.angles[0]),1.1)

    def test_short_visual_loss_keeps_speed_mode_and_same_id_resumes_without_rearming(self):
        d,bus=self.driver();hub=m.TrackingHub(d,dict(m.CONFIG),self.clock.monotonic)
        started=self.clock.now
        hub.offer_frame([person()],640,480,started,d.angles)
        hub.tracking.select((0,1),[person()],started)
        for _ in range(12):
            self.clock.sleep(.04);d._measure()
            bearing=60.*(self.clock.now-started)
            hub.offer_frame([person(bearing,d.angles[0])],640,480,self.clock.now,d.angles)
            hub.tick();d._advance(d.manual_target,d.c['max_rate'])
        self.assertTrue(all(d.native_tracker.engaged))
        before=len(bus.writes)
        for _ in range(4):
            self.clock.sleep(.04);d._measure()
            hub.offer_frame([],640,480,self.clock.now,d.angles)
            hub.tick();d._advance(d.manual_target,d.c['max_rate'])
        self.assertEqual(hub.tracking.state,'COAST')
        self.assertTrue(all(d.native_tracker.engaged))
        self.assertFalse(any(p[2] in (0,5,6) for p in bus.writes[before:]))
        self.clock.sleep(.04);d._measure()
        bearing=60.*(self.clock.now-started)
        hub.offer_frame([person(bearing,d.angles[0])],640,480,self.clock.now,d.angles)
        hub.tick();d._advance(d.manual_target,d.c['max_rate'])
        self.assertEqual(hub.tracking.state,'TRACK')
        self.assertNotIn('SHORT_LOSS',hub.notice)
        self.assertNotIn('TARGET_NOT_VISIBLE',hub.notice)
        self.assertTrue(d.armed)
        self.assertFalse(d.home_requested)
        self.assertFalse(any(p[2] in (5,6) for p in bus.writes[before:]))

    def test_hub_exposes_loss_and_pause_diagnostics_without_homing(self):
        d,bus=self.driver();hub=m.TrackingHub(d,dict(m.CONFIG),self.clock.monotonic)
        hub.offer_frame([person()],640,480,self.clock.now,d.angles)
        hub.tracking.select((0,1),[person()],self.clock.now)
        self.clock.sleep(.04)
        hub.offer_frame([],640,480,self.clock.now,d.angles);hub.tick()
        state=hub.state()
        self.assertIn('wide_motion',state['performance'])
        self.assertIn('IDENTITY_OR_FOV_LOSS',state['performance']['wide_motion']['flags'])
        self.assertFalse(d.home_requested)


class SourcePreservationTests(unittest.TestCase):
    def test_previous_files_are_unchanged(self):
        spec=importlib.util.spec_from_file_location('wide_builder',ROOT/'build.py')
        builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
        self.assertGreater(builder.verify_previous(),100)


if __name__=='__main__':unittest.main()
