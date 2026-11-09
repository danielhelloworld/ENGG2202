"""Spherical camera geometry, pole hysteresis and bounded local maneuver tests."""
import math
import unittest
import test_tracker as app
import test_native_tracking as native

m=app.m


def project(azimuth,elevation,pose):
    c=m.CONFIG;y=math.radians(pose[0]);e=math.radians(-pose[1])
    az,el=map(math.radians,(azimuth,elevation))
    target=(math.cos(el)*math.cos(az),math.cos(el)*math.sin(az),math.sin(el))
    forward=(math.cos(e)*math.cos(y),math.cos(e)*math.sin(y),math.sin(e))
    right=(math.sin(y),-math.cos(y),0.)
    up=(-math.sin(e)*math.cos(y),-math.sin(e)*math.sin(y),math.cos(e))
    dot=lambda v:sum(a*b for a,b in zip(target,v))
    z=dot(forward)
    return (dot(right)/z/(2*math.tan(math.radians(c['camera_fov'][0]/2))),
            dot(up)/z/(2*math.tan(math.radians(c['camera_fov'][1]/2))))


class OverheadTests(unittest.TestCase):
    def test_projected_line_of_sight_recovers_world_direction_at_high_pitch(self):
        for pose,az,el in [((30.,-75.),40.,80.),((390.,-80.),400.,82.),((-175.,-85.),175.,87.)]:
            vector=m.sight_vector(project(az,el,pose),pose,m.CONFIG)
            found=math.degrees(math.atan2(vector[1],vector[0]))
            elevation=math.degrees(math.atan2(vector[2],math.hypot(*vector[:2])))
            self.assertAlmostEqual(m.signed_mechanical(found-az),0.,places=6)
            self.assertAlmostEqual(elevation,el,places=6)

    def planner_at_pole(self,heading=0.):
        p=m.OverheadPlanner(m.CONFIG)
        plan=p.observe(project(heading,85.,(heading,-80.)),(heading,-80.),(heading,-80.),1.)
        self.assertEqual(p.state,'POLE')
        self.assertGreaterEqual(plan['goal'][1],-90.)
        return p

    def test_noise_exactly_overhead_does_not_spin_yaw_or_invert_pitch(self):
        p=self.planner_at_pole()
        for i,az in enumerate([0.,170.,-170.,90.,-90.,180.,0.]*4):
            pose=(0.,-88.)
            plan=p.observe(project(az,89.6,pose),pose,pose,1.04+i*.03)
            self.assertEqual(p.state,'POLE');self.assertEqual(plan['goal'][0],0.)
            self.assertGreaterEqual(plan['goal'][1],-90.)
        self.assertEqual(p.turn_count,0)

    def test_crossing_pole_commits_one_yaw_relay_instead_of_pitch_inversion(self):
        for heading in (0.,350.,-350.):
            p=self.planner_at_pole(heading)
            pose=(heading,-88.)
            error=project(heading+180.,87.,pose)
            p.observe(error,pose,pose,1.04)
            plan=p.observe(error,pose,pose,1.08)
            self.assertEqual(p.state,'TURN');self.assertEqual(p.turn_count,1)
            self.assertAlmostEqual(plan['goal'][0],heading+180.)
            self.assertEqual(plan['goal'][1],-88.)
            for i,yaw in enumerate((heading+20.,heading+80.,heading+130.)):
                plan=p.missing(1.2+i*.1,(yaw,-88.))
                self.assertEqual(plan['goal'],(heading+180.,-88.))
            self.assertIsNone(p.missing(2.,(heading+178.,-88.)))
            self.assertEqual(p.state,'POLE')
            pose=(heading+180.,-88.)
            plan=p.observe(project(heading+180.,70.,pose),pose,pose,2.1)
            self.assertEqual(p.state,'GEOMETRY')
            self.assertAlmostEqual(plan['goal'][1],-70.)

    def test_peak_missed_between_frames_still_uses_the_overhead_relay(self):
        p=m.OverheadPlanner(m.CONFIG);pose=(0.,-80.)
        p.observe(project(0.,80.,pose),pose,pose,1.)
        p.observe(project(180.,80.,pose),pose,pose,1.04)
        plan=p.observe(project(180.,80.,pose),pose,pose,1.08)
        self.assertEqual(p.state,'TURN')
        self.assertEqual(plan['goal'],(180.,-88.))

    def test_maneuver_has_deadline_and_clear_cancels_it(self):
        p=self.planner_at_pole();pose=(0.,-88.);error=project(180.,87.,pose)
        p.observe(error,pose,pose,1.04);p.observe(error,pose,pose,1.08)
        self.assertIsNone(p.missing(5.2,pose))
        p.reset();self.assertEqual(p.state,'NORMAL');self.assertIsNone(p.missing(5.3,pose))

    def test_low_confidence_does_not_trigger_a_half_turn(self):
        p=self.planner_at_pole();pose=(0.,-88.)
        for i in range(10):p.observe(project(180.,87.,pose),pose,pose,1.04+i*.03,score=.3)
        self.assertEqual(p.state,'POLE');self.assertEqual(p.turn_count,0)

    def test_only_unique_same_class_sight_line_can_reacquire_during_overhead_turn(self):
        p=self.planner_at_pole();pose=(0.,-88.);error=project(180.,87.,pose)
        p.observe(error,pose,pose,1.04);p.observe(error,pose,pose,1.08)
        pose=(180.,-88.);ex,ey=project(180.,87.,pose)
        tr=app.track((0,2));tr.update(cx=.5+ex,cy=.5-ey)
        other=dict(tr,key=(0,3));wrong=dict(tr,key=(1,2))
        self.assertIsNone(p.reacquire([tr,other],(0,1),pose,1.2))
        self.assertIsNone(p.reacquire([wrong],(0,1),pose,1.23))
        self.assertIsNone(p.reacquire([tr],(0,1),pose,1.26))
        self.assertEqual(p.reacquire([tr],(0,1),pose,1.29)['key'],(0,2))
        self.assertIsNone(p.reacquire([tr],(0,1),pose,5.7))

    def test_native_short_coast_preserves_id_is_bounded_and_expires(self):
        c=m.ClickTracker(m.CONFIG);tr=app.track();c.select(tr['key'],[tr],1.)
        c.step([tr],1.02,pose=(0.,0.),current_pose=(0.,0.))
        c.predictor.samples=5;c.predictor.velocity=(.5,0.)
        c.last_tracked_velocity=(-43.5,0.);origin=c.last_tracked_goal
        c.step([],1.06,current_pose=(0.,0.))
        self.assertEqual(c.state,'COAST');self.assertEqual(c.selected,tr['key'])
        self.assertIsNotNone(c.aim_goal);self.assertLessEqual(abs(c.aim_goal[0]-origin[0]),4.)
        c.drive(1.3,(0.,0.));self.assertEqual(c.state,'LOST');self.assertIsNone(c.aim_goal)

    def test_level_tracking_keeps_the_verified_flat_controller(self):
        p=m.OverheadPlanner(m.CONFIG)
        self.assertIsNone(p.observe((.1,.1),(0.,-20.),(0.,-20.),1.))
        self.assertEqual(p.state,'NORMAL')


class OverheadDriverTests(unittest.TestCase):
    setUp=native.NativeTrackingTests.setUp
    enable=native.NativeTrackingTests.enable
    driver=native.NativeTrackingTests.driver

    def test_local_hub_executes_yaw_relay_without_browser_or_pitch_inversion(self):
        d,bus=self.driver((0.,-80.),continuous_yaw=True,native_velocity_damping_s=m.CONFIG['native_velocity_damping_s'])
        hub=m.TrackingHub(d,dict(m.CONFIG),self.clock.monotonic)
        tr=app.track();hub.tracking.select(tr['key'],[tr],self.clock.now)
        world=[(0.,80.)]*3+[(0.,85.)]*3+[(180.,87.)]*60+[(180.,70.)]*40
        min_pitch=0.
        for az,el in world:
            self.clock.sleep(.04);d._measure()
            pose=d.angles;ex,ey=project(az,el,pose)
            target=app.track(box=((.5+ex)*640-8,(.5-ey)*480-8,16,16))
            hub.offer_frame([target],640,480,self.clock.now,pose)
            hub.tick();d._advance(d.manual_target,d.c['max_rate'])
            min_pitch=min(min_pitch,d.sent_target[1])
        self.assertIsNone(d.fault);self.assertTrue(d.armed)
        self.assertAlmostEqual(d.angles[0],180.,delta=4.)
        self.assertAlmostEqual(d.angles[1],-70.,delta=3.)
        self.assertGreaterEqual(min_pitch,-90.)
        self.assertEqual(hub.tracking.overhead.turn_count,1)
        self.assertIsNone(hub.token)
        self.assertFalse(any(p[2] in (8,9,10) for p in bus.writes))


if __name__=='__main__':unittest.main()
