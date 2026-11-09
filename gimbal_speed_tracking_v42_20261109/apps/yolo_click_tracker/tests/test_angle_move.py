"""One-shot target angle: synthetic UART plant and ordinary hub authorization."""
import unittest
import test_tracker as app
import test_separate_controls as controls

m=app.m


class AngleMoveTests(unittest.TestCase):
    setUp=controls.SeparateControlTests.setUp
    make_driver=controls.SeparateControlTests.make_driver
    enable=controls.SeparateControlTests.enable

    def setup_hub(self, pose=(0.,0.)):
        d,bus=self.make_driver(pose);self.enable(d)
        h=m.TrackingHub(d,dict(m.CONFIG),self.clock.monotonic)
        h.offer_frame([],640,480,self.clock.now)
        token=h.connect()['token']
        self.clock.pump=lambda:h.tick()
        return d,bus,h,token

    def send(self,h,token,seq=1,action='angle_start',**extra):
        return h.command(dict(token=token,seq=seq,x=0.,y=0.,action=action,**extra))

    def advance(self,d,h,count=1):
        for _ in range(count):
            h.offer_frame([],640,480,self.clock.now)
            self.clock.sleep(.02);d._measure()
            d.phase=d._manual_phase(False)
            d._advance(d.manual_target,d.c['max_rate'])

    def test_single_request_completes_after_browser_handoff_without_more_commands(self):
        d,bus,h,token=self.setup_hub()
        count=len(bus.writes)
        self.send(h,token,yaw=12.,pitch=-8.,speed=30.)
        self.assertEqual(len(bus.writes),count)  # HTTP handler never writes UART.
        self.assertTrue(d.snapshot()['angle_move']['active'])
        h.background(token)
        for _ in range(250):
            self.advance(d,h)
            if d.angle_move.state=='COMPLETE':break
        self.assertEqual(d.angle_move.state,'COMPLETE')
        self.assertTrue(d.armed);self.assertIsNone(d.fault)
        self.assertEqual(tuple(bus.physical.values()),(12.,-8.))
        self.assertEqual(d.sent_target,(12.,-8.))
        report=d.snapshot()['angle_move']
        self.assertGreaterEqual(report['first_write_ms'],report['queue_ms'])
        self.assertGreaterEqual(report['first_motion_ms'],0.)
        before=len([p for p in bus.writes if p[2]==3]);self.advance(d,h,20)
        self.assertEqual(len([p for p in bus.writes if p[2]==3]),before)
        self.assertFalse(any(p[2] in (8,9,10,15,16,17,18) for p in bus.writes))

    def test_invalid_and_duplicate_requests_keep_enable_and_do_not_replace_goal(self):
        d,bus,h,token=self.setup_hub();seq=0
        for yaw,pitch,speed in ((151.,0.,30.),(0.,31.,30.),(float('nan'),0.,30.),(0.,0.,0.),(0.,0.,181.),('10',0.,30.)):
            seq+=1
            with self.assertRaises(m.ControlError):self.send(h,token,seq,yaw=yaw,pitch=pitch,speed=speed)
            self.assertTrue(d.armed);self.assertIsNone(d.angle_pending)
        seq+=1;self.send(h,token,seq,yaw=10.,pitch=-5.,speed=20.)
        pending=d.angle_pending
        with self.assertRaises(m.ControlError):self.send(h,token,seq+1,yaw=-10.,pitch=5.,speed=20.)
        self.assertEqual(d.angle_pending,pending);self.assertTrue(d.armed)

    def test_wrong_token_and_mixed_stick_cannot_start(self):
        d,bus,h,token=self.setup_hub()
        with self.assertRaises(m.ControlError):self.send(h,'wrong',yaw=10.,pitch=0.,speed=30.)
        with self.assertRaises(m.ControlError):
            h.command(dict(token=token,seq=1,x=.2,y=0.,action='angle_start',yaw=10.,pitch=0.,speed=30.))
        self.assertIsNone(d.angle_pending);self.assertTrue(d.armed)

    def test_cancel_then_home_or_disarm_cannot_leave_an_old_queued_move(self):
        for action in ('angle_cancel','home','disarm'):
            with self.subTest(action=action):
                d,bus,h,token=self.setup_hub()
                self.send(h,token,yaw=20.,pitch=-10.,speed=30.)
                self.advance(d,h,8)
                if action=='disarm':d._disable_now()
                else:self.send(h,token,2,action=action)
                if action=='angle_cancel':self.advance(d,h,80)
                self.assertFalse(d.angle_move.active)
                self.assertIsNone(d.angle_pending)
                if action!='disarm':self.assertTrue(d.armed)

    def test_large_cross_zero_move_uses_in_range_steps_in_the_requested_direction(self):
        d,bus,h,token=self.setup_hub((149.,0.))
        self.send(h,token,yaw=-149.,pitch=0.,speed=120.)
        previous=149.
        for _ in range(600):
            self.advance(d,h)
            q=d.sent_target[0]
            self.assertLessEqual(q,previous+1e-8)
            self.assertLessEqual(previous-q,d.c['manual_position_lead_deg'][0]+.051)
            self.assertTrue(-150.<=q<=150.)
            previous=q
            if d.angle_move.state=='COMPLETE':break
        self.assertEqual(d.angle_move.state,'COMPLETE')
        self.assertAlmostEqual(bus.physical[1],-149.,places=1)

    def test_trajectory_respects_speed_acceleration_and_jerk_envelopes(self):
        plan=m.AngleMove();plan.begin((0.,0.),(60.,-40.),30.,1.,1.,m.CONFIG)
        dt=.001;last_v=(0.,0.);last_a=None
        for _ in range(int(plan.duration/dt)+1):
            q,v=plan.waypoint(dt,(0.,0.),(360.,360.))
            self.assertLessEqual(max(abs(x) for x in v),30.+1e-8)
            a=tuple((x-y)/dt for x,y in zip(v,last_v))
            for x,cap in zip(a,m.CONFIG['manual_accel_deg_s2']):self.assertLessEqual(abs(x),cap+1.)
            if last_a:
                for x,y,cap in zip(a,last_a,m.CONFIG['manual_jerk_deg_s3']):self.assertLessEqual(abs(x-y)/dt,cap+10.)
            last_v,last_a=v,a
