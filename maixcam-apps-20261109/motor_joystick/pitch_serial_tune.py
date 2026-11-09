"""ID2-only bounded homing and local response tests; no zero/Flash writes."""
import argparse
import json
import math
from pathlib import Path
import time
import serial
from yaw_serial_tune import p, core, signed


class PitchSession:
    def __init__(self,port,output):
        self.bus=serial.Serial(port,115200,timeout=.01,write_timeout=.2)
        self.output=Path(output)
        self.parser=p.FeedbackParser()
        self.last_write=0.
        self.last=None
        self.target=None
        self.phase='READ_ONLY'
        self.profile=None
        self.experiment_goal=0.
        self.planned_velocity=0.
        self.data=dict(synthetic=False,motor_id=2,port=port,limits=[-100.,30.],
                       zero_changed=False,flash_written=False,samples=[],writes=[],stages=[])

    def write(self,frame):
        if frame[1]!=2 or frame[2] not in (0,1,3,6,14,15,16,17):
            raise RuntimeError('Command not permitted in pitch session')
        time.sleep(max(0.,.002-(time.perf_counter()-self.last_write)))
        if self.bus.write(frame)!=len(frame):
            raise RuntimeError('Incomplete UART write')
        self.last_write=time.perf_counter()
        self.data['writes'].append(dict(t=self.last_write,frame=frame.hex()))

    def feedback(self,kind):
        for _ in range(3):
            self.write(p.request_feedback(2,kind))
            end=time.perf_counter()+.08
            while time.perf_counter()<end:
                for mid,k,value in self.parser.feed(self.bus.read(max(1,self.bus.in_waiting))):
                    if mid==2 and k==kind:return value
        raise RuntimeError('Pitch feedback timeout: %d'%kind)

    def angle(self):
        q=signed(self.feedback(2)/10.)
        now=time.perf_counter()
        if self.last and abs(q-self.last[1])>max(3.,100.*(now-self.last[0])):
            raise RuntimeError('Implausible pitch feedback jump')
        self.last=now,q
        self.data['samples'].append(dict(t=now,angle=q,target=self.target,phase=self.phase,profile=self.profile))
        return now,q

    def position(self,q,recovery=False):
        now,actual=self.angle()
        q=round(q,1)
        if not math.isfinite(q) or abs(q-actual)>6.05:
            raise RuntimeError('Pitch target exceeds local 6 degree horizon')
        if recovery:
            if abs(q)>abs(actual)+.051 or q*actual<-.001:
                raise RuntimeError('Recovery target must approach saved zero')
        elif not -100.<=q<=30.:
            raise RuntimeError('Pitch target exceeds working limits')
        units=round((q%360)*10)%3600
        self.write(p._frame(2,3,units.to_bytes(2,'big')))
        self.target=q

    def hold_measured(self):
        # Fresh local hold is permitted outside the work range after a cancelled recovery.
        _,q=self.angle()
        self.position(q,recovery=True)

    def initialize_hold(self):
        self.bus.reset_input_buffer()
        _,q=self.angle()
        self.data['start_angle']=q
        self.data['voltage_v']=self.feedback(4)/100.
        self.data['acceleration_feedback']=self.feedback(3)
        if not 8<=self.data['voltage_v']<=15:
            raise RuntimeError('Supply voltage outside motor range')
        # Preload pose before switching to direct mode, then request enable once.
        self.position(q,recovery=True)
        self.write(p.select_mode(2,4))
        self.write(p.set_speed(2,10))
        self.write(p.enable(2))

    def home(self):
        self.phase='HOME'
        started=time.perf_counter()
        origin=self.angle()[1]
        self.data['home_origin']=origin
        old_q=self.target
        last_tick=started
        stable=None
        progress_t,progress_q=started,origin
        while True:
            now,actual=self.angle()
            if now-started>20:
                raise RuntimeError('Pitch home deadline expired')
            if not min(origin,0.)-3.<=actual<=max(origin,0.)+3.:
                raise RuntimeError('Pitch departed inward recovery corridor')
            dt=min(.08,max(0.,now-last_tick));last_tick=now
            speed=min(60.,math.sqrt(2.*180.*abs(old_q)))
            q=old_q-math.copysign(min(abs(old_q),speed*dt),old_q) if old_q else 0.
            q=max(actual-5.,min(actual+5.,q))
            q=max(min(actual,0.),min(max(actual,0.),q))
            self.position(q,recovery=True)
            old_q=q
            if abs(actual)<abs(progress_q)-.3:
                progress_t,progress_q=now,actual
            elif abs(actual)>abs(progress_q)+2.:
                raise RuntimeError('Pitch moved away from zero')
            elif abs(actual)>2. and now-progress_t>2.:
                raise RuntimeError('Pitch made no inward progress')
            if abs(actual)<=.5 and abs(self.target)<=.05:
                stable=now if stable is None else stable
                if now-stable>=.4 and self.feedback(0)==0:break
            else:stable=None
            time.sleep(.01)
        self.position(0.)
        self.phase='HOME_HOLD'
        self.data.update(home_completed=True,home_seconds=time.perf_counter()-started,home_angle=self.angle()[1])
        print(json.dumps(dict(event='HOME_HOLD',angle=self.data['home_angle'],seconds=self.data['home_seconds'])),flush=True)

    def settle(self,seconds):
        end=time.perf_counter()+seconds
        while time.perf_counter()<end:
            begin=time.perf_counter();self.angle();time.sleep(max(0.,.02-(time.perf_counter()-begin)))

    def apply(self,profile):
        if not 8<=profile['kp']<=24 or not 7<=profile['ki']<=11 or profile['position_kp']!=50:
            raise RuntimeError('Profile outside pitch search bounds')
        if self.feedback(0)!=0:
            raise RuntimeError('Wait for pitch to stop before changing PI')
        for key,opcode in (('kp',15),('ki',16)):
            self.write(p._frame(2,opcode,profile[key].to_bytes(2,'big')))
        self.profile=dict(profile)

    def offset(self,q):
        if not math.isfinite(q) or abs(q)>15.001:
            raise RuntimeError('Only bounded +/-15 degree trials around zero')
        self.experiment_goal=q

    def advance_goal(self,dt,actual):
        old=self.target
        error=self.experiment_goal-old
        desired=math.copysign(min(30.,math.sqrt(2*120.*abs(error))),error) if error else 0.
        velocity=max(self.planned_velocity-120.*dt,min(self.planned_velocity+120.*dt,desired))
        delta=velocity*dt
        candidate=self.experiment_goal if delta*error>=0 and abs(delta)>=abs(error) else old+delta
        candidate=max(min(old,self.experiment_goal),min(max(old,self.experiment_goal),candidate))
        candidate=max(actual-4.,min(actual+4.,candidate))
        if not -15.001<=candidate<=15.001:
            raise RuntimeError('Planned pitch target escaped test bounds')
        self.position(candidate)
        self.planned_velocity=0. if candidate==self.experiment_goal else velocity

    def test(self):
        _,q=self.angle()
        if abs(q)>.5 or self.feedback(0)!=0:
            raise RuntimeError('Pitch must already be at saved zero before testing')
        self.phase='TEST'
        baseline=dict(kp=10,ki=9,position_kp=50)
        self.write(p.set_speed(2,5))
        candidates=[dict(baseline,kp=v) for v in (12,15,18)]
        engine=core.AutoTuneExperiment(self.apply,self.offset,amplitude=10.,segment_s=2.,tolerance=.4,amplitude_limit=15.)
        previous=self.angle()
        last_tick=time.perf_counter()
        engine.start(baseline,candidates,time.perf_counter())
        reported=0
        while engine.active:
            begin=time.perf_counter()
            now,q=self.angle()
            if not -18.<=q<=18.:
                raise RuntimeError('Pitch left zero-centered test corridor')
            velocity=(q-previous[1])/max(.001,now-previous[0]);previous=now,q
            self.advance_goal(min(.08,max(0.,now-last_tick)),q);last_tick=now
            engine.observe(now,self.experiment_goal-q,velocity)
            if len(engine.results)>reported:
                reported=len(engine.results)
                print(json.dumps(dict(trial=engine.results[-1])),flush=True)
            if (Path(__file__).parent/'PITCH_TUNE_CANCEL').exists():engine.cancel(now,'Operator cancellation')
            time.sleep(max(0.,.02-(time.perf_counter()-begin)))
        self.data['stages'].append(dict(state=engine.state,reason=engine.reason,best=engine.best,trials=engine.results))
        if engine.state!='COMPLETE':raise RuntimeError(str(engine.reason))
        self.data['selected_sent']=engine.best
        self.position(0.);self.phase='HOME_HOLD';self.settle(2.)
        self.data['pose_checks']=[]
        for goal in (15.,0.,-15.,0.):
            self.phase='POSE_CHECK'
            self.offset(goal)
            self.planned_velocity=0.
            last_tick=started=time.perf_counter()
            stable=None
            before=len(self.data['samples'])
            while True:
                now,q=self.angle()
                if not -18.<=q<=18. or now-started>8:
                    raise RuntimeError('Pitch pose check failed')
                self.advance_goal(min(.08,max(0.,now-last_tick)),q);last_tick=now
                if abs(q-goal)<=.4 and abs(self.target-goal)<=.051:
                    stable=now if stable is None else stable
                    if now-stable>=1.5:break
                else:stable=None
                time.sleep(.01)
            samples=self.data['samples'][before:]
            tail=[r for r in samples if r['t']>now-.5]
            self.data['pose_checks'].append(dict(goal=goal,final=q,
                tail_rmse_deg=math.sqrt(sum((r['angle']-goal)**2 for r in tail)/len(tail))))
            print(json.dumps(dict(pose_check=self.data['pose_checks'][-1])),flush=True)
        self.phase='HOME_HOLD';self.position(0.);self.settle(1.)

    def run(self,mode):
        try:
            self.initialize_hold()
            if mode=='home':self.home();self.settle(1.)
            else:self.test()
            self.write(p.select_mode(2,2))
            self.data.update(status='COMPLETE',final_angle=self.angle()[1],final_target=self.target,
                             enable_requested=True,final_mode=2)
        except BaseException as exc:
            self.data.update(status='FAILED',error=str(exc),rollback_pending=mode=='test')
            try:
                self.hold_measured()
                if mode=='test':
                    self.settle(1.)
                    self.apply(dict(kp=10,ki=9,position_kp=50))
                    self.data['rollback_pending']=False
            except Exception as hold:self.data['hold_error']=str(hold)
            raise
        finally:
            self.output.parent.mkdir(parents=True,exist_ok=True)
            self.output.write_text(json.dumps(self.data,indent=2),encoding='utf-8')
            self.bus.close()
            print(json.dumps({k:v for k,v in self.data.items() if k not in ('samples','writes','stages')}),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--mode',choices=('home','test'),required=True)
    args=parser.parse_args()
    PitchSession(args.port,args.output).run(args.mode)
