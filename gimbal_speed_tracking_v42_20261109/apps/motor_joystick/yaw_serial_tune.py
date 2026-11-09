"""Explicit COM6/ID1 F32C experiment, based on operator's known vendor PI values.

No current, zero, Flash, address or pitch commands. Serial must have one owner.
Uses the same repeated-baseline/verification engine as the onboard tuner.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import time
import serial

ROOT = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


p = load('yaw_protocol', ROOT/'vendor/protocol.py')
core = load('yaw_experiment', ROOT/'src/autotune_core.py')


def signed(q):
    return (q+180.) % 360.-180.


class YawTune:
    BASELINE = dict(kp=15, ki=1, position_kp=50)

    def __init__(self, port, output):
        self.output = Path(output)
        self.result = dict(synthetic=False, motor_id=1, port=port,
                           baseline=dict(self.BASELINE, position_ki=5),
                           current_supported=False, flash_written=False, stages=[], samples=[], writes=[])
        self.bus = serial.Serial(port,115200,timeout=.015,write_timeout=.2)
        self.parser = p.FeedbackParser()
        self.origin = self.target = None
        self.last = None
        self.profile = dict(self.BASELINE)
        self.last_write = 0.
        self.fault = None
        self.emergency = False

    def write(self, frame):
        if frame[1] != 1 or frame[2] in (8,9,10,13):
            raise RuntimeError('Disallowed command in yaw calibration')
        time.sleep(max(0., .002-(time.perf_counter()-self.last_write)))
        if self.bus.write(frame) != len(frame):
            raise RuntimeError('Incomplete serial write')
        self.last_write = time.perf_counter()
        self.result['writes'].append(dict(t=self.last_write, frame=frame.hex()))

    def feedback(self, kind):
        for attempt in range(3):
            self.write(p.request_feedback(1,kind))
            end = time.perf_counter()+.08
            while time.perf_counter() < end:
                data=self.bus.read(max(1,self.bus.in_waiting))
                for mid,k,value in self.parser.feed(data):
                    if mid==1 and k==kind:
                        return value
        raise RuntimeError('No valid yaw feedback for type %s' % kind)

    def angle(self):
        q=signed(self.feedback(2)/10.)
        now=time.perf_counter()
        if not -150 <= q <= 150 or (self.origin is not None and abs(q-self.origin)>7):
            self.emergency=True
            self.write(p.disable(1))
            raise RuntimeError('Yaw left bounded calibration corridor')
        if self.last and abs(q-self.last[1])>max(2.,50*(now-self.last[0])):
            raise RuntimeError('Implausible angle jump')
        self.last=(now,q)
        self.result['samples'].append(dict(t=now,angle=q,target=self.target,profile=dict(self.profile)))
        return now,q

    def offset(self, degrees):
        if not math.isfinite(degrees) or abs(degrees)>3.001:
            raise RuntimeError('Only +/-3 degree response trials allowed')
        now,actual=self.angle()
        target=round(self.origin+degrees,1)
        if not -140<=target<=140 or abs(target-actual)>6.5:
            raise RuntimeError('Unsafe yaw step')
        units=round((target%360)*10)%3600
        self.write(p._frame(1,3,units.to_bytes(2,'big')))
        self.target=target

    def apply(self, profile):
        if not 12<=profile['kp']<=24 or not 1<=profile['ki']<=2 or not 50<=profile['position_kp']<=60:
            raise RuntimeError('Profile exceeds candidate bounds')
        if self.feedback(0)!=0:
            raise RuntimeError('PI changes require stopped yaw')
        for key,opcode in (('kp',15),('ki',16),('position_kp',17)):
            self.write(p._frame(1,opcode,profile[key].to_bytes(2,'big')))
        self.profile=dict(profile)

    def settle(self, seconds=1.):
        end=time.perf_counter()+seconds
        while time.perf_counter()<end:
            start=time.perf_counter()
            self.angle()
            time.sleep(max(0.,.02-(time.perf_counter()-start)))

    def stage(self, name, baseline, candidates):
        engine=core.AutoTuneExperiment(self.apply,self.offset,amplitude=3.,segment_s=1.5,tolerance=.4)
        # Complete the first read before starting the observation timebase.
        last_t,last_q=self.angle()
        engine.start(baseline,candidates,time.perf_counter())
        reported=0
        while engine.active:
            begin=time.perf_counter()
            now,q=self.angle()
            velocity=(q-last_q)/max(.001,now-last_t)
            last_t,last_q=now,q
            engine.observe(now,self.target-q,velocity)
            if len(engine.results)>reported:
                print(json.dumps(dict(stage=name,trial=engine.results[-1])),flush=True)
                reported=len(engine.results)
            if (ROOT/'YAW_TUNE_CANCEL').exists():
                engine.cancel(now,'Operator cancellation file')
            time.sleep(max(0.,.02-(time.perf_counter()-begin)))
        report=dict(name=name,state=engine.state,reason=engine.reason,best=engine.best,
                    trials=engine.results,restore_pending=engine.restore_pending)
        self.result['stages'].append(report)
        if engine.state!='COMPLETE':
            raise RuntimeError('Stage failed: '+str(report))
        return dict(engine.best)

    def run(self):
        try:
            self.bus.reset_input_buffer()
            self.origin=round(self.angle()[1],1)
            if abs(self.origin)>20:
                raise RuntimeError('Commission near zero first; origin outside +/-20 degrees')
            self.result['origin']=self.origin
            self.result['voltage_v']=self.feedback(4)/100.
            self.result['initial_acceleration']=self.feedback(3)
            if self.feedback(0)!=0:
                raise RuntimeError('Yaw is moving before calibration')
            self.offset(0)
            self.write(p.select_mode(1,4))
            self.write(p.set_speed(1,5))
            self.write(p.enable(1))
            self.apply(self.BASELINE)
            self.settle()
            print(json.dumps(dict(event='STARTED',origin=self.origin,voltage=self.result['voltage_v'])),flush=True)
            speed=self.stage('speed',dict(self.BASELINE),[
                dict(self.BASELINE,kp=18),dict(self.BASELINE,kp=21),
                dict(self.BASELINE,kp=24),dict(self.BASELINE,ki=2)])
            self.result['speed_selected']=speed
            position=self.stage('position_stiffness',speed,[dict(speed,position_kp=55),dict(speed,position_kp=60)])
            self.result['selected']=position
            self.offset(0)
            self.settle(2.)
            self.write(p.select_mode(1,2))  # Original vendor UI mode; origin target is unchanged.
            self.result.update(status='COMPLETE',final_angle=self.angle()[1],enable_requested=True,
                               final_mode=2,volatile_speed_rpm=5)
        except BaseException as exc:
            self.fault=str(exc)
            self.result.update(status='FAILED',error=str(exc),restore_pending=True)
            if not self.emergency and self.origin is not None:
                try:
                    self.offset(0)
                    self.settle(1.5)
                    self.apply(self.BASELINE)
                    self.write(p.select_mode(1,2))
                    self.result['restore_pending']=False
                    self.result['restored']=dict(self.BASELINE)
                except Exception as restore:
                    self.result['restore_error']=str(restore)
            raise
        finally:
            self.result['last_sent_profile']=self.profile
            self.output.parent.mkdir(parents=True,exist_ok=True)
            self.output.write_text(json.dumps(self.result,indent=2),encoding='utf-8')
            self.bus.close()  # Retain the bounded position target; no routine disable.
            print(json.dumps({k:v for k,v in self.result.items() if k not in ('samples','writes','stages')}),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    YawTune(args.port,args.output).run()
