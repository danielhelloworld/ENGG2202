"""Explicitly authorized pitch response experiment; no PI/zero/Flash writes."""
import argparse
import http.client
import json
from pathlib import Path
import time
from commission_drive import Commission


class InertiaProbe(Commission):
    def check(self,s):
        if s.get('simulated') or s.get('app_version') not in ('inertia-lab-v23','fast-return-v24','native-trajectory-v25','native-aim-v26'):
            raise RuntimeError('Expected actual inertia experiment build')
        if s.get('fault') or not s.get('armed') or not s.get('reference_valid'):
            raise RuntimeError('Motor unavailable: '+str(s.get('fault')))
        if any(not lo<=q<=hi for q,(lo,hi) in zip(s['angles'],s['limits'])):
            raise RuntimeError('Actual angle outside working range')

    def claim(self):
        s=self.request('/api/state');self.check(s)
        if s.get('connected'):raise RuntimeError('Another browser owns control')
        self.token=self.request('/api/connect',{})['token'];self.seq=0

    def release(self):
        if self.token:
            self.request('/api/background',dict(token=self.token));self.token=None

    def move(self,pitch,speed):
        self.claim();start=time.monotonic();initial=self.rows[-1]['state']['angles'][1]
        s=self.request('/api/control',
            dict(token=self.token,seq=1,x=0.,y=0.,action='native_start' if getattr(self,'native',False) else 'inertia_start',
                 pitch=pitch,speed=speed,accel=getattr(self,'native_accel',1)))
        self.seq=1;self.release()  # No PC control heartbeat is required after acceptance.
        samples=[];complete=False
        while time.monotonic()-start<12.:
            s=self.request('/api/state');self.check(s);samples.append(dict(pc_t=time.monotonic(),state=s))
            if (s['angles'][1]-initial)*(1 if pitch>initial else -1)<-1.:
                raise RuntimeError('Measured movement opposite requested pitch direction')
            if s['angle_move']['state']=='COMPLETE':
                complete=True;break
            time.sleep(.04)
        if not complete:raise RuntimeError('Experiment observation deadline; cancelling test')
        end=time.monotonic()+.7
        while time.monotonic()<end:
            s=self.request('/api/state');self.check(s);samples.append(dict(pc_t=time.monotonic(),state=s));time.sleep(.04)
        positions=[r['state']['angles'][1] for r in samples]
        direction=1 if pitch>initial else -1
        overshoot=max(0.,max((q-pitch)*direction for q in positions))
        result=dict(start_pitch=initial,target_pitch=pitch,requested_speed_deg_s=speed,
                    duration_s=samples[-1]['pc_t']-start,overshoot_deg=overshoot,
                    endpoint_error_deg=positions[-1]-pitch,angle_move=s['angle_move'],
                    max_planned_speed_deg_s=max(abs(r['state']['planned_velocity_deg_s'][1]) if 'planned_velocity_deg_s' in r['state'] else 0. for r in samples),
                    protection_restored=not s.get('inertia_test_active'),samples=samples)
        print(json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
        return result

    def run(self):
        results=[];error=None
        try:
            if getattr(self,'native',False):
                results.append(self.move(-10.,30.));results.append(self.move(0.,30.))
            results.append(self.move(-35.,40.))
            for speed in (60.,90.,120.):
                for pitch in (-75.,5.):results.append(self.move(pitch,speed))
                if max(r['overshoot_deg'] for r in results[-2:])>5.:
                    print('Overshoot exceeds quality threshold; do not increase speed',flush=True);break
            results.append(self.move(0.,80.))
        except Exception as exc:
            error=str(exc);raise
        finally:
            try:
                self.conn.close()
                self.conn=http.client.HTTPConnection(self.addr.hostname,self.addr.port or 80,timeout=1.)
                s=self.request('/api/state')
                if s.get('inertia_test_active'):
                    if self.token is None:self.claim()
                    self.command(action='angle_cancel');self.release()
                elif self.token:self.release()
                final=self.request('/api/state');telemetry=self.request('/api/telemetry')
            except Exception as exc:final=dict(error=str(exc));telemetry=None
            self.output.parent.mkdir(parents=True,exist_ok=True)
            self.output.write_text(json.dumps(dict(synthetic=False,error=error,moves=results,final=final,telemetry=telemetry,
                                                  incomplete_requests=self.rows if error else []),indent=2),encoding='utf-8')
            self.conn.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url',required=True);p.add_argument('--output',required=True)
    p.add_argument('--native',action='store_true')
    p.add_argument('--native-accel',type=int,default=1)
    args=p.parse_args();probe=InertiaProbe(args.url,args.output);probe.native=args.native;probe.native_accel=args.native_accel;probe.run()
