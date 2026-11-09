"""Brief constant-input manual motion probe through the ordinary camera API."""
import argparse
import json
from pathlib import Path
import time
from commission_drive import Commission


class MotionProbe(Commission):
    def check(self,s):
        if s.get('simulated') or not s['armed'] or s['fault'] or not s['reference_valid']:
            raise RuntimeError('Healthy actual enabled motor required')
        if s['age'] is None or s['age']>.25 or s.get('feedback_degraded'):
            raise RuntimeError('Feedback stale')
        if abs(s['angles'][0])>40 or not -30<s['angles'][1]<25:
            raise RuntimeError('Probe excursion exceeded local test bounds')

    def home(self):
        self.command(action='home')
        end=time.perf_counter()+15
        while True:
            s=self.hold(.16)
            if not s['home_requested'] and s['phase']=='HOME_HOLD':return
            if time.perf_counter()>end:raise RuntimeError('Probe home deadline')

    def run(self):
        initial=self.request('/api/state')
        self.check(initial)
        if initial.get('connected'):raise RuntimeError('Another web controller active')
        self.token=self.request('/api/connect',{})['token']
        error=None
        try:
            self.home();self.hold(.8)
            for axis in (0,1):
                for sign in (1.,-1.):
                    value=.12*sign
                    tag=dict(axis=axis,input=value,begin=time.perf_counter())
                    self.rows.append(dict(segment=tag))
                    self.hold(.75,x=value if axis==0 else 0.,y=value if axis==1 else 0.)
                    self.hold(1.)
                    tag['end']=time.perf_counter()
                    print(json.dumps(tag),flush=True)
                self.home();self.hold(.5)
            self.hold(2.)
        except Exception as exc:
            error=str(exc);raise
        finally:
            try:
                self.command()
                self.request('/api/background',dict(token=self.token))
                telemetry=self.request('/api/telemetry')
            except Exception as exc:
                telemetry=dict(error=str(exc))
            self.output.parent.mkdir(parents=True,exist_ok=True)
            self.output.write_text(json.dumps(dict(synthetic=False,error=error,rows=self.rows,telemetry=telemetry),indent=2),encoding='utf-8')
            self.conn.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();MotionProbe(args.url,args.output).run()
