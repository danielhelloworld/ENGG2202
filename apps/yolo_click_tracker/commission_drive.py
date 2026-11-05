"""Explicit live commissioning via normal control API; never writes PI/zero/Flash.

The motor worker still owns UART, limits, acceleration and direction mapping.
Only run while the operator has authorized small movements of this installation.
"""
import argparse
import http.client
import json
import math
from pathlib import Path
import time
from urllib.parse import urlsplit


class Commission:
    def __init__(self, url, output):
        self.url = url.rstrip('/')
        self.addr = urlsplit(self.url)
        self.conn = http.client.HTTPConnection(self.addr.hostname, self.addr.port or 80, timeout=.5)
        self.output = Path(output)
        self.rows = []
        self.token = None
        self.seq = 0

    def request(self, path, body=None):
        data = None if body is None else json.dumps(body)
        start = time.monotonic()
        self.conn.request('GET' if body is None else 'POST', path, data,
                          {'Origin': self.url, 'Content-Type': 'application/json', 'Connection': 'keep-alive'})
        r = self.conn.getresponse()
        value = json.loads(r.read())
        if r.status != 200:
            raise RuntimeError('%s %d: %s' % (path, r.status, value))
        state = value.get('state', value)
        if 'angles' in state:
            self.rows.append(dict(pc_t=start, rtt_s=time.monotonic()-start, path=path, state=state))
        return value

    def command(self, x=0., y=0., action=''):
        self.seq += 1
        return self.request('/api/control', dict(token=self.token, seq=self.seq, x=x, y=y, action=action))

    def check(self, s):
        if s.get('simulated') or s.get('app_version') != 'hold-drive-v16':
            raise RuntimeError('Expected actual hold-drive-v16 board')
        if not s['armed'] or s['fault'] or not s['reference_valid']:
            raise RuntimeError('Motor not healthy: %s' % s.get('fault'))
        if s['age'] is None or s['age'] > .25 or s.get('feedback_degraded'):
            raise RuntimeError('Feedback not fresh')
        if any(abs(q) > 24 for q in s['angles']):
            raise RuntimeError('Commissioning excursion exceeded 24 degrees')

    def hold(self, seconds, **command):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            begin = time.monotonic()
            s = self.command(**command)
            self.check(s)
            time.sleep(max(0., .08-(time.monotonic()-begin)))
        return s

    def pulse(self, axis, vector, duration):
        before = self.command()
        self.check(before)
        origin = before['angles'][axis]
        other = before['angles'][1-axis]
        expected = vector*before['joystick_signs'][axis]
        tag = dict(axis=axis, input=vector, duration_s=duration, before=before['angles'], pc_t=time.monotonic())
        end = time.monotonic()+duration
        while time.monotonic() < end:
            begin = time.monotonic()
            s = self.command(x=vector if axis == 0 else 0., y=vector if axis == 1 else 0.)
            self.check(s)
            if (s['angles'][axis]-origin)*math.copysign(1., expected) < -1.:
                raise RuntimeError('Encoder movement opposite configured direction')
            if abs(s['angles'][1-axis]-other) > 2.:
                raise RuntimeError('Uncommanded axis moved more than 2 degrees')
            if abs(s['angles'][axis]-origin) > 18.:
                raise RuntimeError('Pulse exceeded 18 degree excursion')
            time.sleep(max(0., .08-(time.monotonic()-begin)))
        released = self.command()
        self.check(released)
        after = self.hold(1.2)
        tag.update(release=released['angles'], after=after['angles'], complete=True)
        self.rows.append(dict(pulse=tag))
        print(json.dumps(tag), flush=True)

    def run(self):
        initial = self.request('/api/state')
        self.check(initial)
        if initial.get('connected'):
            raise RuntimeError('Another web controller is active')
        self.token = self.request('/api/connect', {})['token']
        try:
            self.hold(1.)
            for axis in (0, 1):
                for level, duration in ((.08, .24), (.16, .24), (.30, .24)):
                    self.pulse(axis, level, duration)
                    self.pulse(axis, -level, duration)
                self.command(action='home')
                start = time.monotonic()
                while True:
                    s = self.hold(.16)
                    if not s['home_requested'] and s['phase'] == 'HOME_HOLD':
                        break
                    if time.monotonic()-start > 15:
                        raise RuntimeError('Home did not finish within commissioning deadline')
                self.hold(1.)
            self.hold(2.)
        finally:
            if self.token:
                try:
                    self.command()  # Release input, retain torque.
                    self.request('/api/background', dict(token=self.token))
                except Exception as exc:
                    print('Handoff pending: %s; onboard lease releases input' % exc, flush=True)
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.output.write_text(json.dumps(dict(synthetic=False, pi_modified=False, rows=self.rows), indent=2), encoding='utf-8')
            self.conn.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    Commission(args.url, args.output).run()
