"""Reproduce the summary and plot from captured actual-board JSON files."""
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parent
home = json.loads((ROOT/'home_telemetry_v16_20261105.json').read_text(encoding='utf-8'))
drive = json.loads((ROOT/'drive_response_v16_20261105.json').read_text(encoding='utf-8'))
telemetry = json.loads((ROOT/'drive_telemetry_v16_20261105.json').read_text(encoding='utf-8'))
rows = [r for r in drive['rows'] if 'state' in r]
pulses = [r['pulse'] for r in drive['rows'] if 'pulse' in r]
enabled = next(r for r in home['samples'] if r['armed'])
held = next(r for r in home['samples'] if r['phase'] == 'HOME_HOLD')
summary = dict(synthetic=False, version='hold-drive-v16', pi_modified=False,
               mechanical_zero_modified=False, limits=[[-150,150],[-100,30]],
               startup_angles_deg=enabled['actual_deg'],
               startup_enable_to_home_hold_s=held['t']-enabled['t'],
               pulse_count=len(pulses),
               pulse_actual_range_deg=[[min(r['state']['angles'][i] for r in rows),
                                        max(r['state']['angles'][i] for r in rows)] for i in (0,1)],
               all_samples_armed=all(r['state']['armed'] for r in rows),
               all_samples_fault_free=all(r['state']['fault'] is None for r in rows),
               http_rtt_p50_ms=statistics.median(r['rtt_s'] for r in rows)*1000,
               final_hold=telemetry['state']['angles'], final_armed=telemetry['state']['armed'])
tail = [r for r in telemetry['samples'] if r['t'] >= telemetry['samples'][-1]['t']-5]
summary['final_five_second_peak_to_peak_deg'] = [max(r['actual_deg'][i] for r in tail)-min(r['actual_deg'][i] for r in tail) for i in (0,1)]
summary['pulses'] = []
for p in pulses:
    i = p['axis']
    sign = -1 if p['input'] > 0 else 1
    first = next((r for r in rows if p['pc_t'] <= r['pc_t'] <= p['pc_t']+1.5
                  and (r['state']['angles'][i]-p['before'][i])*sign >= .3), None)
    summary['pulses'].append(dict(axis=i, input=p['input'],
        configured_direction_observed=(p['after'][i]-p['before'][i])*sign > 0,
        pc_observed_03_degree_latency_s=None if first is None else first['pc_t']-p['pc_t'],
        release_snapshot_to_settled_delta_deg=p['after'][i]-p['release'][i]))
summary['limitations'] = [
    'PC observed latency includes HTTP, application scheduling and angle sampling; it is not pure motor delay.',
    'Pulse length is requested PC duration; request and feedback timing limit its precision.',
    'Release delta starts at the API response snapshot, before UART release is necessarily applied; it is not measured overshoot past a fixed setpoint.',
    '0.1 degree encoder telemetry cannot exclude high frequency vibration.',
    'No visual target was selected; visual tracking gains and FOV were not calibrated.',
    'No PI baseline was known, and no PI or zero/Flash command was sent.',
    'Tests near zero do not establish safe maximum speed or stopping distance at the mechanical limits.']
(ROOT/'drive_summary_v16_20261105.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    print(json.dumps(summary,indent=2))
else:
    fig, axes = plt.subplots(3,1,figsize=(11,9),layout='constrained')
    hr = [r for r in home['samples'] if enabled['t'] <= r['t'] <= held['t']+2]
    for i,label in enumerate(('Yaw','Pitch')):
        axes[0].plot([r['t']-enabled['t'] for r in hr],[r['actual_deg'][i] for r in hr],label=label+' actual')
    axes[0].axhline(30,color='grey',ls=':',label='Pitch upper working limit')
    axes[0].set_title('Actual board: inward startup recovery (saved zero unchanged)')
    axes[0].set_ylabel('Angle (degrees)')
    axes[0].set_xlabel('Seconds after first enabled sample')
    axes[0].legend(loc='upper right')
    start=rows[0]['pc_t']
    for i,label in enumerate(('Yaw','Pitch')):
        axes[i+1].plot([r['pc_t']-start for r in rows],[r['state']['angles'][i] for r in rows],label='Actual')
        axes[i+1].plot([r['pc_t']-start for r in rows],[r['state']['target'][i] for r in rows],alpha=.75,label='Last sent target')
        for p in pulses:
            if p['axis']==i:
                axes[i+1].axvspan(p['pc_t']-start,p['pc_t']-start+p['duration_s'],alpha=.15,color='green')
        axes[i+1].set_title(label+' bidirectional pulses; green = requested input period')
        axes[i+1].set_xlabel('PC monotonic elapsed seconds')
        axes[i+1].set_ylabel('Angle (degrees)')
        axes[i+1].legend(loc='upper right')
    for ax in axes:
        ax.grid(alpha=.25)
    fig.savefig(ROOT/'drive_response_v16_20261105.png',dpi=160)
    plt.close(fig)
    print(json.dumps({k:v for k,v in summary.items() if k not in ('pulses','limitations')},indent=2))
