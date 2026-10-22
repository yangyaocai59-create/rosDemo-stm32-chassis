"""Offline-only analysis of the separately authorized one long-turn repeat."""
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import struct

here = Path(__file__).resolve().parent
root = next(p for p in here.parents if (p/'Application').is_dir())
read = lambda p: json.loads(p.read_text(encoding='utf-8'))
summary = read(here/'capture/summary.json')
data = read(here/'capture/decoded.json')
uart = (here/'capture/ch340.log').read_text(encoding='utf-8')
assert summary['errors'] == [] and summary['pre_motion_clear_count'] == 1
assert summary['detailed_trace_captured'] and summary['ports_closed']
assert summary['final_zero_speed_outputs_disabled']
assert summary['final_status'][5:10] == [1,0,2,0,336]
assert not any(summary['final_status'][10:16])
assert summary['velocity_commands'] == 319
assert summary['final_status'][2]-summary['initial_status'][2] == 319
assert summary['final_status'][3] == summary['initial_status'][3] == 1
tx = summary['tx_commands']
assert sum(t['type'] == 3 for t in tx) == 1
assert all(t['phase'] == 'authorized_clear_once' for t in tx if t['type'] == 3)
assert all(t['phase'] == 'LEFT_motion' and t['payload_hex'] == '000000002c010000'
           for t in tx if t['type'] == 1)
assert all(t['type'] in (1,2,3,4) for t in tx)
frames = [json.loads(l) for l in (here/'capture/usb_frames.jsonl').read_text(encoding='utf-8').splitlines()]
wheel_frames = [r for r in frames if r['type'] == 0x82]
assert len(wheel_frames) == len(data['wheel'])
assert all(list(struct.unpack('<Iqqqqiiii',bytes.fromhex(a['payload_hex']))) == b['values']
           for a,b in zip(wheel_frames,data['wheel']))
assert not summary['host_crc_errors']
assert all(not any(r['values'][17:20]) for r in data['status'])
motion = [r for r in data['wheel'] if r['phase'] == 'LEFT_motion']
motion_status = [r for r in data['status'] if r['phase'] == 'LEFT_motion']
assert all(r['values'][6] == r['values'][8] == 0 for r in motion_status)
assert all(r['values'][5] == 2 or (r['elapsed_s'] <= .6 and r['values'][5] == 1)
           for r in motion_status)
steady = [r for r in motion if r['elapsed_s'] >= 1]
plateaus = {}
for name, index in zip(('LF','LR','RF','RR'),range(1,5)):
    episodes=[]
    start=last=steady[0]
    for row in steady[1:]:
        if row['values'][index] != start['values'][index]:
            span=last['values'][0]-start['values'][0]
            if span >= 100:
                episodes.append(dict(first_source_ms=start['values'][0],last_source_ms=last['values'][0],
                                     span_ms=span,count=start['values'][index]))
            start=row
        last=row
    span=last['values'][0]-start['values'][0]
    if span>=100:
        episodes.append(dict(first_source_ms=start['values'][0],last_source_ms=last['values'][0],
                             span_ms=span,count=start['values'][index]))
    plateaus[name]=episodes

trace=[]
current=None
for line in uart.splitlines():
    match=re.search(r'TRACE t=(\d+) dt=(\d+) req=(-?\d+)/(-?\d+) applied=(-?\d+)/(-?\d+) state=(\d+) stop=(\d+) fault=(\d+) stby=(\d+)',line)
    if match:
        current=dict(zip(('t','dt','req_v','req_w','applied_v','applied_w','state','stop','fault','stby'),map(int,match.groups())))
        current['wheels']={}
        trace.append(current)
    match=re.search(r'TRACE (LF|LR|RF|RR) dc=(-?\d+) raw=(-?\d+) target=(-?\d+) cps=(-?\d+) ff_x10=(-?\d+) p_x10=(-?\d+) i_x10=(-?\d+) pwm=(-?\d+)',line)
    if match and current is not None:
        current['wheels'][match[1]]=dict(zip(('dc','raw','target','cps','ff_x10','p_x10','i_x10','pwm'),map(int,match.groups()[1:])))
assert len(trace)==128 and all(len(s['wheels'])==4 for s in trace)
active=[s for s in trace if s['state']==2]
assert len(active)==127 and all(s['dt']==20 and s['stby']==1 and not s['fault'] for s in active)
assert trace[-1]['stby']==0 and all(w['pwm']==0 for w in trace[-1]['wheels'].values())
tail={name:dict(mean_cps=statistics.mean(s['wheels'][name]['cps'] for s in active),
                zero_delta_samples=sum(s['wheels'][name]['dc']==0 for s in active),
                pwm_range=[min(s['wheels'][name]['pwm'] for s in active),max(s['wheels'][name]['pwm'] for s in active)])
      for name in ('LF','LR','RF','RR')}
zero_runs=[]
run=[]
for sample in active:
    if sample['wheels']['LR']['dc']==0:
        run.append(sample)
    else:
        if run:
            zero_runs.append(run)
            run=[]
if run:
    zero_runs.append(run)
longest=max(zero_runs,key=len)
lr_zero=dict(sample_times_ms=[s['t'] for s in longest],
             intervals_covered_ms=sum(s['dt'] for s in longest),
             pwm_permille=[s['wheels']['LR']['pwm'] for s in longest],
             ff_x10=[s['wheels']['LR']['ff_x10'] for s in longest],
             i_x10=[s['wheels']['LR']['i_x10'] for s in longest])
counts=[b-a for a,b in zip(summary['initial_counts'],summary['stopped_counts'])]
meters_per_count=math.pi*.065/1560
encoder_angle=(((counts[2]+counts[3])-(counts[0]+counts[1]))/2)*meters_per_count/.395
baseline=summary['baseline_wz_mrad_s']
prev=[r['values'] for r in data['imu'] if r['phase']=='countdown'][-1]
independent_yaw=0
for row in data['imu']:
    if row['phase'] not in ('LEFT_motion','LEFT_stopping'):
        continue
    v=row['values']
    if v[0] != prev[0]:
        dt=(v[0]-prev[0])&0xffffffff
        assert dt<=250
        independent_yaw+=((v[7]+prev[7])/2-baseline)/1000*dt/1000
        prev=v
assert abs(math.degrees(independent_yaw)-summary['approx_yaw_after_coast_deg'])<1e-8
supplies=[r['values'][6] for r in data['battery'] if r['phase']=='LEFT_motion']
manifest=read(root/'build/firmware/0.18.21/source_manifest.json')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert all(sha(root/n)==h for n,h in manifest['files'].items())
result=dict(result='COMPLETED_ONE_LONG_LEFT_NOT_REPEATABILITY_OR_ENCODER_TRUTH_ACCEPTANCE',
    motion_seconds=summary['motion_seconds'],approx_IMU_yaw_deg=summary['approx_yaw_after_coast_deg'],
    host_baseline_wz_mrad_s=summary['baseline_wz_mrad_s'],
    commands_sent_and_accepted=319,new_rejected=0,new_firmware_faults=0,
    motion_supply_mV_min_max=[min(supplies),max(supplies)],
    prior_fault_clear_count=1,final_state='IDLE/EXPLICIT, zero speeds, outputs disabled',
    net_counts_including_coast=counts,
    encoder_yaw_deg_if_existing_ROS_effective_track_395mm=math.degrees(encoder_angle),
    geometry_assumption='Wheel diameter65mm,1560counts/rev;395mm effective ROS track is only a calculation assumption, not firmware change/calibration',
    usb_zero_net_count_plateaus_after_1s=plateaus,
    tail_only_source_span_ms=[active[0]['t'],active[-1]['t']],tail_only_wheel_metrics=tail,
    tail_only_longest_LR_zero_delta_run=lr_zero,
    note='128-frame internal ring retains only final ~2.54s, not PWM for all 20s. USB net plateaus are not independently verified physical stops.',
    operator_report='Four wheels basically continuous, no obvious abnormality',
    independent_video_or_external_angle_received=False,
    payload_redecode_matches=len(wheel_frames),firmware_source_hashes_unchanged=len(manifest['files']))
(here/'analysis.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
(here/'trace_parsed.json').write_text(json.dumps(trace,indent=2)+'\n',encoding='utf-8')
with (here/'wheel_motion.csv').open('w',encoding='utf-8',newline='') as stream:
    writer=csv.writer(stream)
    writer.writerow(['source_ms','elapsed_s','LF_count','LR_count','RF_count','RR_count','LF_cps','LR_cps','RF_cps','RR_cps'])
    for row in motion:
        v=row['values']
        writer.writerow([v[0],row['elapsed_s'],*v[1:]])
print(json.dumps(result,indent=2))
