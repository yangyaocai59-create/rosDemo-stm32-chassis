"""Analyze the single already-finished authorized run; no hardware imports."""
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

here=Path(__file__).resolve().parent
capture=here/'capture'
summary=json.loads((capture/'summary.json').read_text())
decoded=json.loads((capture/'decoded.json').read_text())
trace=json.loads((capture/'trace_parsed.json').read_text())
final=summary['final_status']
assert summary['physical_setup']=='ground' and summary['focused_left_only']
assert summary['angular_command_mrad_s']==300 and not summary['errors']
assert len(summary['directions'])==1 and summary['directions'][0]['commands_sent']==120
assert not summary['clear_prior_health_fault_once_authorized']
assert final[5]==1 and final[6]==final[8]==0 and final[7]==2
assert final[9]==0x0150 and not any(final[10:16])
initial=decoded['status'][0]['values']
assert final[2]-initial[2]==120 and final[3]==initial[3]==0
assert not any(final[17:20]) and summary['host_crc_errors']==0
assert all(c['type'] in (1,2,4) for c in summary['tx_commands'])
assert sum(c['type']==1 for c in summary['tx_commands'])==120
active=[f for f in trace if f['state']==2 and len(f['wheels'])==4]
assert len(trace)==128 and len(active)==127
assert all(f['dt']==20 and f['req_w']==f['applied_w']==300 and
           f['req_v']==f['applied_v']==0 and f['fault']==0 and f['stby']==1 for f in active)
motion=[r for r in decoded['wheel'] if r['phase']=='LEFT_motion']
wheels={}
timeline=[]
for i,name in enumerate(('LF','LR','RF','RR')):
    rows=[f['wheels'][name] for f in active]
    groups=[]
    for f,row in zip(active,rows):
        if row['dc']==0:
            if groups and f['t']==groups[-1]['last_ms']+f['dt']:
                groups[-1]['last_ms']=f['t'];groups[-1]['covered_ms']+=f['dt']
            else:groups.append(dict(first_ms=f['t'],last_ms=f['t'],covered_ms=f['dt']))
    plateaus=[];first=0
    for j in range(1,len(motion)+1):
        if j==len(motion) or motion[j]['values'][1+i]!=motion[first]['values'][1+i]:
            if j-first>=2:
                plateaus.append(dict(first_ms=motion[first]['values'][0],
                    last_ms=motion[j-1]['values'][0],samples=j-first,
                    span_ms=motion[j-1]['values'][0]-motion[first]['values'][0],
                    phase_elapsed_start_s=motion[first]['elapsed_s']))
            first=j
    wheels[name]=dict(target_printed_cps=sorted(set(r['target'] for r in rows)),
        mean_filtered_cps=round(statistics.mean(r['cps'] for r in rows),2),
        filtered_range_cps=[min(r['cps'] for r in rows),max(r['cps'] for r in rows)],
        rms_error_cps=round(math.sqrt(statistics.mean((r['cps']-r['target'])**2 for r in rows)),2),
        zero_increment_groups=groups,longest_zero_ms=max((g['covered_ms'] for g in groups),default=0),
        raw_cps_range=[min(r['raw'] for r in rows),max(r['raw'] for r in rows)],
        pwm_range_permille=[min(r['pwm'] for r in rows),max(r['pwm'] for r in rows)],
        USB_full_motion_peak_abs_cps=max(abs(r['values'][5+i]) for r in motion),
        USB_same_net_count_runs=plateaus)
for f in active:
    row=dict(t_ms=f['t'],dt_ms=f['dt'])
    row.update({w+'_'+k:f['wheels'][w][k] for w in ('LF','LR','RF','RR')
                for k in ('dc','raw','target','cps','ff_x10','p_x10','i_x10','pwm')})
    timeline.append(row)
with (here/'all_wheel_timeline.csv').open('w',newline='') as out:
    writer=csv.DictWriter(out,fieldnames=list(timeline[0]));writer.writeheader();writer.writerows(timeline)
text=(capture/'ch340.log').read_text(encoding='utf-8')
timing=[list(map(int,m)) for m in re.findall(
 r'CTRL TIMING: motor_dt=(\d+)\.\.(\d+) safety_dt=(\d+)\.\.(\d+) overrun=(\d+)/(\d+) log_drop=(\d+) stack_free=(\d+)/(\d+)',text)]
assert timing and all(v[4:7]==[0,0,0] for v in timing)
assert not re.search('Firmware version|Reset cause',text)
release=json.loads((here.parents[2]/'build/firmware/0.18.21/source_manifest.json').read_text())
root=here.parents[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert len(release['files'])==167 and all(sha(root/n)==h for n,h in release['files'].items())
proof=json.loads((root/'build/firmware/0.18.21/verification.json').read_text())
assert all(sha(root/f'build/firmware/0.18.21/rosDemo-0.18.21.{ext}')==h for ext,h in proof['sha256'].items())
comparison=json.loads((capture/'baseline_comparison.json').read_text())
result=dict(scope='ONE_GROUND_LEFT_300_ONLY_NOT_REPEATABILITY_OR_PRECISION_ACCEPTANCE',
    firmware_identity='User reports .21 flashed; no boot banner or independent flash readback captured',
    source_capture='build/hardware_tests/loaded_pivot_compare_20261022_105749',
    user_observation='RR 连续，没有明显停顿或突然加速',
    result='MIXED_SUBJECTIVE_IMPROVEMENT_BUT_RR_MIDRUN_FEEDBACK_GAP_REMAINS_NOT_ACCEPTED',
    duration_s=6,velocity_commands=120,command_accepted_delta=120,rejected_delta=0,
    trace_span_ms=[active[0]['t'],active[-1]['t']],active_trace_frames=127,
    trace_is_last_2_54s_not_full_startup=True,wheels=wheels,
    voltage_mV=summary['directions'][0]['supply_mV_min_max'],
    final_status=final,host_crc_errors=0,fault_observed=False,boot_reset_banner_observed=False,
    motor_period_ms=[min(v[0] for v in timing),max(v[1] for v in timing)],
    safety_period_ms=[min(v[2] for v in timing),max(v[3] for v in timing)],
    minimum_stack_free_bytes=dict(motor=min(v[7] for v in timing),safety=min(v[8] for v in timing)),
    final_IMU_bias_valid=bool(decoded['imu'][-1]['values'][-1]&2),
    RR_previous_visible_zero_ms=140,RR_previous_zero_group_start_truncated=True,
    RR_current_visible_zero_ms=wheels['RR']['longest_zero_ms'],
    RR_midrun_USB_same_net_count=wheels['RR']['USB_same_net_count_runs'],
    RR_same_net_count_span_is_not_exact_physical_stop_duration=True,
    detailed_trace_does_not_cover_RR_midrun_gap=True,
    RR_previous_rms_cps=comparison['baseline']['LEFT']['wheel']['RR']['rms_error_cps'],
    RR_current_rms_cps=wheels['RR']['rms_error_cps'],
    source_167_and_four_artifact_hashes_unchanged=True,ports_closed=True,
    repeated=False,fault_cleared=False,calibration_sent=False,firmware_modified=False,
    limits=['Single LEFT only: no repeatability, opposite-direction, ROS tracking or navigation acceptance.',
      'Recorded waveforms differ between runs; this is not same-input software replay or proof of unique cause.',
      'RR has 140ms identical USB net count at 3.16..3.31s while filtered speed decays 83 to 8cps; tail-only no-zero trace cannot negate it.',
      'Same net count is a feedback gap, not proof of exact physical stall duration or unique cause; user saw continuous movement.',
      'Mean error and startup/other-wheel velocity deviations remain; no new tuning justified solely by this run.',
      'Detailed ring is final 2.54s; USB same net counts are not high-frequency encoder waveforms.',
      'Supply telemetry range is not a scope measurement and cannot exclude fast power transients.',
      'No observed reset/banner is not proof against every possible reset; no reset requested.'])
(here/'metrics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
event=dict(kind='RECORDED_MIDRUN_RR_NET_COUNT_PLATEAU_NOT_UNIQUE_CAUSE',
    note='RR net count939 repeats at156803/156843/156903/156943ms; no detailed PWM trace retained for that event.',
    wheel=[r for r in motion if 156600<=r['values'][0]<=157250],
    status=[r for r in decoded['status'] if r['phase']=='LEFT_motion' and 156600<=r['values'][0]<=157250])
(here/'RR_midrun_feedback_window.json').write_text(json.dumps(event,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
