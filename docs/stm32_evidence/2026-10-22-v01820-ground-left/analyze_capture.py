"""Analyze an already-finished capture. Never opens serial or moves motors."""
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
assert summary['physical_setup']=='ground' and summary['focused_left_only']
assert summary['angular_command_mrad_s']==300 and not summary['errors']
assert len(summary['directions'])==1 and summary['directions'][0]['commands_sent']==120
assert not summary['clear_prior_health_fault_once_authorized']
final=summary['final_status']
assert final[5]==1 and final[6]==final[8]==0 and final[7]==2
assert not any(final[10:16]) and final[9]==0x0150
initial=decoded['status'][0]['values']
assert final[2]-initial[2]==120 and final[3]==initial[3]==0 and not any(final[17:20])
assert summary['host_crc_errors']==0
active=[f for f in trace if f['state']==2 and len(f['wheels'])==4]
assert len(trace)==128 and len(active)==127
assert all(f['dt']==20 and f['req_w']==f['applied_w']==300 and
           f['req_v']==f['applied_v']==0 and f['fault']==0 for f in active)
motion=[r for r in decoded['wheel'] if r['phase']=='LEFT_motion']
wheels={}
for index,name in enumerate(('LF','LR','RF','RR')):
    rows=[f['wheels'][name] for f in active]
    longest=run=0
    for f,row in zip(active,rows):
        run=run+f['dt'] if row['dc']==0 else 0
        longest=max(longest,run)
    wheels[name]={'target_printed_cps':sorted(set(v['target'] for v in rows)),
      'mean_filtered_cps':round(statistics.mean(v['cps'] for v in rows),2),
      'filtered_range_cps':[min(v['cps'] for v in rows),max(v['cps'] for v in rows)],
      'rms_error_cps':round(math.sqrt(statistics.mean((v['cps']-v['target'])**2 for v in rows)),2),
      'longest_raw_zero_ms':longest,'zero_dc_samples':sum(v['dc']==0 for v in rows),
      'pwm_range_permille':[min(v['pwm'] for v in rows),max(v['pwm'] for v in rows)],
      'USB_motion_peak_abs_cps':max(abs(r['values'][5+index]) for r in motion)}
text=(capture/'ch340.log').read_text(encoding='utf-8')
timing=[list(map(int,m)) for m in re.findall(
 r'CTRL TIMING: motor_dt=(\d+)\.\.(\d+) safety_dt=(\d+)\.\.(\d+) overrun=(\d+)/(\d+) log_drop=(\d+) stack_free=(\d+)/(\d+)',text)]
assert timing and all(v[4:7]==[0,0,0] for v in timing)
commands=summary['tx_commands']
assert not any(c['type']==3 for c in commands) # CMD_CLEAR_FAULT = 0x03
assert all(c['type'] in (1,2,4) for c in commands) # velocity/STOP/status only
result={'scope':'ONE_GROUND_LEFT_ONLY_NOT_REPEATABILITY_OR_TRACKING_ACCEPTANCE',
 'firmware_identity':'User-reported flash of 0.18.20; no boot version banner captured',
 'source_capture':'build/hardware_tests/loaded_pivot_compare_20261022_100241',
 'user_observation':'RR有明显停顿后恢复或突然加速',
 'performance_result':'FAIL_OBSERVED_RR_STOP_RECOVER_NOT_A_TRACKING_PASS',
 'duration_s':6,'command_mrad_s':300,'velocity_commands':120,
 'complete_active_trace_frames':127,'trace_span_ms':[active[0]['t'],active[-1]['t']],
 'trace_is_last_window_not_full_startup':True,'wheels':wheels,
 'RR_zero_group_starts_at_first_trace_frame':active[0]['wheels']['RR']['dc']==0,
 'RR_visible_140ms_is_not_full_gap_length':True,
 'voltage_mV':summary['directions'][0]['supply_mV_min_max'],
 'final_status':final,'faults_observed':False,'host_crc_errors':0,
 'minimum_reported_stack_free_bytes':{'motor':min(v[7] for v in timing),'safety':min(v[8] for v in timing)},
 'source_timing_ms':{'motor':[min(v[0] for v in timing),max(v[1] for v in timing)],
                     'safety':[min(v[2] for v in timing),max(v[3] for v in timing)]},
 'ports_closed':True,'automatic_repeat':False,'fault_cleared':False,'calibration_sent':False,
 'limits':['One loaded LEFT run only; user confirms RR stop/recovery; not accepted as tracking improvement.',
           'Startup peaks use full USB motion; detailed trace contains only last 2.54 seconds.',
           'Observed stack high-watermark covers this run, not every error/library/interrupt path.',
           'Build-time verification remains historical; not overwritten as hardware pass.']}
(here/'metrics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
manifest={str(p.relative_to(here)):hashlib.sha256(p.read_bytes()).hexdigest()
          for p in here.rglob('*') if p.is_file() and p.name!='capture_manifest.json'}
(here/'capture_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
