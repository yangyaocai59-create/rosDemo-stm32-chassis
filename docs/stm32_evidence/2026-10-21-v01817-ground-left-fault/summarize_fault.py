"""Offline summary of the failed motion and subsequent stopped-only trace read."""
import hashlib
import json
from pathlib import Path
import struct

here = Path(__file__).resolve().parent
root = next(p for p in here.parents if (p/'Application').is_dir())
motion = here/'motion_capture'
trace_dir = here/'fault_trace'
frames = json.loads((trace_dir/'trace_parsed.json').read_text())
active = [r for r in frames if r['state'] == 2]
stopped = [r for r in frames if r['state'] == 3]
assert len(frames) == 88 and len(active) == 87 and len(stopped) == 1
assert all(b['t']-a['t'] == 20 for a,b in zip(frames,frames[1:]))
assert all(r['stby'] == 1 and r['req_w'] == 300 and r['req_v'] == 0 for r in active)
assert stopped[0]['stby'] == 0 and stopped[0]['fault'] == 5
assert all(w['pwm'] == 0 for w in stopped[0]['wheels'].values())
decoded = json.loads((motion/'decoded.json').read_text())
for row in decoded['wheel']:
    assert list(struct.unpack('<I4q4i', bytes.fromhex(row['payload_hex']))) == row['values']
summary = json.loads((motion/'summary.json').read_text())
read = json.loads((trace_dir/'summary.json').read_text())
assert read['errors'] == [] and read['trace_end_seen'] and read['uart_tx_hex'] == ['74']
assert all(c['type'] == 4 for c in read['tx_commands'])
assert read['final_status'][5:10] == [3,5,6,0,464] and not any(read['final_status'][10:16])
assert not any(read['final_wheel'][5:])
assert 'wheel_mask=0x01' in (motion/'ch340.log').read_text()

def max_run(values):
    longest = run = 0
    for yes in values:
        run = run+1 if yes else 0
        longest = max(longest, run)
    return longest*20

wheel = {}
for name in ('LF','LR','RF','RR'):
    rows = [r['wheels'][name] for r in active]
    wheel[name] = {'sum_dc': sum(r['dc'] for r in rows),
                   'nonzero_dc_frames': sum(r['dc'] != 0 for r in rows),
                   'pwm_min_max': [min(r['pwm'] for r in rows),max(r['pwm'] for r in rows)],
                   'max_zero_dc_frame_accounting_ms': max_run([r['dc']==0 for r in rows]),
                   'max_near_rest_frame_accounting_ms': max_run([abs(r['cps'])<=20 for r in rows])}
fault_status = next(r for r in decoded['status'] if r['phase']=='LEFT_motion' and r['values'][5]==3)
stop = next(c for c in summary['tx_commands'] if c['phase']=='LEFT_motion' and c['type']==2)
assert fault_status['elapsed_s'] <= stop['elapsed_s']
manifest = json.loads((root/'build/firmware/0.18.17/source_manifest.json').read_text())
assert all(hashlib.sha256((root/name).read_bytes()).hexdigest()==sha for name,sha in manifest['files'].items())
report = {'scope': 'failed single left run; never a tracking PASS or six-second completed test',
    'active_frames': len(active), 'active_time_ms': [active[0]['t'],active[-1]['t']],
    'fault_frame_time_ms': stopped[0]['t'], 'wheel': wheel,
    'commands_sent': sum(c['type']==1 for c in summary['tx_commands']),
    'fault_status_before_host_stop': True,
    'first_fault_status_host_motion_elapsed_s': fault_status['elapsed_s'],
    'host_stop_motion_elapsed_s': stop['elapsed_s'],
    'source_files_checked': len(manifest['files']),
    'final_status': read['final_status'], 'final_wheel': read['final_wheel'],
    'read_only_get_status_count': len(read['tx_commands']),
    'user_observation': '没看清，无法确认',
    'limits': ['No independent visual confirmation; counts are feedback evidence, not proof of a physical seizure.',
               'Same wheel-mask log can result from legacy stall or exhausted-front recovery; private fault branch not telemetered.',
               '87 ACTIVE frames include startup ramp, not the steady 2.54-second window of completed tests.',
               'No fault clear, retry, calibration, firmware edit or flashing occurred.'],
    'input_sha256': {str(p.relative_to(here)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (motion/'ch340.log', motion/'decoded.json', motion/'summary.json',
                  trace_dir/'ch340.log', trace_dir/'trace_parsed.json', trace_dir/'summary.json')}}
(here/'fault_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('input_sha256','limits')}, ensure_ascii=False, indent=2))
