"""Offline audit of the one aborted long turn. No serial imports/hardware."""
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import struct

here = Path(__file__).resolve().parent
root = next(p for p in here.parents if (p/'Application').is_dir())
read = lambda p: json.loads(p.read_text(encoding='utf-8'))
data = read(here/'capture/decoded.json')
summary = read(here/'capture/summary.json')
uart = (here/'capture/ch340.log').read_text(encoding='utf-8')
assert 'wheel_mask=0x02' in uart
fault_stamp = int(re.search(r'\[(\d+)\] \[ERROR\] Fault manager latched wheel stall: wheel_mask=0x02', uart)[1])
assert fault_stamp == 2555822
assert summary['velocity_commands'] == 78
assert summary['final_zero_speed_outputs_disabled'] and summary['ports_closed']
assert summary['final_status'][5:9] == [3,5,6,0]
assert summary['final_status'][3] == 1
assert summary['host_crc_errors'] == 0
assert all(t['type'] in (1,2,4) for t in summary['tx_commands'])
vel = [t for t in summary['tx_commands'] if t['type'] == 1]
assert len(vel) == 78 and all(t['payload_hex'] == '000000002c010000' for t in vel)
raw_wheel = [read_row for line in (here/'capture/usb_frames.jsonl').read_text(encoding='utf-8').splitlines()
             if (read_row := json.loads(line))['type'] == 0x82]
assert len(raw_wheel) == len(data['wheel'])
assert all(list(struct.unpack('<Iqqqqiiii', bytes.fromhex(a['payload_hex']))) == b['values']
           for a,b in zip(raw_wheel,data['wheel']))
motion_wheel = [r for r in data['wheel'] if r['phase'] == 'LEFT_motion']
motion_imu = [r for r in data['imu'] if r['phase'] == 'LEFT_motion']
motion_status = [r for r in data['status'] if r['phase'] == 'LEFT_motion']
assert all(not any(r['values'][17:20]) for r in data['status'])
plateau = [r for r in motion_wheel if 2555203 <= r['values'][0] <= 2555803]
assert len(plateau) == 13 and all(r['values'][2] == -3886 for r in plateau)
before = [r for r in motion_wheel if 2555003 <= r['values'][0] <= 2555143]
assert all(r['values'][2] == -3885 for r in before)
# Independent trapezoidal integration of unique motion samples. The runner
# seeds its first interval from the final countdown sample; include it here.
seed = [r for r in data['imu'] if r['phase'] == 'countdown'][-1]['values']
angle = 0
prev = seed
baseline = summary['baseline_wz_mrad_s']
for r in motion_imu:
    v = r['values']
    if v[0] != prev[0]:
        dt = (v[0]-prev[0]) & 0xffffffff
        assert dt <= 250
        angle += ((v[7]+prev[7])/2-baseline) / 1000 * dt/1000
        prev = v
assert abs(math.degrees(angle)-summary['approx_yaw_on_error_deg']) < 1e-8
manifest = read(root/'build/firmware/0.18.21/source_manifest.json')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert all(sha(root/n) == h for n,h in manifest['files'].items())
result = dict(
    result='LONG_TURN_FAILED_FIRMWARE_LR_STALL_NOT_180_DEG_OR_ENCODER_ACCEPTANCE',
    fault_wheel='LR', fault_wheel_mask='0x02', fault_time_ms=fault_stamp,
    firmware_runtime_fault=5, controller_fault=0,
    first_active_status_ms=motion_status[0]['values'][0],
    first_active_to_latch_ms=fault_stamp-motion_status[0]['values'][0],
    last_motion_sample_host_elapsed_s=motion_imu[-1]['elapsed_s'],
    approximate_IMU_angle_deg=math.degrees(angle),
    host_only_baseline_mrad_s=baseline,
    angle_is_approximate_not_independent_heading=True,
    LR_zero_net_plateau_source_ms=[plateau[0]['values'][0], plateau[-1]['values'][0]],
    LR_zero_net_plateau_stamp_span_ms=600,
    LR_prior_short_plateau_source_ms=[before[0]['values'][0],before[-1]['values'][0]],
    LR_count_change_between_plateaus=-1,
    commands_sent=78, accepted=summary['final_status'][2]-summary['initial_status'][2],
    rejected=summary['final_status'][3]-summary['initial_status'][3],
    payload_redecode_matches=len(raw_wheel),
    observed_operator_report='LR kept turning; no visible pause',
    physical_rotation_independently_verified=False,
    fault_cleared_or_retried=False,
    detailed_PWM_ring_retrieved=False,
    missing_trace_reason='Runner aborted immediately on state-rejected ACK; normal-only dump path was not entered. Ports closed after stop; not reopened solely to avoid reset risk.',
    final_zero_speed_output_disabled=True,
    current_state='FAULT latched; must not resume without further authorization',
    firmware_source_hashes_unchanged=len(manifest['files']),
)
(here/'analysis.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
with (here/'wheel_window.csv').open('w',encoding='utf-8',newline='') as stream:
    writer = csv.writer(stream)
    writer.writerow(['source_ms','host_elapsed_s','LF_count','LR_count','RF_count','RR_count','LF_cps','LR_cps','RF_cps','RR_cps'])
    for r in motion_wheel:
        v=r['values']
        if v[0]>=2554800:
            writer.writerow([v[0],r['elapsed_s'],*v[1:]])
print(json.dumps(result,indent=2))
