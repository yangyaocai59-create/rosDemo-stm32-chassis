"""Offline-only audit of the completed capture. Never opens hardware ports."""
import csv
import hashlib
import json
from pathlib import Path
import struct

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p / 'Application').is_dir())
CAPTURE = HERE / 'capture'


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


summary = read_json(CAPTURE / 'summary.json')
trace = read_json(CAPTURE / 'trace_parsed.json')
decoded = read_json(CAPTURE / 'decoded.json')
frames = [json.loads(line) for line in
          (CAPTURE / 'usb_frames.jsonl').read_text(encoding='utf-8').splitlines()]
wheel_frames = [r for r in frames if r['type'] == 0x82]
assert len(wheel_frames) == len(decoded['wheel'])
for raw, recorded in zip(wheel_frames, decoded['wheel']):
    values = list(struct.unpack('<Iqqqqiiii', bytes.fromhex(raw['payload_hex'])))
    assert values == recorded['values']
    assert raw['seq'] == recorded['seq'] and raw['phase'] == recorded['phase']

assert summary['errors'] == [] and summary['host_crc_errors'] == 0
assert summary['automatic_stop_reason'] == 'RR_NET_COUNT_PLATEAU'
assert len(summary['directions']) == 1
run = summary['directions'][0]
assert run['commands_sent'] == 65 and not run['firmware_health_fault_observed']
assert run['actual_motion_seconds'] <= 6
velocity = [r for r in summary['tx_commands'] if r['type'] == 1]
assert len(velocity) == 65
assert all(r['phase'] == 'LEFT_motion' and r['payload_hex'] == '000000002c010000'
           for r in velocity)
assert all(r['type'] in (1, 2, 4) for r in summary['tx_commands'])

active = [s for s in trace if s['state'] == 2]
assert len(trace) == 128 and len(active) == 127
assert all(len(s['wheels']) == 4 and s['dt'] == 20 and s['stby'] == 1
           and s['fault'] == 0 and s['req_w'] == s['applied_w'] == 300
           for s in active)
assert all(b['t'] - a['t'] == 20 for a, b in zip(active, active[1:]))
zero = []
for sample in reversed(active):
    if sample['wheels']['RR']['dc'] != 0:
        break
    zero.append(sample)
zero.reverse()
assert len(zero) == 6 and (zero[0]['t'], zero[-1]['t']) == (1735523, 1735623)
assert all(s['wheels']['RR']['raw'] == 0 and s['wheels']['RR']['target'] == 240
           for s in zero)
assert [s['wheels']['RR']['pwm'] for s in zero] == [116, 118, 120, 122, 123, 125]
stop = trace[-1]
assert stop['t'] == 1735643 and stop['state'] == 1 and stop['stop'] == 2
assert stop['stby'] == 0 and all(w['pwm'] == 0 for w in stop['wheels'].values())
plateau = [r['values'] for r in decoded['wheel']
           if 1735523 <= r['values'][0] <= 1735623]
assert len(plateau) >= 2 and all(v[4] == 2577 for v in plateau)
end = summary['final_status']
assert end[1:4] == [185, 185, 0] and end[5:10] == [1, 0, 2, 0, 336]
assert all(v == 0 for v in end[10:16]) and all(v == 0 for v in end[17:20])

manifest = read_json(ROOT / 'build/firmware/0.18.21/source_manifest.json')
proof = read_json(ROOT / 'build/firmware/0.18.21/verification.json')
assert len(manifest['files']) == 167
assert all(sha(ROOT / n) == h for n, h in manifest['files'].items())
assert all(sha(ROOT / f'build/firmware/0.18.21/rosDemo-0.18.21.{ext}') == h
           for ext, h in proof['sha256'].items())
commands = read_json(ROOT / 'build/Debug/compile_commands.json')
system_command = next(c['command'] for c in commands
                      if c['file'].replace('\\', '/').endswith('/system_stm32f4xx.c'))
assert 'DATA_IN_ExtSRAM' not in system_command and 'DATA_IN_ExtSDRAM' not in system_command

rows = [s for s in trace if 1735303 <= s['t'] <= 1735643]
fields = ['t', 'state', 'stby'] + [f'{name}_{field}'
          for name in ('LF', 'LR', 'RF', 'RR')
          for field in ('dc', 'raw', 'target', 'cps', 'ff_x10', 'p_x10', 'i_x10', 'pwm')]
with (HERE / 'event_window.csv').open('w', encoding='utf-8', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    for s in rows:
        row = {k: s[k] for k in ('t', 'state', 'stby')}
        row.update({f'{name}_{field}': value for name, wheel in s['wheels'].items()
                    for field, value in wheel.items()})
        writer.writerow(row)

result = {
    'result': 'OFFLINE_EVIDENCE_CONSISTENT_NOT_UNIQUE_CAUSE_OR_ACCEPTANCE',
    'firmware_identity': summary['firmware_identity_basis'],
    'motion_seconds': run['actual_motion_seconds'],
    'velocity_commands': len(velocity),
    'trace_active_samples': len(active),
    'trace_period_ms': 20,
    'RR_zero_delta_sample_times_ms': [s['t'] for s in zero],
    'RR_zero_delta_covered_estimator_intervals_ms': sum(s['dt'] for s in zero),
    'RR_zero_delta_first_last_stamp_span_ms': zero[-1]['t'] - zero[0]['t'],
    'RR_zero_delta_PWM_permille': [s['wheels']['RR']['pwm'] for s in zero],
    'RR_zero_delta_filtered_cps': [s['wheels']['RR']['cps'] for s in zero],
    'RR_zero_delta_I_x10_permille': [s['wheels']['RR']['i_x10'] for s in zero],
    'other_wheel_deltas_during_RR_gap': {
        name: [s['wheels'][name]['dc'] for s in zero] for name in ('LF', 'LR', 'RF')},
    'RR_USB_plateau': plateau,
    'stop_trace_time_ms': stop['t'],
    'UART_STOP_log_time_ms': 1735636,
    'PWM_is_software_report_not_electrical_probe': True,
    'raw_timer_edges_or_AB_waveforms_recorded': False,
    'payload_decode_matches_saved_wheel_records': len(wheel_frames),
    'CRC_scope': 'Acquisition reports zero CRC errors; JSON stores decoded payloads, not complete CRC-bearing wire frames.',
    'operator_video_report': 'RR kept turning until whole-vehicle automatic stop.',
    'original_video_received': False,
    'independently_verified_physical_rotation': False,
    'production_source_hashes_unchanged': len(manifest['files']),
    'firmware_artifact_hashes_unchanged': proof['sha256'],
    'SystemInit_external_memory_defines_absent': True,
    'hardware_access_by_this_analysis': False,
}
(HERE / 'event_metrics.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
print(json.dumps({k: result[k] for k in (
    'result', 'velocity_commands', 'RR_zero_delta_covered_estimator_intervals_ms',
    'payload_decode_matches_saved_wheel_records', 'production_source_hashes_unchanged',
    'original_video_received')}, indent=2))
