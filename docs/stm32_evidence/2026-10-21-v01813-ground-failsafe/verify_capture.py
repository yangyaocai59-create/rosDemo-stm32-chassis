"""Offline assertions only; never opens serial ports or sends commands."""
import hashlib
import itertools
import json
import pathlib
import re
import sys


folder = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(__file__).parent / 'capture'
summary = json.loads((folder / 'summary.json').read_text(encoding='utf-8'))
trace = json.loads((folder / 'trace_parsed.json').read_text(encoding='utf-8'))
decoded = json.loads((folder / 'decoded.json').read_text(encoding='utf-8'))
uart = (folder / 'ch340.log').read_text(encoding='utf-8')
active = [r for r in trace if r['state'] == 2]
assert len(active) == 127 and len(trace) == 128
assert all(r['dt'] == 20 and len(r['wheels']) == 4 for r in trace)
assert all(b['t'] - a['t'] == 20 for a, b in zip(trace, trace[1:]))
assert all((r['req_v'], r['req_w'], r['applied_v'], r['applied_w']) == (0, -200, 0, -200) for r in active)
assert all(r['wheels']['RF']['target'] == -160 for r in active)


def runs(predicate):
    result = []
    for matches, group in itertools.groupby(active, key=predicate):
        records = list(group)
        if matches:
            result.append(dict(first_ms=records[0]['t'], last_ms=records[-1]['t'],
                               samples=len(records), accounted_dt_ms=sum(r['dt'] for r in records),
                               timestamp_span_ms=records[-1]['t'] - records[0]['t'],
                               left_censored=records[0] is active[0],
                               right_censored=records[-1] is active[-1]))
    return result


near_rest = runs(lambda r: abs(r['wheels']['RF']['cps']) <= 20)
raw_zero = runs(lambda r: r['wheels']['RF']['dc'] == 0)
assert near_rest[-1]['first_ms'] == 674802 and near_rest[-1]['accounted_dt_ms'] == 1500
assert raw_zero[-1]['first_ms'] == 674722 and raw_zero[-1]['accounted_dt_ms'] == 1580
match = re.search(r'\[(\d+)\] \[ERROR\] Fault manager latched wheel stall: wheel_mask=0x([0-9A-Fa-f]+)', uart)
assert match and int(match[1]) == 676282 and int(match[2], 16) == 4
assert 'Chassis runtime FAULT: health manager fault, controller=none; motors disabled' in uart
assert 'TRACE END;' in uart
stopped = trace[-1]
assert (stopped['t'], stopped['state'], stopped['stop'], stopped['fault'], stopped['stby']) == (676302, 3, 6, 5, 0)
assert (stopped['req_v'], stopped['req_w'], stopped['applied_v'], stopped['applied_w']) == (0, 0, 0, 0)
assert all(w['pwm'] == 0 and w['ff_x10'] == 0 and w['p_x10'] == 0 and w['i_x10'] == 0 for w in stopped['wheels'].values())
tx = summary['tx_commands']
assert [r['phase'] for r in tx if r['type'] == 2] == ['initial']
assert not any(r['type'] == 3 for r in tx)
velocity = [r for r in tx if r['type'] == 1]
assert len(velocity) == 97 and all(r['phase'] == 'RIGHT_motion' for r in velocity)
assert summary['firmware_stop_verified_before_host_stop'] and not summary['final_host_stop_sent']
assert not summary['errors'] and summary['host_crc_errors'] == 0
final = summary['final_status']
assert final[5:9] == [3, 5, 6, 0] and final[9] == 0x01D0
assert final[10:16] == [0] * 6 and final[17:20] == [0] * 3
assert decoded['wheel'][-1]['values'][5:9] == [0] * 4
initial = decoded['status'][0]['values']
assert final[2] - initial[2] == 95 and final[3] - initial[3] == 2
assert 'Firmware version' not in uart and 'Reset cause' not in uart
timing = re.findall(r'motor_dt=(\d+)\.\.(\d+) safety_dt=(\d+)\.\.(\d+) overrun=(\d+)/(\d+) log_drop=(\d+) stack_free=(\d+)/(\d+)', uart)
assert timing and all(tuple(map(int, r[:7])) == (20, 20, 10, 10, 0, 0, 0) for r in timing)
verdict = {
    'verdict': 'PASS_OBSERVED_RF_RECOVERY_FAILURE_SHUTDOWN_NOT_A_PIVOT_TRACKING_PASS',
    'fault_uart_ms': int(match[1]), 'wheel_mask': '0x04 (RF)',
    'rf_near_rest_runs': near_rest, 'rf_raw_zero_runs': raw_zero,
    'first_fault_trace_ms': stopped['t'], 'trace_after_fault_uart_ms': stopped['t'] - int(match[1]),
    'first_fault_status_ms': summary['first_firmware_health_fault_status'][0],
    'host_elapsed_to_fault_s': summary['first_fault_host_elapsed_s'],
    'final_status_uptime_ms': final[0], 'no_stop_after_motion': True, 'no_clear_fault': True,
    'velocity_commands_sent': len(velocity), 'accepted_delta': 95, 'rejected_delta': 2,
    'timing_records': len(timing), 'min_stack_free_reported_motor_safety': [min(int(r[7]) for r in timing), min(int(r[8]) for r in timing)],
    'identity_limit': 'User-reported flash; no fresh version banner or Flash readback captured.',
    'measurement_limit': 'STBY/PWM are firmware trace state, not independent electrical/current measurements; no physical RF observation supplied yet.',
    'hashes': {name: hashlib.sha256((folder / name).read_bytes()).hexdigest() for name in ('ch340.log', 'summary.json', 'decoded.json', 'usb_frames.jsonl', 'trace_parsed.json', 'analysis.json')},
}
(folder / 'analysis_verdict.json').write_text(json.dumps(verdict, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(verdict, ensure_ascii=False, indent=2))
