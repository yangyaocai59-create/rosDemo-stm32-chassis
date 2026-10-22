"""Frozen-C reconstruction of recorded feedback. Not a motor/ground model."""
import csv
import hashlib
import io
import json
import math
import pathlib
import subprocess

folder = pathlib.Path(__file__).resolve().parent
root = next(p for p in folder.parents if (p/'Tools/tests/run_motion_tests.ps1').is_file())
capture = folder.parent/'2026-10-21-v01815-ground-pivot/capture'
manifest = json.loads((root/'build/firmware/0.18.15/source_manifest.json').read_text(encoding='utf-8'))
for name in ('wheel_speed_controller.c', 'wheel_speed_controller.h', 'wheel_speed_control_config.h', 'motor_driver.h', 'chassis_config.h'):
    assert hashlib.sha256((folder/'source_snapshot'/name).read_bytes()).hexdigest() == manifest['files']['Application/Chassis/'+name]
exe = root/'build/tests/v01815_cycle_replay.exe'
subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(folder/'front_replay.c'), '-lm', '-o', str(exe)], check=True)
frames = [r for r in json.loads((capture/'trace_parsed.json').read_text(encoding='utf-8')) if r['state'] == 2]
assert len(frames) == 127 and all(r['dt'] == 20 for r in frames)
fields = ['t', 'measured', 'ff', 'p', 'i', 'pwm', 'load', 'recovery', 'still_ms', 'moving_ms', 'overspeed_ms', 'recovery_ms', 'no_motion_ms', 'recovering', 'exhausted', 'failed']
target = 0.2*0.210/2*1560/(math.pi*0.065)
report = {'kind': 'inferred_private_state_from_frozen_C_not_hardware_private_telemetry_or_a_physical_plant', 'wheels': {}}


def sequence(name, sign, fraction):
    value = sign*(abs(frames[0]['wheels'][name]['cps'])+fraction)
    result = [value]
    last_motion = frames[0]['t']
    for r in frames[1:]:
        w = r['wheels'][name]
        value += 20/70*(w['raw']-value)
        if w['dc']:
            last_motion = r['t']
        elif r['t']-last_motion >= 200:
            value = 0
        result.append(value)
    return result


for name, index, sign, base in [('LF', 0, 1, 51.8+0.146*target), ('RF', 2, -1, 56.8+0.14249*target)]:
    observed = [r['wheels'][name] for r in frames]
    first = observed[0]
    candidates = []
    for fi in range(1, 100, 5):
        speeds = sequence(name, sign, fi/100)
        if not all(int(v) == r['cps'] for r, v in zip(observed, speeds)):
            continue
        for li in range(5, 100, 10):
            load = abs(first['ff_x10'])/10+li/1000-base
            for ii in range(5, 100, 10):
                integral = sign*(abs(first['i_x10'])/10+ii/1000)
                text = f'{index} {frames[0]["t"]} {load:.9f} {integral:.9f} {speeds[0]:.9f}\n'
                text += ''.join(f'{r["t"]} {v:.9f}\n' for r, v in zip(frames[1:], speeds[1:]))
                run = subprocess.run([str(exe)], input=text, capture_output=True, text=True)
                assert run.returncode == 0, run.stderr
                rows = [dict(zip(fields, map(float, row))) for row in csv.reader(io.StringIO(run.stdout))]
                errors = {k: max(abs(r[k]-w[source]/10) for r, w in zip(rows, observed))
                          for k, source in [('ff', 'ff_x10'), ('p', 'p_x10'), ('i', 'i_x10')]}
                mismatches = sum(int(r['pwm']) != w['pwm'] for r, w in zip(rows, observed))
                max_pwm = max(abs(r['pwm']-w['pwm']) for r, w in zip(rows, observed))
                candidates.append((mismatches*10+sum(errors.values()), fi/100, load, integral, errors, mismatches, max_pwm, rows, run.stdout))
    assert candidates, name
    best = min(candidates, key=lambda c: c[0])
    _, fraction, load, integral, errors, mismatches, max_pwm, rows, csv_text = best
    assert max(errors.values()) < 0.12 and max_pwm <= 1, (name, errors, mismatches, max_pwm)
    compatible = [c for c in candidates if max(c[4].values()) < 0.12 and c[6] <= 1]
    signatures = {tuple((int(r['t']), int(r['recovering']), int(r['overspeed_ms'])) for r in c[7]) for c in compatible}
    (folder/f'{name.lower()}_inferred_replay.csv').write_text(','.join(fields)+'\n'+csv_text, encoding='utf-8')
    starts, exits, withdrawals, handoffs = [], [], [], []
    for previous, r in zip(rows, rows[1:]):
        if not previous['recovering'] and r['recovering']:
            starts.append(int(r['t']))
        if previous['recovering'] and not r['recovering']:
            exits.append({'t': int(r['t']), 'directed_speed_cps': r['measured']*sign,
                          'remaining_transient': r['recovery'], 'directed_i': r['i']*sign})
        if previous['load']-r['load'] > 0.1:
            withdrawals.append({'t': int(r['t']), 'speed': r['measured']*sign,
                'raw_cps': observed[rows.index(r)]['raw']*sign,
                'load_removed': previous['load']-r['load'],
                'transient_removed': previous['recovery']-r['recovery'],
                'i_delta': (r['i']-previous['i'])*sign,
                'support_delta': (r['ff']+r['i']-previous['ff']-previous['i'])*sign})
        if previous['recovery']-r['recovery'] > 0.01 and r['measured']*sign <= target and r['moving_ms'] == 40:
            handoffs.append({'t': int(r['t']), 'speed': r['measured']*sign,
                             'support_delta': (r['ff']+r['i']-previous['ff']-previous['i'])*sign})
    checkpoints = [290403, 291883, 291943, 292023, 292143, 292243, 292343, 292363, 292383, 292403, 292443, 292483, 292543, 292623, 292803, 292923]
    report['wheels'][name] = {'samples': len(rows), 'tested_candidates': len(candidates),
        'compatible_candidates_in_tested_family': len(compatible),
        'distinct_recovering_overspeed_signatures': len(signatures),
        'seed': {'load': load, 'i': integral, 'cps_fraction': fraction, 't': frames[0]['t'], 'transient': 0, 'moving_ms': 40},
        'max_error_permille_vs_printed': errors, 'pwm_mismatches': mismatches, 'max_pwm_difference': max_pwm,
        'pwm_mismatch_rows': [{'t': int(r['t']), 'replayed': int(r['pwm']), 'recorded': w['pwm']}
                              for r,w in zip(rows,observed) if int(r['pwm']) != w['pwm']],
        'recovery_start_times': starts, 'recovery_exits': exits, 'withdrawals': withdrawals,
        'moving_handoff_steps': handoffs, 'max_no_motion_ms': max(r['no_motion_ms'] for r in rows),
        'max_abs_FF_plus_I_delta_during_moving_handoff': max((abs(r['support_delta']) for r in handoffs),default=0),
        'checkpoints': [r for r in rows if int(r['t']) in checkpoints]}
report['limits'] = [
    'Initial load/I/filtered speed fitted within first printed bins; transient=0, moving_ms=40 assumed at window entry. Other private-state families are not excluded.',
    'Compatible candidates are only this grid. Not a uniqueness proof, Flash identity confirmation or hardware private-state telemetry.',
    'Recorded counts/feedback prescribed. No plant/current/friction/supply-transient model; no prediction of changed-controller response.',
    'A matching replay confirms compatibility of this branch sequence, not the sole physical cause or readiness of any proposed fix.'
]
report['sha256'] = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [capture/'trace_parsed.json', folder/'source_snapshot/wheel_speed_controller.c', folder/'front_replay.c']}
(folder/'replay_analysis.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps({n: {k:v for k,v in r.items() if k not in ('checkpoints','withdrawals','moving_handoff_steps')} for n,r in report['wheels'].items()}, indent=2))
