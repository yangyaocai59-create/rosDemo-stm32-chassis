"""Offline frozen-C replay with inferred initial conditions; no serial APIs."""
import csv
import hashlib
import io
import json
import math
import pathlib
import subprocess

folder = pathlib.Path(__file__).resolve().parent
root = next(p for p in folder.parents if (p/'Tools/tests/run_motion_tests.ps1').is_file())
capture = folder.parent/'2026-10-21-v01814-ground-pivot/capture'
manifest = json.loads((root/'build/firmware/0.18.14/source_manifest.json').read_text(encoding='utf-8'))
for name in ('wheel_speed_controller.c', 'wheel_speed_controller.h', 'wheel_speed_control_config.h', 'motor_driver.h', 'chassis_config.h'):
    assert hashlib.sha256((folder/'source_snapshot'/name).read_bytes()).hexdigest() == manifest['files']['Application/Chassis/'+name]
exe = root/'build/tests/v01814_front_replay.exe'
compile_result = subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(folder/'front_replay.c'), '-lm', '-o', str(exe)],
                                capture_output=True, text=True)
assert compile_result.returncode == 0, compile_result.stdout+compile_result.stderr
synthetic = subprocess.run([str(exe), 'synthetic'], capture_output=True, text=True)
assert synthetic.returncode == 0, synthetic.stdout+synthetic.stderr
assert synthetic.stdout.count('RESULT ') == 4
(folder/'synthetic_probe.log').write_text(synthetic.stdout, encoding='utf-8')

fields = ['t', 'measured', 'ff', 'p', 'i', 'pwm', 'load', 'recovery', 'still_ms', 'moving_ms',
          'load_overspeed_ms', 'recovery_ms', 'no_motion_ms', 'recovering', 'exhausted', 'failed', 'hold']
trace = [r for r in json.loads((capture/'trace_parsed.json').read_text(encoding='utf-8')) if r['state'] == 2]
assert len(trace) == 127 and all(r['dt'] == 20 for r in trace)
target_magnitude = 0.2*0.210/2*1560/(math.pi*0.065)
report = {'kind': 'inferred_initial_state_replay_not_private_telemetry_or_a_physical_plant', 'wheel': {}}


def speed_sequence(name, fraction):
    sign = 1 if trace[0]['wheels'][name]['cps'] >= 0 else -1
    value = sign*(abs(trace[0]['wheels'][name]['cps'])+fraction)
    values = [value]
    last_motion = trace[0]['t']
    for r in trace[1:]:
        w = r['wheels'][name]
        value += 20/70*(w['raw']-value)
        if w['dc']:
            last_motion = r['t']
        elif r['t']-last_motion >= 200:
            value = 0
        values.append(value)
    return values


for name, index, seed_row, load in [('LF', 0, 0, 60), ('RF', 2, 2, 25)]:
    sign = 1 if name == 'LF' else -1
    observed = trace[seed_row:]
    target = sign*target_magnitude
    wire_i = observed[0]['wheels'][name]['i_x10']/10
    candidates = []
    for fraction_i in range(1, 100, 5):
        fraction = fraction_i/100
        speeds = speed_sequence(name, fraction)
        if not all(int(v) == r['wheels'][name]['cps'] for r, v in zip(trace, speeds)):
            continue
        for offset_i in range(1, 100, 5):
            offset = offset_i/1000
            integral = wire_i+offset if wire_i >= 0 else wire_i-offset
            header = f'{index} {observed[0]["t"]} {target:.9f} {load} {integral:.9f} {speeds[seed_row]:.9f}\n'
            text = header+''.join(f'{r["t"]} {v:.9f}\n' for r, v in zip(observed[1:], speeds[seed_row+1:]))
            result = subprocess.run([str(exe)], input=text, capture_output=True, text=True)
            assert result.returncode == 0, result.stderr
            rows = [dict(zip(fields, map(float, row))) for row in csv.reader(io.StringIO(result.stdout))]
            assert len(rows) == len(observed)
            errors = {key: max(abs(row[key]-r['wheels'][name][source]/10) for row, r in zip(rows, observed))
                      for key, source in [('ff', 'ff_x10'), ('p', 'p_x10'), ('i', 'i_x10')]}
            mismatch = sum(int(row['pwm']) != r['wheels'][name]['pwm'] for row, r in zip(rows, observed))
            largest_pwm = max(abs(row['pwm']-r['wheels'][name]['pwm']) for row, r in zip(rows, observed))
            score = mismatch*10+sum(errors.values())
            candidates.append((score, fraction, integral, errors, mismatch, largest_pwm, rows, result.stdout))
    assert candidates, name
    best = min(candidates, key=lambda c: c[0])
    _, fraction, integral, errors, mismatch, largest_pwm, rows, csv_text = best
    assert max(errors.values()) < 0.12 and largest_pwm <= 1, (name, errors, mismatch, largest_pwm)
    compatible = [c for c in candidates if max(c[3].values()) < 0.12 and c[5] <= 1]
    signatures = {tuple((int(row['t']), int(row['recovering']), int(row['hold'])) for row in c[6]) for c in compatible}
    (folder/f'{name.lower()}_inferred_replay.csv').write_text(','.join(fields)+'\n'+csv_text, encoding='utf-8')
    starts, exits = [], []
    for prev, row in zip(rows, rows[1:]):
        if not prev['recovering'] and row['recovering']:
            starts.append(int(row['t']))
        if prev['recovering'] and not row['recovering']:
            speed = row['measured']*sign
            exits.append({'t': int(row['t']), 'directed_speed_cps': speed,
                          'moving_ms': int(row['moving_ms']), 'recovery_age_ms': int(row['recovery_ms']),
                          'remaining_transient_permille': row['recovery'],
                          'I_directed_permille': row['i']*sign,
                          'still_underspeed': speed < target_magnitude})
    under_hold = [row for row in rows if row['hold'] and 20 < row['measured']*sign < target_magnitude]
    report['wheel'][name] = {'seed_t': observed[0]['t'], 'seed_load_permille': load,
                             'fitted_initial_printed_bin_fraction': fraction,
                             'fitted_I_at_seed': integral, 'fit_samples': len(rows),
                             'tested_candidates': len(candidates), 'compatible_candidates': len(compatible),
                             'distinct_recovering_hold_signatures_among_compatible': len(signatures),
                             'max_abs_error_permille_vs_printed': errors,
                             'pwm_rounding_mismatches': mismatch, 'max_pwm_discrepancy_permille': largest_pwm,
                             'recovery_start_times': starts, 'recovery_exit_events': exits,
                             'moving_underspeed_integral_held_samples': len(under_hold),
                             'moving_underspeed_integral_held_accounted_ms': len(under_hold)*20,
                             'max_no_motion_ms': max(row['no_motion_ms'] for row in rows)}
report['limits'] = [
    'Initial private state is inferred, not directly observed. LF starts from full load 60, RF from load25 after initial overspeed; both begin recovery=0/moving_ms=40.',
    'Grid fitting selects values within first printed cps/I bins; compatible candidates are only this tested family, not proof of unique private state.',
    'Same recorded feedback is prescribed; there is no motor, ground friction, driver current, supply-transient or changed-controller response model.',
    'Replay fit supports the identified existing branch sequence, not sole physical cause or proof that a proposed policy will improve real motion.'
]
report['sha256'] = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [capture/'trace_parsed.json', folder/'source_snapshot/wheel_speed_controller.c', folder/'front_replay.c']}
(folder/'replay_analysis.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
