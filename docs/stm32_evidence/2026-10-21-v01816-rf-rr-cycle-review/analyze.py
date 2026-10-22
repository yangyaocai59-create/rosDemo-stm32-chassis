"""Offline frozen-C compatibility replay; no serial, firmware edits or flashing."""
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess

here = Path(__file__).resolve().parent
root = next(p for p in here.parents if (p / 'Tools/tests/run_motion_tests.ps1').is_file())
capture = here.parent / '2026-10-21-v01816-ground-pivot300/capture'
manifest = json.loads((root / 'build/firmware/0.18.16/source_manifest.json').read_text())
sources = list((here / 'source_snapshot').glob('*'))
assert len(sources) == 5
for p in sources:
    assert hashlib.sha256(p.read_bytes()).hexdigest() == manifest['files']['Application/Chassis/' + p.name]
exe = root / 'build/tests/v01816_rf_rr_review.exe'
subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(here / 'cycle_replay.c'),
                '-lm', '-o', str(exe)], check=True)
frames = [r for r in json.loads((capture / 'trace_parsed.json').read_text())
          if r['phase'] == 'LEFT_trace' and r['state'] == 2]
assert len(frames) == 127 and all(r['dt'] == 20 for r in frames)
fields = ['t', 'measured', 'ff', 'p', 'i', 'pwm', 'load', 'recovery',
          'still_ms', 'moving_ms', 'overspeed_ms', 'recovering', 'failed']
target = 0.3 * 0.210 / 2 * 1560 / (math.pi * 0.065)
report = {'scope': 'frozen unmodified C; recorded input, inferred anchor state; not a plant model',
          'wheels': {}}

for name, index, anchor, base in [('RF', 2, 974582, 54.7 + .14703 * target),
                                  ('RR', 3, 974482, 58.9 + .15779 * target)]:
    start = next(i for i, r in enumerate(frames) if r['t'] == anchor)
    segment = frames[start:]
    observed = [r['wheels'][name] for r in segment]
    candidates = []
    for fi in range(1, 100, 5):
        speed = frames[0]['wheels'][name]['cps'] + fi / 100
        last_motion = frames[0]['t']
        speeds = [speed]
        for r in frames[1:]:
            w = r['wheels'][name]
            speed += 20 / 70 * (w['raw'] - speed)
            if w['dc']:
                last_motion = r['t']
            elif r['t'] - last_motion >= 200:
                speed = 0
            speeds.append(speed)
        if not all(int(v) == r['wheels'][name]['cps'] for r, v in zip(frames, speeds)):
            continue
        for ii in range(5, 100, 10):
            first_i = observed[0]['i_x10'] / 10
            integral = first_i + (ii / 1000 if first_i >= 0 else -ii / 1000)
            # RR chassis bias is prescribed from recorded FF, NOT reconstructed
            # private rear state. RF anchor has known-zero support compatible with base FF.
            biases = [0 if name == 'RF' else r['ff_x10'] / 10 + .05 - base for r in observed]
            text = f'{index} {anchor} {integral:.9f} {speeds[start]:.9f} {biases[0]:.9f}\n'
            text += ''.join(f'{r["t"]} {v:.9f} {b:.9f}\n'
                            for r, v, b in zip(segment[1:], speeds[start + 1:], biases[1:]))
            run = subprocess.run([str(exe)], input=text, text=True, capture_output=True, check=True)
            rows = [dict(zip(fields, map(float, row))) for row in csv.reader(io.StringIO(run.stdout))]
            errors = {key: max(abs(r[key] - w[src] / 10) for r, w in zip(rows, observed))
                      for key, src in [('ff', 'ff_x10'), ('p', 'p_x10'), ('i', 'i_x10')]}
            pwm_mismatches = sum(int(r['pwm']) != w['pwm'] for r, w in zip(rows, observed))
            pwm_max = max(abs(r['pwm'] - w['pwm']) for r, w in zip(rows, observed))
            candidates.append((pwm_mismatches * 10 + sum(errors.values()), errors, pwm_mismatches,
                               pwm_max, rows, run.stdout, fi / 100, integral))
    assert candidates, name
    best = min(candidates, key=lambda c: c[0])
    _, errors, mismatches, pwm_max, rows, text, fraction, integral = best
    assert max(errors.values()) < .12 and pwm_max <= 1, (name, errors, mismatches, pwm_max)
    compatible = [c for c in candidates if max(c[1].values()) < .12 and c[3] <= 1]
    (here / (name.lower() + '_compatibility_replay.csv')).write_text(','.join(fields) + '\n' + text)
    report['wheels'][name] = {'anchor_ms': anchor, 'samples': len(rows), 'tested': len(candidates),
        'compatible': len(compatible), 'seed': {'initial_cps_fraction': fraction, 'integral': integral},
        'max_error_permille': errors, 'pwm_mismatches': mismatches, 'max_pwm_difference': pwm_max,
        'events': [r for r in rows if r['t'] <= 975342],
        'caveat': ('RF private state inferred from an anchor, not directly measured.' if name == 'RF'
                   else 'RR bias prescribed from printed FF; cannot prove rear private state or origin of initial I.')}

report['limits'] = ['No current, supply transient, friction or motor model; no prediction of a modified controller.',
                    'Within-bin initial state search is a compatibility check, not uniqueness or root-cause proof.',
                    'Printed target is truncated; C replay computes nominal geometry target at 300 mrad/s.']
report['sha256'] = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [capture / 'trace_parsed.json', here / 'cycle_replay.c', *sources]}
(here / 'analysis.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({name: {k: v for k, v in info.items() if k != 'events'}
                  for name, info in report['wheels'].items()}, indent=2))
