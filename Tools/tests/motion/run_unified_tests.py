"""Host-only .22 candidate regression. Never opens ports or flashes hardware.

Old per-wheel pivot waveform/source-identity tests are legacy-only; this runner
does not relabel them as evidence for the new candidate control law.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'build/tests/unified_pivot_logs'
OUT.mkdir(parents=True, exist_ok=True)
CC = os.environ.get('CC') or shutil.which('gcc') or 'gcc'
RESULTS = {}

def run(name, command):
    result = subprocess.run(list(map(str, command)), cwd=ROOT, capture_output=True,
                            text=True, encoding='utf-8', errors='replace')
    output = result.stdout + result.stderr
    (OUT / (name + '.log')).write_text(output, encoding='utf-8')
    if result.returncode or re.search(r'\b(?:warning|error):', output, re.I):
        raise RuntimeError(name + ': ' + output[-6000:])
    groups = len(re.findall(r'^PASS:', output, re.M))
    RESULTS[name] = {'exit': result.returncode, 'pass_groups': groups,
                     'sha256': hashlib.sha256(output.encode()).hexdigest()}
    print(f'{name}: PASS ({groups} assertion groups)', flush=True)
    return output

includes = ['-ITools/tests/imu_spi_probe/stubs', '-IApplication/Chassis',
            '-IApplication/Safety', '-IApplication/Services', '-IApplication/Diagnostics']
runtime = ['Application/Chassis/' + p + '.c' for p in (
    'chassis_runtime', 'chassis_controller', 'chassis_kinematics',
    'chassis_motion_profile', 'wheel_speed_controller')]
flags = ['-std=c11', '-Wall', '-Wextra', '-Werror']
for label, extra in [('strict', []), ('optimized', ['-O2']), ('analyzer', ['-fanalyzer'])]:
    for name, sources in [
        ('pi', ['Tools/tests/motion/test_unified_pivot.c']),
        ('runtime', ['Tools/tests/motion/test_unified_runtime.c'] + runtime +
                    ['Application/Safety/fault_manager.c'])]:
        binary = ROOT / f'build/tests/unified_{name}_{label}.exe'
        run(name + '_' + label + '_build', [CC] + flags + extra + includes + sources + ['-lm', '-o', binary])
        run(name + '_' + label, [binary])

# NORMAL output is compared byte for byte against sealed .21 source.
baseline = ROOT / 'docs/stm32_evidence/2026-10-22-unified-pivot-v1/baseline_0.18.21'
outputs = []
for label, source, inc in [
    ('baseline', baseline / 'wheel_speed_controller.c', ['-I' + str(baseline)]),
    ('candidate', 'Application/Chassis/wheel_speed_controller.c', [])]:
    binary = ROOT / f'build/tests/unified_normal_{label}.exe'
    run('normal_' + label + '_build', [CC] + flags + inc + includes +
        ['Tools/tests/motion/unified_normal_compare.c', source, '-lm', '-o', binary])
    outputs.append(run('normal_' + label, [binary]))
assert outputs[0] == outputs[1], 'NORMAL outputs differ from frozen .21'
assert len(outputs[0].splitlines()) == 40000
RESULTS['normal_identity'] = {'updates': 40000, 'byte_identical': True}
print('NORMAL differential: PASS, 40000 updates identical', flush=True)

binary = ROOT / 'build/tests/unified_driver_test.exe'
run('driver_build', [CC] + flags + ['-ITools/tests/motion/stubs'] + includes[1:] +
    ['Tools/tests/motion/test_motor_driver.c', 'Application/Chassis/motor_driver.c', '-o', binary])
run('driver', [binary])

(OUT / 'results.json').write_text(json.dumps(RESULTS, indent=2) + '\n', encoding='utf-8')
print('PASS: unified candidate regression complete; prescribed feedback is not loaded hardware validation')
