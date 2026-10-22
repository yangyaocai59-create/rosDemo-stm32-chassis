"""Offline prototype only. No serial imports, GPIO access, firmware link or flash."""
import hashlib
import json
import os
import pathlib
import re
import subprocess

folder = pathlib.Path(__file__).resolve().parent
root = next(p for p in folder.parents if (p/'Tools/tests/run_motion_tests.ps1').is_file())
prototype = folder/'prototype'
baseline = folder.parent/'2026-10-21-v01815-lf-cycle-review/source_snapshot'
out = root/'build/prototypes/lf_coordination_v1'
out.mkdir(parents=True, exist_ok=True)
manifest = json.loads((root/'build/firmware/0.18.15/source_manifest.json').read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def unchanged():
    assert all(digest(root/name) == old for name,old in manifest['files'].items())
    assert digest(root/'build/firmware/0.18.15/rosDemo-0.18.15.hex') == '55e8b5b9563822dcf1595b1fa86d321dc6ebeaea81f7608c7ffbdbef184b2ba5'


unchanged()
for name in ('wheel_speed_controller.c', 'wheel_speed_controller.h', 'wheel_speed_control_config.h', 'motor_driver.h', 'chassis_config.h'):
    assert digest(baseline/name) == manifest['files']['Application/Chassis/'+name]

logs = []


def run(label, command):
    result = subprocess.run(list(map(str, command)), cwd=root, capture_output=True,
                            text=True, encoding='utf-8', errors='replace')
    output = result.stdout+result.stderr
    logs.append(f'$ {label}\n{output}')
    (folder/'test_run.log').write_text('\n'.join(logs), encoding='utf-8')
    assert result.returncode == 0, f'{label}: {output}'
    assert not re.search(r'\b(?:warning|error):', output, re.I), output
    if output.strip():
        print(label+': '+output.strip(), flush=True)
    return output


flags = ['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I'+str(prototype),
         '-ITools/tests/imu_spi_probe/stubs', '-IApplication/Chassis',
         '-IApplication/Safety', '-IApplication/Services', '-IApplication/Diagnostics']
public = re.findall(r'^(?:bool|void|float)\s+(WheelSpeedController_\w+)\(',
                    (baseline/'wheel_speed_controller.h').read_text(), re.M)
assert len(public) >= 10
run('prefixed frozen baseline', flags + ['-D'+n+'=Baseline_'+n for n in public] +
    ['-c', baseline/'wheel_speed_controller.c', '-o', out/'baseline.o'])
run('prototype object', flags + ['-c', prototype/'wheel_speed_controller.c', '-o', out/'candidate.o'])
run('isolation build', flags+[folder/'test_isolation.c', out/'baseline.o', out/'candidate.o', '-lm', '-o', out/'isolation.exe'])
outputs = [run('isolation', [out/'isolation.exe'])]
run('policy build', flags+[folder/'test_policy.c', '-lm', '-o', out/'policy.exe'])
outputs.append(run('policy', [out/'policy.exe']))

runtime = ['Application/Chassis/chassis_runtime.c', 'Application/Chassis/chassis_controller.c',
           'Application/Chassis/chassis_kinematics.c', 'Application/Chassis/chassis_motion_profile.c']
groups = {
    'motion_core': ['Tools/tests/motion/test_motion_core.c', 'Application/Chassis/chassis_motion_profile.c',
                    'Application/Safety/fault_manager.c', 'Application/Services/control_timing.c'],
    'runtime': ['Tools/tests/motion/test_runtime_motion.c']+runtime+['Application/Diagnostics/motion_trace.c'],
    'front_pivot': [folder/'regression/test_front_pivot.c'],
    'front_handoff': [folder/'regression/test_front_support_handoff.c'],
    'load_confirmation': [folder/'regression/test_front_load_confirmation.c'],
    'recovery_safety': ['Tools/tests/motion/test_front_recovery_safety.c']+runtime+['Application/Safety/fault_manager.c'],
}
for name,sources in groups.items():
    run(name+' build', flags+sources+[out/'candidate.o', '-lm', '-o', out/(name+'.exe')])
    outputs.append(run(name, [out/(name+'.exe')]))
run('motor driver build', ['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
    '-ITools/tests/motion/stubs', '-IApplication/Chassis', '-IApplication/Safety',
    'Tools/tests/motion/test_motor_driver.c', 'Application/Chassis/motor_driver.c', '-o', out/'driver.exe'])
outputs.append(run('motor driver', [out/'driver.exe']))
native_pass = sum(len(re.findall(r'^PASS:', s, re.M)) for s in outputs)
assert native_pass == 42, native_pass
usb = run('unchanged USB regression', ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', 'Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:', usb, re.M)) == 20

armbin = pathlib.Path('C:/Users/yang/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin')
armflags = [armbin/'arm-none-eabi-gcc.exe', '-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16',
            '-mfloat-abi=hard', '-std=gnu11', '-O0', '-g3', '-Wall', '-Wextra', '-Werror',
            '-fstack-usage', '-ffunction-sections', '-fdata-sections']
for name,source in [('baseline', baseline/'wheel_speed_controller.c'), ('candidate', prototype/'wheel_speed_controller.c')]:
    run(name+' ARM object ONLY', armflags+['-c', source, '-o', out/(name+'_arm.o')])
size = run('ARM object size, NOT linked firmware', [armbin/'arm-none-eabi-size.exe', out/'baseline_arm.o', out/'candidate_arm.o'])
stack = {}
for name in ('baseline', 'candidate'):
    text = (out/(name+'_arm.su')).read_text()
    stack[name] = [line for line in text.splitlines() if 'FrontSupport\t' in line or 'UpdateWithMode\t' in line]
    assert all(line.endswith('static') for line in stack[name])
unchanged()
report = {'kind': 'OFFLINE_PROTOTYPE_ONLY_NOT_A_FIRMWARE_RELEASE', 'native_pass_groups': native_pass,
    'usb_pass_groups': 20, 'warnings': 0, 'production_source_test_hashes_unchanged': len(manifest['files']),
    'production_hex_unchanged': True, 'serial_opened': False, 'motion_sent': False,
    'firmware_linked': False, 'flashed': False, 'hardware_tested': False,
    'isolation_steps': 80000, 'ARM_object_sizes': size, 'static_stack_lines': stack,
    'prototype_hashes': {p.name:digest(p) for p in prototype.iterdir() if p.is_file()},
    'files': {str(p.relative_to(folder)):digest(p) for p in folder.rglob('*')
              if p.is_file() and p.name not in ('verification.json','test_run.log')},
    'limits': ['Prescribed feedback/artificial private seeds are not a physical plant.',
               'Aggregate support budgets exclude P/error-I, sign/mode/target changes and opposing-I reset.',
               'Object compilation is NOT a complete linked ARM firmware or on-board stack/timing proof.',
               'Lower coordinated rise/slower load withdrawal can alter breakaway and unloading transients.']}
(folder/'verification.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('files','prototype_hashes')}, indent=2), flush=True)
