"""Offline prototype only: no serial, motion, ARM firmware link or flash."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Tools/tests/run_motion_tests.ps1').is_file())
prototype=here/'prototype'
baseline=here.parent/'2026-10-21-v01817-lf-start-review/source_snapshot'
out=root/'build/prototypes/lf_first_start_v1'
out.mkdir(parents=True,exist_ok=True)
manifest=json.loads((root/'build/firmware/0.18.17/source_manifest.json').read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def unchanged():
    assert all(sha(root/n)==s for n,s in manifest['files'].items())
    assert sha(root/'build/firmware/0.18.17/rosDemo-0.18.17.hex')=='234dc0c49fb80fc32c787783b59e1ea2be724cc100f21fc563d7fce7d558a42f'
unchanged()
for p in baseline.iterdir():
    assert sha(p)==manifest['files']['Application/Chassis/'+p.name]
logs=[]
def run(label,command):
    r=subprocess.run(list(map(str,command)),cwd=root,capture_output=True,
                     text=True,encoding='utf-8',errors='replace')
    output=r.stdout+r.stderr
    logs.append('$ '+label+'\n'+output)
    (here/'test_run.log').write_text('\n'.join(logs),encoding='utf-8')
    assert r.returncode==0,(label,output)
    assert not re.search(r'\b(?:warning|error):',output,re.I),output
    if output.strip(): print(label+': '+output.strip(),flush=True)
    return output
flags=['gcc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(prototype),'-I'+str(root),
       '-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','-IApplication/Safety',
       '-IApplication/Services','-IApplication/Diagnostics']
public=re.findall(r'^(?:bool|void|float)\s+(WheelSpeedController_\w+)\(',
                  (baseline/'wheel_speed_controller.h').read_text(),re.M)
assert len(public)>=10
run('frozen .17 object',flags+['-D'+n+'=Baseline_'+n for n in public]+
    ['-c',baseline/'wheel_speed_controller.c','-o',out/'baseline.o'])
run('prototype object',flags+['-c',prototype/'wheel_speed_controller.c','-o',out/'candidate.o'])
run('isolation build',flags+[here/'test_isolation.c',out/'baseline.o',out/'candidate.o','-lm','-o',out/'isolation.exe'])
outputs=[run('isolation',[out/'isolation.exe'])]
for name,source in [('first_start',here/'test_policy.c'),
                    ('coordination',here/'regression/test_coordination.c')]:
    run(name+' build',flags+[source,'-lm','-o',out/(name+'.exe')])
    outputs.append(run(name,[out/(name+'.exe')]))
runtime=['Application/Chassis/chassis_runtime.c','Application/Chassis/chassis_controller.c',
         'Application/Chassis/chassis_kinematics.c','Application/Chassis/chassis_motion_profile.c']
groups={
    'motion_core':['Tools/tests/motion/test_motion_core.c','Application/Chassis/chassis_motion_profile.c',
                   'Application/Safety/fault_manager.c','Application/Services/control_timing.c'],
    'runtime':['Tools/tests/motion/test_runtime_motion.c']+runtime+['Application/Diagnostics/motion_trace.c'],
    'front_pivot':['Tools/tests/motion/test_front_pivot.c'],
    'front_handoff':['Tools/tests/motion/test_front_support_handoff.c'],
    'load_confirmation':['Tools/tests/motion/test_front_load_confirmation.c'],
    'recovery_safety':['Tools/tests/motion/test_front_recovery_safety.c']+runtime+['Application/Safety/fault_manager.c']}
for name,sources in groups.items():
    run(name+' build',flags+sources+[out/'candidate.o','-lm','-o',out/(name+'.exe')])
    outputs.append(run(name,[out/(name+'.exe')]))
run('extended safety build',flags+[here/'test_extended_safety.c']+runtime+
    ['Application/Diagnostics/motion_trace.c','-lm','-o',out/'extended_safety.exe'])
outputs.append(run('extended safety',[out/'extended_safety.exe']))
run('motor driver build',['gcc','-std=c11','-Wall','-Wextra','-Werror',
    '-ITools/tests/motion/stubs','-IApplication/Chassis','-IApplication/Safety',
    'Tools/tests/motion/test_motor_driver.c','Application/Chassis/motor_driver.c','-o',out/'driver.exe'])
outputs.append(run('motor driver',[out/'driver.exe']))
native_pass=sum(len(re.findall(r'^PASS:',s,re.M)) for s in outputs)
assert native_pass==47,native_pass
usb=run('unchanged USB tests',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File','Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:',usb,re.M))==20
armbin=Path('C:/Users/yang/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin')
armflags=[armbin/'arm-none-eabi-gcc.exe','-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16',
          '-mfloat-abi=hard','-std=gnu11','-O0','-g3','-Wall','-Wextra','-Werror',
          '-fstack-usage','-ffunction-sections','-fdata-sections']
for label,source in [('baseline',baseline/'wheel_speed_controller.c'),('candidate',prototype/'wheel_speed_controller.c')]:
    run(label+' ARM object ONLY',armflags+['-c',source,'-o',out/(label+'_arm.o')])
size=run('ARM object size NOT firmware',[armbin/'arm-none-eabi-size.exe',out/'baseline_arm.o',out/'candidate_arm.o'])
stacks={name:[line for line in (out/(name+'_arm.su')).read_text().splitlines()
              if 'FrontSupport\t' in line or 'UpdateWithMode\t' in line] for name in ('baseline','candidate')}
run('fixed recorded feedback command comparison',[os.sys.executable,here/'compare_recorded.py'])
unchanged()
report={'kind':'OFFLINE_PROTOTYPE_ONLY_NOT_A_FIRMWARE_RELEASE','native_pass_groups':native_pass,
        'usb_pass_groups':20,'production_source_test_hashes_unchanged':len(manifest['files']),
        'production_hex_unchanged':True,'serial_opened':False,'motion_sent':False,
        'firmware_linked':False,'flashed':False,'hardware_tested':False,'warnings':0,
        'isolation_updates':120010,'ARM_object_size':size,'static_stack_lines':stacks,
        'files':{str(p.relative_to(here)):sha(p) for p in here.rglob('*')
                 if p.is_file() and p.name not in ('verification.json','test_run.log')},
        'limits':['Recorded/artificial feedback is not a plant model.',
                  'Only LF reverse first start changes; not confirmed moving LF/RF/rear/NORMAL.',
                  '600ms cap applies to special first-start extension; late entry keeps old 300ms path.',
                  'Object compilation is not linked firmware or hardware timing/stack proof.']}
(here/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='files'},indent=2),flush=True)
