"""OFFLINE prototype regression and object compilation; never hardware."""
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import re
import subprocess

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Application').is_dir())
proto=here/'prototype'
golden=here.parent/'2026-10-22-v01818-lr-rf-review/source_snapshot'
out=root/'build/prototypes/lr_running_v2'
out.mkdir(parents=True,exist_ok=True)
manifest=json.loads((root/'build/firmware/0.18.18/source_manifest.json').read_text())
artifacts=json.loads((root/'build/firmware/0.18.18/verification.json').read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def unchanged():
    assert all(sha(root/n)==h for n,h in manifest['files'].items())
    assert all(sha(root/f'build/firmware/0.18.18/rosDemo-0.18.18.{s}')==h for s,h in artifacts['sha256'].items())
unchanged()
changed=[p.name for p in proto.iterdir() if not (golden/p.name).exists() or sha(p)!=sha(golden/p.name)]
assert set(changed)=={'chassis_controller.c','wheel_speed_controller.c','wheel_speed_controller.h','wheel_speed_control_config.h','fault_manager.c'},changed
# v1 control algorithm is frozen. v2 adds a read-only query and fault consumer.
v1=here.parent/'2026-10-22-lr-running-prototype-v1'
for stem in ('chassis_controller.c','wheel_speed_control_config.h'):
    assert sha(proto/stem)==sha(v1/'prototype'/stem)
production_manager=(root/'Application/Safety/fault_manager.c').read_text()
prototype_manager=(proto/'fault_manager.c').read_text()
old_predicate=re.search(r'const bool stall_condition =\s*(.*?);',production_manager,re.S)[1]
retained_predicate=re.search(r'const bool legacy_stall_condition =\s*(.*?);',prototype_manager,re.S)[1]
assert old_predicate==retained_predicate
stall_constants=r'^#define FAULT_MANAGER_STALL_.*$'
assert re.findall(stall_constants,production_manager,re.M)==re.findall(stall_constants,prototype_manager,re.M)
logs=[]
def run(name,cmd,stdin=None,save=None):
    r=subprocess.run(list(map(str,cmd)),cwd=root,capture_output=True,text=True,
                     encoding='utf-8',errors='replace',input=stdin)
    assert r.returncode==0,(name,r.stdout[-2000:],r.stderr[-3000:])
    assert not re.search(r'\b(?:warning|error):',r.stdout+r.stderr,re.I)
    if save: Path(save).write_text(r.stdout)
    logs.append(name+'\n'+(r.stdout if len(r.stdout)<16000 else
        f'large output: {len(r.stdout.splitlines())} lines sha256={hashlib.sha256(r.stdout.encode()).hexdigest()}\n')+r.stderr)
    (here/'test_run.log').write_text('\n'.join(logs))
    print(name+': PASS',flush=True)
    return r.stdout
flags=['gcc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(proto),
       '-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','-IApplication/Safety',
       '-IApplication/Services','-IApplication/Diagnostics']
runtime=['Application/Chassis/chassis_runtime.c','Application/Chassis/chassis_kinematics.c',
         'Application/Chassis/chassis_motion_profile.c','Application/Diagnostics/motion_trace.c']
for name in ('policy','safety'):
    run(name+' build',flags+[here/f'test_{name}.c']+runtime+['-lm','-o',out/(name+'.exe')])
    run(name,[out/(name+'.exe')])
run('fault linkage build',flags+[here/'test_fault_link.c']+runtime[:3]+[
    proto/'fault_manager.c','-lm','-o',out/'fault_link.exe'])
linkage=run('fault linkage',[out/'fault_link.exe'])
assert len(re.findall(r'^PASS:',linkage,re.M))==4
run('actual front safety regression build',flags+['Tools/tests/motion/test_front_recovery_safety.c',
    proto/'chassis_controller.c',proto/'wheel_speed_controller.c']+runtime[:3]+[
    proto/'fault_manager.c','-lm','-o',out/'front_safety.exe'])
run('actual front safety regression',[out/'front_safety.exe'])
streams={}
for label,extra in [('v1',['-DCONTROL_V1']),('v2',[])]:
    run(label+' running control identity build',flags+extra+[here/'control_identity.c']+
      runtime+['-lm','-o',out/(label+'_control.exe')])
    streams[label]=run(label+' running control identity',[out/(label+'_control.exe')])
assert streams['v1']==streams['v2'] and len(streams['v2'].splitlines())==30000
run('candidate runtime compatibility subset build',flags+[here/'runtime_compat.c',proto/'chassis_controller.c',
    proto/'wheel_speed_controller.c']+runtime+['-lm','-o',out/'runtime_compat.exe'])
runtime_subset=run('candidate runtime compatibility subset',[out/'runtime_compat.exe'])
assert len(re.findall(r'^PASS:',runtime_subset,re.M))==15
for label,extra in [('baseline',['-DBASELINE']),('candidate',[])]:
    run(label+' isolation build',flags+extra+[here/'isolation.c']+runtime+['-lm','-o',out/(label+'.exe')])
for label,arg,count in [('isolation',[],80000),('startup',['startup'],10000)]:
    old=run(label+' frozen',[out/'baseline.exe']+arg)
    new=run(label+' prototype',[out/'candidate.exe']+arg)
    assert old==new,label+' differential mismatch'
    assert len(new.splitlines())==count
    print(f'{label}: {count} byte-identical updates',flush=True)
baseline_motion=run('unchanged production motion regression',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File','Tools/tests/run_motion_tests.ps1'])
usb=run('unchanged production USB regression',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File','Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:',baseline_motion,re.M))==47
assert len(re.findall(r'^PASS:',usb,re.M))==20
# Original direct-wheel tests do not activate the new chassis/PI handshake;
# they remain applicable without editing any production assertion.
for name,source in [('front_pivot','test_front_pivot.c'),('front_handoff','test_front_support_handoff.c'),
                    ('load_confirmation','test_front_load_confirmation.c')]:
    run(name+' candidate build',flags+['Tools/tests/motion/'+source,proto/'wheel_speed_controller.c','-lm','-o',out/(name+'.exe')])
    run(name+' candidate',[out/(name+'.exe')])
# Keep the old complete runtime harness as an explicit compatibility check;
# scoped LR restart assertions are allowed to fail, never claimed as passing.
r=subprocess.run(list(map(str,flags+['Tools/tests/motion/test_runtime_motion.c',proto/'chassis_controller.c',
    proto/'wheel_speed_controller.c']+runtime+['-lm','-o',out/'legacy_runtime.exe'])),cwd=root,capture_output=True,text=True)
assert r.returncode==0,r.stderr
r=subprocess.run([str(out/'legacy_runtime.exe')],cwd=root,capture_output=True,text=True)
(here/'legacy_runtime_compatibility.log').write_text(r.stdout+r.stderr)
legacy_runtime_pass=r.returncode==0
assert not legacy_runtime_pass
assert 'fabsf(initial_bias[w]-expected)<0.001f' in r.stderr+r.stdout
assert 'line 135' in r.stderr+r.stdout
print('legacy runtime full assertions: '+('PASS' if legacy_runtime_pass else 'NOT COMPATIBLE; scoped old LR expectation retained in log'),flush=True)
run('replay build',flags+[here/'replay.c']+runtime+['-lm','-o',out/'replay.exe'])
frames=[f for f in json.loads((here.parent/'2026-10-22-v01818-ground-left/capture/trace_parsed.json').read_text()) if f['state']==2]
inputs=''.join(f'{f["t"]} {f["wheels"]["LR"]["dc"]} {f["wheels"]["RF"]["dc"]} '
  f'{f["wheels"]["LR"]["cps"]} {f["wheels"]["LR"]["i_x10"]} {f["wheels"]["LR"]["pwm"]} '
  f'{f["wheels"]["RF"]["cps"]} {f["wheels"]["RF"]["ff_x10"]} {f["wheels"]["RF"]["i_x10"]} {f["wheels"]["RF"]["pwm"]}\n' for f in frames)
replay=run('same recorded feedback command comparison',[out/'replay.exe','0'],inputs,here/'recorded_commands.csv')
rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(replay))]
assert len(rows)==127
old_rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader((here.parent/'2026-10-22-v01818-lr-rf-review/replay_mid.csv').open())]
for old,new in zip(old_rows,rows):
    assert all(old[k]==new[k] for k in old if k.startswith('rf_'))
comparison={'kind':'FIXED_FEEDBACK_COMMANDS_NOT_WHEEL_PREDICTION',
 'new_episode_first_ms':next(int(v['t']) for v in rows if v['lr_active']),
 'old_restart_first_ms':next(int(v['t']) for v in old_rows if v['lr_restart_active']),
 'peak_temporary_permille':max(v['lr_temporary'] for v in rows),
 'new_max_pwm_abs':max(abs(v['lr_pwm']) for v in rows),
 'old_max_pwm_abs':max(abs(v['lr_pwm']) for v in old_rows),
 'RF_conditional_replay_bit_identical':True}
(here/'recorded_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
fault_trip={}
for label,extra in [('baseline',['-DBASELINE']),('candidate',[])]:
    manager='Application/Safety/fault_manager.c' if label=='baseline' else proto/'fault_manager.c'
    run(label+' automatic fault audit build',flags+extra+[here/'audit_fault.c']+
      runtime[:3]+[manager,'-lm','-o',out/(label+'_fault.exe')])
    fault_trip[label]={}
    for seed in (-50,-20,0,6.7,20,50):
        fault_output=run(label+' automatic fault audit I='+str(seed),
          [out/(label+'_fault.exe'),str(seed)],
          save=here/(label+'_automatic_fault_I'+str(seed)+'.log'))
        fault_trip[label][str(seed)]=int(re.search(r'RESULT trip_from_stall_ms=(\d+)',fault_output)[1])
fault_delays={k:fault_trip['candidate'][k]-v for k,v in fault_trip['baseline'].items()}
grid=[]
for base in (0,0xFFFFFC00):
    for seed in range(-50,51):
        pair={}
        for label in ('baseline','candidate'):
            output=run(label+' grid I='+str(seed)+' base='+str(base),
                [out/(label+'_fault.exe'),str(seed),str(base)])
            pair[label]=int(re.search(r'RESULT trip_from_stall_ms=(\d+)',output)[1])
        assert pair['candidate']<=pair['baseline'],(base,seed,pair)
        grid.append({'base':base,'seed_i':seed,**pair})
(here/'fault_grid.json').write_text(json.dumps(grid,indent=2)+'\n')
arm=Path('C:/Users/yang/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin')
arm_flags=[arm/'arm-none-eabi-gcc.exe','-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16',
 '-mfloat-abi=hard','-std=gnu11','-O0','-g3','-Wall','-Wextra','-Werror','-fstack-usage',
 '-ffunction-sections','-fdata-sections','-DUSE_HAL_DRIVER','-DSTM32F407xx',
 '-ICore/Inc','-IDrivers/STM32F4xx_HAL_Driver/Inc','-IDrivers/CMSIS/Device/ST/STM32F4xx/Include',
 '-IDrivers/CMSIS/Include','-IMiddlewares/Third_Party/FreeRTOS/Source/include',
 '-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F',
 '-IApplication/Chassis','-IApplication/Safety','-IApplication/Services']
for label,folder in [('baseline',golden),('candidate',proto)]:
    for stem in ('wheel_speed_controller','chassis_controller','fault_manager'):
        source=(root/'Application/Safety/fault_manager.c') if label=='baseline' and stem=='fault_manager' else folder/(stem+'.c')
        run(label+' '+stem+' ARM OBJECT ONLY',arm_flags+['-c',source,'-o',out/(label+'_'+stem+'.o')])
size=run('ARM object size NOT firmware',[arm/'arm-none-eabi-size.exe']+sorted(out.glob('*.o')))
stack={}
for label in ('baseline','candidate'):
    for stem in ('wheel_speed_controller','chassis_controller','fault_manager'):
        su=out/(label+'_'+stem+'.su')
        # Compiler-generated diagnostics, not hand-edited source artifacts.
        (here/(label+'_'+stem+'.su')).write_bytes(su.read_bytes())
        for line in su.read_text().splitlines():
            fields=line.split('\t')
            stack[label+':'+fields[0].split(':')[-1]]=int(fields[1])
unchanged()
report={'kind':'OFFLINE_PROTOTYPE_ONLY_NOT_A_FIRMWARE_RELEASE',
 'acceptance':'REJECTED_SAFETY_TIMING_REGRESSION' if any(v>0 for v in fault_delays.values()) else 'OFFLINE_SAFETY_LINK_CHECKS_PASSED_NOT_HARDWARE_VALIDATED',
 'production_motion_pass_groups':47,'production_usb_pass_groups':20,
 'unchanged_scope_bit_identical_updates':80000,'first_start_steady_updates_identical':10000,
 'prototype_policy_groups':5,'prototype_safety_groups':1,'prototype_front_regression_groups':11,
 'fault_link_boundary_groups':4,'actual_front_safety_regression_groups':2,
 'v1_v2_running_control_identical_updates':30000,'automatic_fault_grid_cases':len(grid),
 'grid_max_added_delay_ms':max(v['candidate']-v['baseline'] for v in grid),
 'prototype_runtime_compatibility_subset_groups':15,
 'legacy_runtime_full_suite_pass':legacy_runtime_pass,'recorded_comparison':comparison,
 'automatic_fault_audit_trip_after_stall_ms':fault_trip,
 'automatic_fault_audit_added_delay_ms':fault_delays,
 'automatic_stall_timing_not_delayed':all(v<=0 for v in fault_delays.values()),
 'legacy_stall_predicate_and_constants_verbatim_preserved':True,
 'ARM_static_function_stack_bytes':stack,
 'ready_for_production_integration':False,
 'production_source_test_hashes_unchanged':141,'four_artifacts_unchanged':True,
 'serial_opened':False,'motion_sent':False,'fault_cleared':False,'firmware_linked':False,
 'flashed':False,'hardware_tested':False,'ARM_object_sizes':size,
 'prototype_changed_files':changed,
 'files':{str(p.relative_to(here)):sha(p) for p in here.rglob('*') if p.is_file()
          and p.name not in ('verification.json','test_run.log')},
 'limits':['Fixed feedback does not predict the wheel response.',
  'New ownership requires zero legacy bias; retained legacy support keeps old behavior.',
  'New safety path still requires target >=200cps; lower target LR coverage is not expanded.',
  'Output gate is intentionally bypassed only for owned LR reverse running failure: this tightens protection and may stop a sustained underpowered maneuver earlier.',
  'Baseline 47 groups are not all prototype runtime tests; see compatibility log.',
  'Only native executables and unlinked ARM objects were compiled.']}
(here/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
logs.append('OVERALL ACCEPTANCE: '+report['acceptance']+'\n'+
            json.dumps({'automatic_fault_audit_added_delay_ms':fault_delays,
                        'ready_for_production_integration':False},indent=2)+'\n')
(here/'test_run.log').write_text('\n'.join(logs))
print(json.dumps({k:v for k,v in report.items() if k!='files'},indent=2),flush=True)
if any(v>0 for v in fault_delays.values()):
    print('REJECTED: automatic stall qualification delayed. Do not integrate or generate firmware.',flush=True)
    raise SystemExit(2)
