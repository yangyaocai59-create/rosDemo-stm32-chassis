"""Read existing sources/evidence and run isolated host tests. Never touches hardware.
All generated results stay here; production .22 and frozen firmware are read-only.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=next(p for p in HERE.parents if (p/'Application').is_dir())
OUT=ROOT/'build/tests/v01822_user_review'
OUT.mkdir(parents=True,exist_ok=True)
CC=shutil.which('gcc') or 'D:/x86_64-15.2.0-release-posix-seh-ucrt-rt_v13-rev1/mingw64/bin/gcc.exe'
env={**os.environ,'PATH':str(Path(CC).parent)+os.pathsep+os.environ['PATH']}
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((ROOT/'build/firmware/0.18.22/source_manifest.json').read_text())
proof=json.loads((ROOT/'build/firmware/0.18.22/verification.json').read_text())
def check_frozen():
    for name,h in manifest['files'].items(): assert sha(ROOT/name)==h,name
    for ext,h in proof['sha256'].items():
        assert sha(ROOT/f'build/firmware/0.18.22/rosDemo-0.18.22.{ext}')==h,ext
check_frozen()
def run(name,args):
    r=subprocess.run(list(map(str,args)),cwd=ROOT,env=env,capture_output=True,
      text=True,encoding='utf-8',errors='replace')
    text=r.stdout+r.stderr
    (HERE/(name+'.log')).write_text(text,encoding='utf-8')
    assert r.returncode==0,(name,text[-5000:])
    assert not re.search(r'\b(?:warning|error):',text,re.I),(name,text[-5000:])
    print(name+': PASS',flush=True)
    return text
inc=['-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','-IApplication/Safety',
     '-IApplication/Diagnostics','-IApplication/Services']
flags=['-std=c11','-Wall','-Wextra','-Werror']
summary={}
baseline=ROOT/'docs/stm32_evidence/2026-10-22-unified-pivot-v1/baseline_0.18.21'
for label,source,legacy in [('old21',baseline/'wheel_speed_controller.c',1),
 ('candidate22',ROOT/'Application/Chassis/wheel_speed_controller.c',0)]:
    binary=OUT/(label+'.exe')
    run(label+'_build',[CC]+flags+inc+[f'-DREVIEW_LEGACY={legacy}',
      HERE/'test_filter_and_effort.c',source,'-lm','-o',binary])
    output=run(label,[binary])
    summary[label]={'filter_summaries':[line.split(',')[1:] for line in output.splitlines() if line.startswith('SUMMARY,')],
      'p_gain':[line.split(',')[1:] for line in output.splitlines() if line.startswith('P_GAIN,')],
      'authority':[line.split(',')[1:] for line in output.splitlines() if line.startswith('AUTHORITY,')]}
binary=OUT/'trace.exe'
run('trace_build',[CC]+flags+inc+[HERE/'test_trace_review.c','-lm','-o',binary])
trace=run('trace',[binary])
summary['trace_capacity']=next(line.split(',')[1:] for line in trace.splitlines() if line.startswith('TRACE_CAPACITY,'))
runtime=['Application/Chassis/'+p+'.c' for p in (
 'chassis_runtime','chassis_controller','chassis_kinematics','chassis_motion_profile','wheel_speed_controller')]
binary=OUT/'fault_boundary.exe'
run('fault_boundary_build',[CC]+flags+inc+['-ITools/tests/motion',HERE/'test_fault_boundary.c']+
 runtime+['Application/Safety/fault_manager.c','-lm','-o',binary])
fault=run('fault_boundary',[binary])
summary['fault_boundaries']=[line.split(',')[1:] for line in fault.splitlines() if line.startswith('FAULT_BOUNDARY,')]
# Verify the input voltage range from named historical captures, not as a
# statement about today's rail or the unverified voltage of the original scan.
summary['historical_vm_mV']={}
for name,relative in [('long_left','docs/stm32_evidence/2026-10-22-v01821-long-left-repeat/capture/decoded.json'),
 ('practical_right','docs/stm32_evidence/2026-10-22-pivot-feedback-policy-prototype-v1/input_right/decoded.json')]:
    decoded=json.loads((ROOT/relative).read_text(encoding='utf-8'))
    # Decoded battery tuple: t, samples, errors, raw, filtered_raw,
    # adc_mV, SUPPLY_mV, state, flags, ... . Index1 is NOT voltage.
    values=[p['values'][6] for p in decoded.get('battery',[])]
    assert values
    summary['historical_vm_mV'][name]={'file':relative,'samples':len(values),'min':min(values),'max':max(values)}
run('candidate_regression',['python','Tools/tests/motion/run_unified_tests.py'])
usb=run('usb_regression',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File','Tools/tests/run_usb_tx_tests.ps1'])
summary['usb_groups']=len(re.findall(r'^PASS:',usb,re.M))
assert summary['usb_groups']==20
run('odometry_selftest',['python','Tools/wheel_odometry_test.py','--self-test'])
run('protocol_selftest',['python','Tools/usb_cdc_test.py','--self-test'])
check_frozen()
summary.update({'production_source_items_unchanged':len(manifest['files']),
 'candidate_artifacts_unchanged':True,'serial_opened':False,'motion_sent':False,
 'flashed':False,'hardware_validated':False,'physical_root_cause_confirmed':False})
(HERE/'verification.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
print('PASS: review complete, sources/artifacts unchanged; results in '+str(HERE/'verification.json'),flush=True)
