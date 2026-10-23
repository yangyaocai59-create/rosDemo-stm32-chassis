"""Production .21 RR exit tests; native stubs/prescribed feedback only.
No hardware, serial, reset, calibration, motion or programmer calls.
"""
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess

root=Path(__file__).resolve().parents[3]
out=root/'build/tests/rr_exit'
out.mkdir(parents=True,exist_ok=True)
reference=root/'docs/stm32_evidence/2026-10-22-rr-exit-prototype-v1'
baseline=root/'docs/stm32_evidence/2026-10-22-v01821-offline/baseline_0.18.20'
manifest=json.loads((root/'build/firmware/0.18.20/source_manifest.json').read_text())
proof=json.loads((reference/'verification.json').read_text())

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def tokens(p): return re.sub(r'\s+','',re.sub(r'/\*.*?\*/|//[^\n]*','',p.read_text(),flags=re.S))
for n,h in proof['files'].items(): assert sha(reference/n)==h,n
for p in baseline.iterdir():
    matches=[h for n,h in manifest['files'].items() if Path(n).name==p.name]
    assert len(matches)==1 and sha(p)==matches[0],p.name
for n in ('wheel_speed_controller.c','wheel_speed_controller.h','wheel_speed_control_config.h'):
    assert tokens(root/'Application/Chassis'/n)==tokens(reference/'prototype'/n),n
assert sha(root/'Application/Safety/fault_manager.c')==sha(baseline/'fault_manager.c')
logs=[]
def run(label,cmd,stdin=None,emit=False,save=None):
    r=subprocess.run(list(map(str,cmd)),cwd=root,input=stdin,capture_output=True,
                     text=True,encoding='utf-8',errors='replace')
    s=r.stdout+r.stderr
    logs.append(label+'\nCOMMAND '+repr(list(map(str,cmd)))+'\n'+(s if len(s)<14000 else
      f'stdout lines={len(r.stdout.splitlines())}, sha256={hashlib.sha256(r.stdout.encode()).hexdigest()}\n'+r.stderr))
    (out/'test_run.log').write_text('\n'.join(logs),encoding='utf-8')
    assert r.returncode==0,(label,s[-6000:])
    assert not re.search(r'\b(?:warning|error):',s,re.I),(label,s)
    if save: (out/save).write_text(r.stdout)
    if emit: print(r.stdout,end='',flush=True)
    return r.stdout
flags=['gcc','-std=c11','-Wall','-Wextra','-Werror','-ITools/tests/imu_spi_probe/stubs',
       '-IApplication/Chassis','-IApplication/Safety','-IApplication/Services','-IApplication/Diagnostics']
runtime=['Application/Chassis/chassis_runtime.c','Application/Chassis/chassis_kinematics.c',
         'Application/Chassis/chassis_motion_profile.c','Application/Diagnostics/motion_trace.c']
def build(name,stem,old=False,extra=(),manager=False):
    fs=flags[:5]+(['-I'+str(baseline)] if old else [])+flags[5:]
    deps=runtime[:3]+[baseline/'fault_manager.c' if old else root/'Application/Safety/fault_manager.c'] if manager else runtime
    run(name+' build',fs+(['-DBASELINE'] if old else [])+list(extra)+
        [root/'Tools/tests/motion'/('test_rr_exit'+stem+'.c')]+deps+['-lm','-o',out/(name+'.exe')])
def exe(n):return out/(n+'.exe')
for n,stem in (('policy',''),('safety','_safety'),('concurrency','_concurrency')):
    build(n,stem);run(n,[exe(n)],emit=True)

for label,old in (('baseline',True),('candidate',False)):
    build(label+'_identity','_identity',old)
    build(label+'_scope','_isolation',old)
    build(label+'_fault','_fault_audit',old,manager=True)
    build(label+'_replay','_replay',old)
build('observed_scope','_isolation',extra=['-DINJECT_COUNTS'])
a=run('baseline no-channel identity',[exe('baseline_identity')])
b=run('candidate no-channel identity',[exe('candidate_identity')])
assert a==b and len(b.splitlines())==30000
print('PASS: 30000 RR running updates without fast observations identical to frozen .20',flush=True)
for label,args,count in (('scope',[],80000),('first-start',['startup'],10000)):
    a=run('baseline '+label,[exe('baseline_scope')]+args)
    b=run('candidate '+label,[exe('candidate_scope')]+args)
    assert a==b and len(b.splitlines())==count*2
    print(f'PASS: {count} {label} updates/exported LR state identical to frozen .20',flush=True)
    if not args:
        assert run('observed unrelated scope',[exe('observed_scope')])==a
        print('PASS: 80000 unrelated-scope updates unchanged with injected RR observations',flush=True)

grid=[]
for phase in (0,1):
    for base in (0,0xFFFFFC00):
        for seed in list(range(-50,51))+[5.85]:
            pair={}
            for label in ('baseline','candidate'):
                s=run(f'{label} grid I={seed} base={base} exitphase={phase}',
                      [exe(label+'_fault'),seed,base,phase])
                pair[label]=int(re.search(r'RESULT trip_from_stall_ms=(\d+)',s)[1])
            assert pair['candidate']<=pair['baseline'],(phase,base,seed,pair)
            grid.append(dict(exit_phase=phase,base=base,seed_i=seed,**pair))
(out/'fault_grid.json').write_text(json.dumps(grid,indent=2)+'\n')
print('PASS: 408 paired actual fault-manager/runtime cases: no added stall delay, latch/four-stop/reject/explicit-clear/no-auto-resume, wrap and post-fast-exit',flush=True)

frames=[f for f in json.loads((root/'docs/stm32_evidence/2026-10-22-v01820-ground-left/capture/trace_parsed.json').read_text()) if f['state']==2]
assert len(frames)==127
inputs=''.join(f'{f["t"]} {f["dt"]} '+' '.join(str(f['wheels'][w][k])
  for w in ('LF','LR','RF','RR') for k in ('dc','cps','ff_x10','i_x10','pwm'))+'\n' for f in frames)
for label,shift in (('low',-.49),('mid',0),('high',.49)):
    for flavor in ('baseline','candidate'):
        s=run(flavor+' replay '+label,[exe(flavor+'_replay'),shift],inputs,save=flavor+'_'+label+'.csv')
        expected=(reference/(flavor+'_'+label+'.csv')).read_text()
        assert s==expected,(flavor,label)
print('PASS: production .21 all-four-wheel replay exactly matches approved prototype across three conditional seeds; prescribed feedback, NOT physical prediction',flush=True)
report=dict(result='PASS_SOFTWARE_NOT_HARDWARE',new_pass_groups=17,policy_groups=7,
    immediate_shutdown_groups=1,concurrency_groups=3,automatic_fault_pairs=len(grid),
    max_added_stall_delay_ms=max(r['candidate']-r['baseline'] for r in grid),
    candidate_stall_delay_ms=sorted(set(r['candidate'] for r in grid)),
    comparison='frozen .20, independent approved RR exit prototype, prescribed feedback',
    formal_pure_policy_matches_prototype=True,chassis_has_RR_transaction_guard=True,
    RTOS_timing_measured=False,serial_opened=False,motion_sent=False,
    frozen_baseline_sha256={p.name:sha(p) for p in baseline.iterdir()})
(out/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
