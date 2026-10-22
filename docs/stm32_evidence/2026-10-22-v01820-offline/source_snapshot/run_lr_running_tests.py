"""Host-only LR policy/safety regression, frozen baseline and scope checks.

Only native executables are built; never opens serial, moves or flashes.
"""
import hashlib
import json
from pathlib import Path
import re
import subprocess

root=Path(__file__).resolve().parents[3]
out=root/'build/tests/lr_running'
out.mkdir(parents=True,exist_ok=True)
frozen=root/'docs/stm32_evidence/2026-10-22-v01819-offline/baseline_0.18.18'
baseline=json.loads((root/'build/firmware/0.18.18/source_manifest.json').read_text())
prototype=root/'docs/stm32_evidence/2026-10-22-lr-running-prototype-v2/prototype'
proof=json.loads((prototype.parent/'verification.json').read_text())
prototype_hashes={n.replace('\\','/'):h for n,h in proof['files'].items()}

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
for p in frozen.iterdir():
    matches=[h for n,h in baseline['files'].items() if Path(n).name==p.name]
    assert len(matches)==1 and sha(p)==matches[0],p.name
for p in prototype.iterdir():
    assert sha(p)==prototype_hashes['prototype/'+p.name],p.name

def c_tokens(p):
    return re.sub(r'\s+','',re.sub(r'/\*.*?\*/|//[^\n]*','',p.read_text(),flags=re.S))

rr_reference=root/'docs/stm32_evidence/2026-10-22-rr-running-prototype-v1'
rr_proof=json.loads((rr_reference/'verification.json').read_text())
for n,h in rr_proof['files'].items():
    assert sha(rr_reference/n)==h,n
for stem in ('chassis_controller.c','wheel_speed_controller.c',
             'wheel_speed_controller.h','wheel_speed_control_config.h','fault_manager.c'):
    folder='Safety' if stem=='fault_manager.c' else 'Chassis'
    assert c_tokens(rr_reference/'baseline'/stem)==c_tokens(prototype/stem),stem
    assert c_tokens(root/'Application'/folder/stem)==c_tokens(rr_reference/'prototype'/stem),stem

logs=[]
def run(label,command,emit=False):
    r=subprocess.run(list(map(str,command)),cwd=root,capture_output=True,text=True,
                     encoding='utf-8',errors='replace')
    output=r.stdout+r.stderr
    assert r.returncode==0,(label,output[-6000:])
    assert not re.search(r'\b(?:warning|error):',output,re.I),(label,output)
    logs.append(label+'\n'+(output if len(output)<16000 else
       f'{len(output.splitlines())} lines, SHA256={hashlib.sha256(output.encode()).hexdigest()}\n'))
    (out/'test_run.log').write_text('\n'.join(logs))
    if emit: print(output,end='',flush=True)
    return output

flags=['gcc','-std=c11','-Wall','-Wextra','-Werror','-ITools/tests/imu_spi_probe/stubs',
       '-IApplication/Chassis','-IApplication/Safety','-IApplication/Services','-IApplication/Diagnostics']
runtime=['Application/Chassis/chassis_runtime.c','Application/Chassis/chassis_kinematics.c',
         'Application/Chassis/chassis_motion_profile.c','Application/Diagnostics/motion_trace.c']
for stem in ('lr_running','lr_running_safety','lr_running_fault_link'):
    manager=[root/'Application/Safety/fault_manager.c'] if stem.endswith('fault_link') else []
    dependencies=runtime[:3] if manager else runtime
    run(stem+' build',flags+['Tools/tests/motion/test_'+stem+'.c']+dependencies+manager+
        ['-lm','-o',out/(stem+'.exe')])
    run(stem,[out/(stem+'.exe')],True)

baseline_flags=flags[:5]+['-I'+str(frozen)]+flags[5:]
run('frozen complete runtime build',baseline_flags+[frozen/'test_runtime_motion.c',
    frozen/'chassis_controller.c',frozen/'wheel_speed_controller.c']+runtime+
    ['-lm','-o',out/'baseline_runtime.exe'])
full_baseline=run('frozen complete runtime',[out/'baseline_runtime.exe'])
assert len(re.findall(r'^PASS:',full_baseline,re.M))==15
print('PASS: frozen .18 complete original runtime 15 groups retained and passing; changed LR expectations replaced only in current tests',flush=True)

for label,extra in [('baseline',['-DBASELINE']),('candidate',[])]:
    run(label+' scope build',flags+extra+['Tools/tests/motion/test_lr_running_isolation.c']+
        runtime+['-lm','-o',out/(label+'_scope.exe')])
for label,args,count in [('unchanged scope excluding LR reverse/RR forward pivot',[],80000),('startup',['startup'],10000)]:
    old=run(label+' baseline',[out/'baseline_scope.exe']+args)
    new=run(label+' candidate',[out/'candidate_scope.exe']+args)
    assert old==new and len(new.splitlines())==count
    print(f'PASS: {count} {label} updates bit-identical to frozen .18',flush=True)

identity={}
for label,extra in [('prototype',['-DREFERENCE']),('production',[])]:
    run(label+' identity build',flags+extra+['Tools/tests/motion/test_lr_running_identity.c']+
        runtime+['-lm','-o',out/(label+'_identity.exe')])
    identity[label]=run(label+' identity',[out/(label+'_identity.exe')])
assert identity['prototype']==identity['production']
assert len(identity['production'].splitlines())==30000
print('PASS: 30000 LR running control outputs/private states bit-identical to validated v2 prototype',flush=True)

for label,extra in [('baseline',['-DBASELINE']),('candidate',[])]:
    manager=frozen/'fault_manager.c' if label=='baseline' else root/'Application/Safety/fault_manager.c'
    selected_flags=baseline_flags if label=='baseline' else flags
    run(label+' fault audit build',selected_flags+extra+['Tools/tests/motion/test_lr_running_fault_audit.c']+
      runtime[:3]+[manager,'-lm','-o',out/(label+'_fault.exe')])
grid=[]
for base in (0,0xFFFFFC00):
    for seed in list(range(-50,51))+[6.7]:
        pair={}
        for label in ('baseline','candidate'):
            result=run(label+f' fault I={seed} base={base}',
                [out/(label+'_fault.exe'),seed,base])
            pair[label]=int(re.search(r'RESULT trip_from_stall_ms=(\d+)',result)[1])
        assert pair['candidate']<=pair['baseline'],(seed,base,pair)
        grid.append(dict(base=base,seed_i=seed,**pair))
(out/'fault_grid.json').write_text(json.dumps(grid,indent=2)+'\n')
assert len(grid)==204
print('PASS: 204 real runtime/fault-manager audits: no added stall delay, latch/all-output shutdown/reject/explicit-clear/no-auto-resume, including tick wrap',flush=True)
report={'result':'PASS_OFFLINE_NOT_HARDWARE','new_pass_groups':15,
        'unchanged_scope_updates':80000,'first_start_updates':10000,
        'production_vs_prototype_updates':30000,'fault_audit_cases':len(grid),
        'max_added_fault_delay_ms':max(v['candidate']-v['baseline'] for v in grid),
        'candidate_trip_ms':sorted(set(v['candidate'] for v in grid)),
        'baseline_runtime_groups':15,'serial_opened':False,'motion_sent':False,
        'fault_cleared_on_hardware':False,
        'frozen_baseline_sha256':{p.name:sha(p) for p in frozen.iterdir()}}
(out/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
