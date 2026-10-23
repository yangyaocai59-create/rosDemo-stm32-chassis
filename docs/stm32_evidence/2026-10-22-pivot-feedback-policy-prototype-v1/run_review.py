"""Compile/replay the isolated safety-policy experiment, no hardware access."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=next(p for p in HERE.parents if (p/'Application').is_dir())
OUT=HERE/'results'
OUT.mkdir(exist_ok=True)
NAMES=('LF','LR','RF','RR')
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=read(ROOT/'build/firmware/0.18.21/source_manifest.json')
proof=read(ROOT/'build/firmware/0.18.21/verification.json')
assert len(manifest['files'])==167
assert all(sha(ROOT/n)==h for n,h in manifest['files'].items())
assert all(sha(ROOT/f'build/firmware/0.18.21/rosDemo-0.18.21.{ext}')==h for ext,h in proof['sha256'].items())

def run(command,**kw):
    result=subprocess.run(list(map(str,command)),cwd=ROOT,capture_output=True,
                          text=True,encoding='utf-8',errors='replace',**kw)
    if result.returncode:
        raise RuntimeError(str(command)+'\n'+result.stdout+result.stderr)
    return result.stdout+result.stderr

common=['gcc','-std=c11','-Wall','-Wextra','-Werror']
prototype=HERE/'prototype'
test=ROOT/'build/tests/pivot_feedback_policy_test.exe'
replay=ROOT/'build/tests/pivot_feedback_policy_replay.exe'
build_test=run(common+[prototype/'test_policy.c',prototype/'pivot_feedback_policy.c','-lm','-o',test])
build_replay=run(common+[prototype/'replay_policy.c',prototype/'pivot_feedback_policy.c','-lm','-o',replay])
(OUT/'native_build.log').write_text(build_test+build_replay+'PASS: strict native builds, no warnings\n',encoding='utf-8')
test_log=run([test])
assert test_log.count('PASS:')==10
(OUT/'native_tests.log').write_text(test_log,encoding='utf-8')
optimized=ROOT/'build/tests/pivot_feedback_policy_O2_test.exe'
analyzer_log=run(common+['-O2','-fanalyzer',prototype/'test_policy.c',prototype/'pivot_feedback_policy.c','-lm','-o',optimized])
optimized_log=run([optimized])
assert optimized_log.count('PASS:')==10
(OUT/'native_O2_analyzer.log').write_text(analyzer_log+optimized_log,encoding='utf-8')
arm=Path('C:/Users/yang/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-gcc.exe')
arm_log=run([arm,'-std=c11','-Wall','-Wextra','-Werror','-mcpu=cortex-m4',
             '-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-O2','-fstack-usage',
             '-c',prototype/'pivot_feedback_policy.c','-o',ROOT/'build/tests/pivot_feedback_policy_arm.o'])
(OUT/'isolated_arm_compile.log').write_text(arm_log+'PASS: isolated Cortex-M4 object compiled, NOT a linked/flashable firmware\n',encoding='utf-8')

def parse_trace(folder):
    trace=[]; current=None
    for line in (folder/'ch340.log').read_text(encoding='utf-8').splitlines():
        m=re.search(r'TRACE t=(\d+) dt=(\d+) req=(-?\d+)/(-?\d+) applied=(-?\d+)/(-?\d+) state=(\d+) stop=(\d+) fault=(\d+) stby=(\d+)',line)
        if m:
            current=dict(zip(('t','dt','req_v','req_w','v','w','state','stop','fault','stby'),map(int,m.groups())))
            current['wheels']={}; trace.append(current)
        m=re.search(r'TRACE (LF|LR|RF|RR) dc=(-?\d+) raw=(-?\d+) target=(-?\d+) cps=(-?\d+) ff_x10=(-?\d+) p_x10=(-?\d+) i_x10=(-?\d+) pwm=(-?\d+)',line)
        if m and current is not None:
            current['wheels'][m[1]]=dict(zip(('dc','raw','target','cps','ff','p','i','pwm'),map(int,m.groups()[1:])))
    return [t for t in trace if t['state']==2 and len(t['wheels'])==4]

def replay_rows(label,rows,warm=False,synthetic_after=None):
    text=''.join(' '.join(map(str,row))+'\n' for row in rows)
    (OUT/(label+'.input.txt')).write_text(text,encoding='utf-8')
    csv_text=run([replay]+(['--warm-start'] if warm else []),input=text)
    (OUT/(label+'.output.csv')).write_text('source_ms,result,last_progress_ms,severe_since_ms\n'+csv_text,encoding='utf-8')
    outputs=[list(map(int,line.split(','))) for line in csv_text.splitlines()]
    assert len(outputs)==len(rows)
    faults=[r for r in outputs if r[1] in (2,3,4)]
    first=faults[0] if faults else None
    assert all(r[1]!=4 for r in outputs),(label,'Evidence assembly/replay failure',outputs)
    return dict(samples=len(rows),warning_samples=sum(r[1]==1 for r in outputs),
        first_fault=first,first_fault_is_counterfactual=bool(first and synthetic_after and first[0]>synthetic_after),
        warm_start_assumption=warm,synthetic_after_source_ms=synthetic_after)

cases={}
sources={
 'successful_left':HERE.parent/'2026-10-22-v01821-long-left-repeat/capture',
 'failed_left':HERE.parent/'2026-10-22-v01821-long-left-once/capture',
 'failed_right':HERE/'input_right'}
fault_stamps={'failed_left':2555822,'failed_right':3944482}
for case,folder in sources.items():
    data=read(folder/'decoded.json')
    phase='RIGHT_motion' if case=='failed_right' else 'LEFT_motion'
    # Exclude any source sample already at/after the real firmware's stop.
    samples=[r['values'] for r in data['wheel'] if r['phase']==phase
             and r['values'][0]<fault_stamps.get(case,0xffffffff)]
    sign=(-1, -1, 1, 1) if phase=='LEFT_motion' else (1,1,-1,-1)
    for wheel,name in enumerate(NAMES):
        rows=[]; prev=None
        for v in samples:
            if prev is not None and v[0]==prev[0]: continue
            dt=(v[0]-prev[0])&0xffffffff if prev else 20
            delta=v[wheel+1]-prev[wheel+1] if prev else 0
            # Full USB capture lacks per-wheel PWM. Policy decisions do not
            # use PWM magnitude; zero here means UNMEASURED, not a real zero.
            rows.append([v[0],v[0],dt,v[wheel+1],delta,sign[wheel]*240,v[wheel+5],0,1,1])
            prev=v
        cases[case+'_USB_'+name]=replay_rows(case+'_USB_'+name,rows)
    if case=='failed_left': continue # No detailed ring captured in that run.
    trace=parse_trace(folder)
    assert len(trace)==127
    for name in NAMES:
        rows=[]; count=0
        for t in trace:
            w=t['wheels'][name]; count+=w['dc']
            rows.append([t['t'],t['t'],t['dt'],count,w['dc'],w['target'],w['cps'],w['pwm'],1,1])
        label=case+'_trace_'+name
        cases[label]=replay_rows(label,rows,warm=True)
        if case=='failed_right' and name=='RF':
            last=rows[-1]; appended=[r[:] for r in rows]
            measured=float(last[6])
            # Synthetic no-count continuation AFTER real stop. It tests a
            # finite deadline only, NOT whether the real wheel would recover.
            for extra in range(20,1601,20):
                measured*=50/70
                now=last[0]+extra
                appended.append([now,now,20,last[3],0,last[5],measured,last[7],1,1])
            label+='__SYNTHETIC_FROZEN_EXTENSION'
            cases[label]=replay_rows(label,appended,warm=True,synthetic_after=last[0])

for name in NAMES:
    assert cases['successful_left_USB_'+name]['first_fault'] is None
    assert cases['successful_left_trace_'+name]['first_fault'] is None
extension=cases['failed_right_trace_RF__SYNTHETIC_FROZEN_EXTENSION']
assert extension['first_fault_is_counterfactual'] and extension['first_fault'][1] in (2,3)
assert all(sha(ROOT/n)==h for n,h in manifest['files'].items())
result=dict(result='PASS_ISOLATED_RULE_TESTS_NOT_FIRMWARE_OR_PHYSICAL_RECOVERY_VALIDATION',
    native_test_groups=10,replay_cases=len(cases),cases=cases,
    optimized_native_test_groups=10,gcc_static_analyzer_pass=True,
    isolated_cortex_m4_object_compile_pass=True,
    protected_source_hashes_unchanged=167,firmware_artifacts_unchanged=proof['sha256'],
    hardware_access=False,production_changes=False,new_firmware_generated=False,
    source_observations={'right_fault_mask':'0x04','right_fault_time_ms':3944482,
      'right_raw_zero_tail_intervals_ms':520,'operator_report':'Four wheels looked continuous; no clear abnormality',
      'external_180_degree_acceptance':'FAILED_TO_COMPLETE; left run canceled; no independent external angle result'},
    limits=['USB-only replay assumes fixed +/-240cps target after startup and has no PWM measurement.',
            'Tail replay assumes healthy prehistory; it is not complete internal-state reconstruction.',
            'No mechanical motor/ground/encoder-noise model or verified A/B waveforms.',
            'Sparse pulses can postpone the 750ms raw no-progress deadline; 1500ms severe cap bounds that case.',
            'Lower targets, controller safety snapshots, fault injection and task/transport integration are not implemented.',
            'Windows GCC undefined-behavior sanitizer linker unavailable: cannot find -lubsan; strict ordinary native tests pass.',
            'Not proof of electrical/thermal safety or permission to ignore missing counts.'])
(HERE/'verification.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(test_log,end='')
for label in ('failed_left_USB_LR','failed_right_USB_RF','failed_right_trace_RF','failed_right_trace_RF__SYNTHETIC_FROZEN_EXTENSION'):
    print(label,json.dumps(cases[label]))
print('PASS: 167 production source hashes and four .21 firmware artifacts unchanged; NO firmware generated or hardware opened')
