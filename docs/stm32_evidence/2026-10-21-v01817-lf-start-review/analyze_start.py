"""Frozen-C LF startup replay from zero, no fitted private state or hardware access."""
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Application').is_dir())
manifest=json.loads((root/'build/firmware/0.18.17/source_manifest.json').read_text())
sources=list((here/'source_snapshot').glob('*'))
assert len(sources)==9
for p in sources:
    assert hashlib.sha256(p.read_bytes()).hexdigest()==manifest['files']['Application/Chassis/'+p.name]
exe=root/'build/tests/v01817_lf_start_replay.exe'
subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',str(here/'start_replay.c'),
                '-lm','-o',str(exe)],check=True)
report={'scope':'zero-state frozen-C replay driven by recorded increments, not a physical plant',
        'trials':{},'sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in [here/'start_replay.c',*sources]}}
for label,path in [('first',here.parent/'2026-10-21-v01817-ground-left-fault/fault_trace/trace_parsed.json'),
                   ('repeat',here.parent/'2026-10-21-v01817-ground-left-repeat/capture/trace_parsed.json')]:
    active=[r for r in json.loads(path.read_text()) if r['state']==2]
    assert all(r['dt']==20 for r in active)
    inputs=''.join(f'{r["t"]} {r["wheels"]["LF"]["dc"]}\n' for r in active)
    run=subprocess.run([str(exe)],input=inputs,text=True,capture_output=True,check=True)
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(run.stdout))]
    assert len(rows)==len(active)
    errors={k:max(abs(r[k]-obs['wheels']['LF'][src]/10) for r,obs in zip(rows,active))
            for k,src in [('ff','ff_x10'),('p','p_x10'),('i','i_x10')]}
    mismatches={k:sum(int(r[k])!=obs[src] for r,obs in zip(rows,active))
                for k,src in [('applied_mrad','applied_w')]}
    mismatches['target']=sum(int(r['target'])!=obs['wheels']['LF']['target'] for r,obs in zip(rows,active))
    mismatches['measured']=sum(int(r['measured'])!=obs['wheels']['LF']['cps'] for r,obs in zip(rows,active))
    mismatches['pwm']=sum(int(r['pwm'])!=obs['wheels']['LF']['pwm'] for r,obs in zip(rows,active))
    assert not any(mismatches.values()),(label,mismatches)
    assert max(errors.values())<.101,(label,errors)
    start=next(r for r in rows if r['recovering'])
    full=next(r for r in rows if r['applied_mrad']>=299.99)
    exit_row=next(r for a,r in zip(rows,rows[1:]) if a['recovering'] and not r['recovering'])
    peak=max(rows,key=lambda r:abs(r['pwm']))
    failed=[r for r in rows if r['failed']]
    (here/(label+'_replay.csv')).write_text(run.stdout)
    report['trials'][label]={'samples':len(rows),'all_printed_pwm_target_speed_match':True,
       'max_error_permille':errors,'mismatches':mismatches,'recovery_entry':start,
       'first_full_target':full,'recovery_exit':exit_row,'peak_pwm':peak,
       'max_recovery_permille':max(r['recovery'] for r in rows),
       'peak_allowed_headroom_permille':max(r['headroom'] for r in rows),
       'failed_rows':failed,'final_active':rows[-1],
       'entry_to_full_target_ms':full['t']-start['t'],
       'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
report['limits']=['Counts are prescribed. No model of torque, friction, driver current, supply dips or new-control feedback.',
                 'Zero controller/filter initial state is supported by preceding reset/idle/zero speed; no fitted hidden state.',
                 'C profile/kinematics/controller calls reconstruct the trace-compatible schedule, not every RTOS instruction.',
                 'A matching private-state path explains commands, not the unique physical cause or benefit of a new hold time.',
                 'No production firmware, safety gates, serial sessions or hardware state changed.']
(here/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report['trials'],indent=2))
