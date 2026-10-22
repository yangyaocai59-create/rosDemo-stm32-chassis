"""Compare command accounting on prescribed measured counts, NOT new wheel motion."""
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Application').is_dir())
baseline=here.parent/'2026-10-21-v01817-lf-start-review'
out=root/'build/prototypes/lf_first_start_v1'
out.mkdir(parents=True,exist_ok=True)
for name,source in [('baseline',baseline/'start_replay.c'),('prototype',here/'start_replay.c')]:
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',str(source),'-lm',
                    '-o',str(out/(name+'_replay.exe'))],check=True)
report={'kind':'COMMAND_COMPARISON_ON_FIXED_RECORDED_FEEDBACK_NOT_A_PHYSICAL_PREDICTION','trials':{}}
paths=[('first',here.parent/'2026-10-21-v01817-ground-left-fault/fault_trace/trace_parsed.json'),
       ('repeat',here.parent/'2026-10-21-v01817-ground-left-repeat/capture/trace_parsed.json')]
for label,path in paths:
    trace=[r for r in json.loads(path.read_text()) if r['state']==2]
    inputs=''.join(f'{r["t"]} {r["wheels"]["LF"]["dc"]}\n' for r in trace)
    outputs={}
    for name in ('baseline','prototype'):
        r=subprocess.run([str(out/(name+'_replay.exe'))],input=inputs,text=True,capture_output=True,check=True)
        rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(r.stdout))]
        assert len(rows)==len(trace),(label,name,len(rows),len(trace))
        (here/(label+'_'+name+'_commands.csv')).write_text(r.stdout)
        assert all(int(v['target'])==f['wheels']['LF']['target'] and
                   int(v['measured'])==f['wheels']['LF']['cps'] for v,f in zip(rows,trace))
        if name=='baseline':
            assert all(int(v['pwm'])==f['wheels']['LF']['pwm'] for v,f in zip(rows,trace))
        assert max(abs(v['pwm']) for v in rows)<=300
        assert max(v['recovery']-v['i'] for v in rows)<=50.001
        failed=[v for v in rows if v['failed']]
        assert failed and int(failed[0]['t'])==trace[-1]['t']
        entry=next(v for v in rows if v['recovering'])
        exit_row=next(b for a,b in zip(rows,rows[1:]) if a['recovering'] and not b['recovering'])
        full=[v for v in rows if v['load']>=64.999 and v['recovery']>=49.999 and v['recovering']]
        outputs[name]={'samples':len(rows),'peak_pwm_abs':max(abs(v['pwm']) for v in rows),
          'peak_recovery_permille':max(v['recovery'] for v in rows),
          'recovery_entry_ms':int(entry['t']),'recovery_exit_ms':int(exit_row['t']),
          'full_support_first_ms':int(full[0]['t']) if full else None,
          'full_support_frame_accounting_ms':20*len(full),
          'failure_ms':int(failed[0]['t']),
          'first_elapsed_at_exit':exit_row.get('first_elapsed'),
          'full_hold_at_exit':exit_row.get('full_hold')}
    assert outputs['prototype']['peak_pwm_abs']>outputs['baseline']['peak_pwm_abs']
    assert outputs['prototype']['peak_recovery_permille']==50
    assert outputs['prototype']['full_support_frame_accounting_ms']>=100
    assert outputs['prototype']['first_elapsed_at_exit']<=600
    report['trials'][label]={'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),**outputs}
report['limits']=['The same feedback is forced into both controllers. Real feedback could differ.',
                  'Higher/longer commanded effort is intentional but not evidence the real wheel starts.',
                  'Failure time remains unchanged for these fixed inputs; no safety gate relaxed.',
                  'No serial, motion, production file modification, firmware link or flash.']
(here/'recorded_comparison.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
