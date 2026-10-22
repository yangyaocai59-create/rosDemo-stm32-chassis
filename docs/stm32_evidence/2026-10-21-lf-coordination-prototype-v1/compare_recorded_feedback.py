"""Prescribe the SAME previously inferred speeds. Never predict real wheel speed."""
import csv
import hashlib
import io
import json
import pathlib
import subprocess

folder = pathlib.Path(__file__).resolve().parent
root = next(p for p in folder.parents if (p/'Tools/tests/run_motion_tests.ps1').is_file())
prior = folder.parent/'2026-10-21-v01815-lf-cycle-review'
old_report = json.loads((prior/'replay_analysis.json').read_text(encoding='utf-8'))
fields = ['t','measured','ff','p','i','pwm','load','recovery','still_ms','moving_ms',
          'overspeed_ms','recovery_ms','no_motion_ms','recovering','exhausted','failed']
out = root/'build/prototypes/lf_coordination_v1'
executables = {}
for policy,source in [('baseline',prior/'front_replay.c'),('prototype',folder/'replay_prototype.c')]:
    exe = out/(policy+'_replay.exe')
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',str(source),'-lm','-o',str(exe)],check=True)
    executables[policy]=exe
report = {'kind':'PRESCRIBED_OLD_FEEDBACK_NOT_A_PLANT_OR_NEW_TRACKING_TEST', 'wheels':{}}
for name,index,sign in [('LF',0,1),('RF',2,-1)]:
    seed=old_report['wheels'][name]['seed']
    recorded=list(csv.DictReader((prior/(name.lower()+'_inferred_replay.csv')).open(encoding='utf-8')))
    text=f'{index} {seed["t"]} {seed["load"]:.9f} {seed["i"]:.9f} {float(recorded[0]["measured"]):.9f}\n'
    text+=''.join(f'{int(float(r["t"]))} {float(r["measured"]):.9f}\n' for r in recorded[1:])
    results={}
    for policy,exe in executables.items():
        run=subprocess.run([str(exe)],input=text,text=True,capture_output=True)
        assert run.returncode==0,run.stderr
        rows=[dict(zip(fields,map(float,r))) for r in csv.reader(io.StringIO(run.stdout))]
        assert len(rows)==127
        (folder/f'{name.lower()}_{policy}_prescribed.csv').write_text(','.join(fields)+'\n'+run.stdout,encoding='utf-8')
        support=[sign*(r['ff']+r['i']) for r in rows]
        results[policy]={'rows':rows,'max_abs_PWM':max(abs(r['pwm']) for r in rows),
            'max_abs_FF_plus_I_step':max(abs(b-a) for a,b in zip(support,support[1:])),
            'max_support':max(support),'max_recovery':max(r['recovery'] for r in rows),
            'failure_samples':sum(bool(r['failed']) for r in rows),
            'support_at_selected_times':{int(r['t']):sign*(r['ff']+r['i']) for r in rows
                if int(r['t']) in (291883,292023,292043,292243,292343,292383,292443,292543,292803,292923)}}
    old,new=results['baseline']['rows'],results['prototype']['rows']
    if name=='RF':
        assert old==new, 'RF must be exactly unchanged with identical initial state/input'
    else:
        for row in new:
            assert row['load']<=60.001 and row['load']>=0
            assert row['recovery']+row['i']<=50.001
        assert results['prototype']['max_abs_FF_plus_I_step']<results['baseline']['max_abs_FF_plus_I_step']
    report['wheels'][name]={policy:{k:v for k,v in values.items() if k!='rows'} for policy,values in results.items()}
report['limits']=[
    'Initial private state inherited from the previous conditional fit, not measured on board.',
    'All recorded speeds/counts are prescribed. No prediction of new RMS error, stopping intervals or actual overshoot is possible.',
    'This removes/softens specified algorithmic branch discontinuities only, not proof of closed-loop stability.',
    'Slower withdrawal and lower combined rise may hurt unloading/startup; real validation remains necessary.'
]
report['sha256']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (folder/'prototype/wheel_speed_controller.c',prior/'lf_inferred_replay.csv',prior/'rf_inferred_replay.csv')}
(folder/'replay_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
