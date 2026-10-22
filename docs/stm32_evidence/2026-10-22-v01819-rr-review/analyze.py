"""Native conditional replay and provenance checks; no hardware access."""
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Application').is_dir())
capture=here.parent/'2026-10-22-v01819-ground-left/capture'
manifest=json.loads((root/'build/firmware/0.18.19/source_manifest.json').read_text())
artifact=json.loads((root/'build/firmware/0.18.19/verification.json').read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def unchanged():
    assert all(sha(root/p)==h for p,h in manifest['files'].items())
    assert all(sha(root/f'build/firmware/0.18.19/rosDemo-0.18.19.{ext}')==h for ext,h in artifact['sha256'].items())
unchanged()
frozen={}
for p in (here/'source_snapshot').iterdir():
    original=('Tools/tests/motion/' if p.name=='test_runtime_motion.c' else
              'Application/Diagnostics/' if p.name=='motion_trace.c' else 'Application/Chassis/')+p.name
    assert sha(p)==manifest['files'][original],p.name
    frozen[original]=sha(p)
frames=[f for f in json.loads((capture/'trace_parsed.json').read_text()) if f['state']==2]
assert len(frames)==127 and all(len(f['wheels'])==4 and f['dt']==20 for f in frames)
assert all(b['t']-a['t']==20 for a,b in zip(frames,frames[1:]))
assert all(f['req_w']==f['applied_w']==300 and f['stby']==1 and not f['fault'] for f in frames)
inputs=''.join(f'{f["t"]} {f["dt"]} '+ ' '.join(str(f['wheels'][w][field])
    for w in ('RR','RF') for field in ('dc','cps','ff_x10','i_x10','pwm'))+'\n' for f in frames)
out=root/'build/analysis/v01819_rr'
out.mkdir(parents=True,exist_ok=True)
command=['gcc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(here/'source_snapshot'),
 '-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','-IApplication/Safety',
 '-IApplication/Services','-IApplication/Diagnostics',here/'replay.c',
 'Application/Chassis/chassis_runtime.c','Application/Chassis/chassis_kinematics.c',
 'Application/Chassis/chassis_motion_profile.c','Application/Diagnostics/motion_trace.c',
 '-lm','-o',out/'replay.exe']
compiled=subprocess.run(list(map(str,command)),cwd=root,capture_output=True,text=True)
(here/'build.log').write_text(compiled.stdout+compiled.stderr)
assert compiled.returncode==0,compiled.stdout+compiled.stderr
assert not re.search(r'\b(?:warning|error):',compiled.stdout+compiled.stderr,re.I)
baseline=json.loads((root/'build/firmware/0.18.18/source_manifest.json').read_text())
baseline_snapshot=here.parent/'2026-10-22-v01819-offline/baseline_0.18.18'
for p in baseline_snapshot.iterdir():
    matches=[h for name,h in baseline['files'].items() if Path(name).name==p.name]
    assert len(matches)==1 and sha(p)==matches[0],p.name
baseline_command=command.copy()
baseline_command[5]='-I'+str(baseline_snapshot)
baseline_command.insert(5,'-DBASELINE')
baseline_command[-1]=out/'baseline.exe'
compiled_old=subprocess.run(list(map(str,baseline_command)),cwd=root,capture_output=True,text=True)
(here/'baseline_build.log').write_text(compiled_old.stdout+compiled_old.stderr)
assert compiled_old.returncode==0,compiled_old.stdout+compiled_old.stderr
assert not re.search(r'\b(?:warning|error):',compiled_old.stdout+compiled_old.stderr,re.I)
replays={}
for initial_restart in (0,1):
    for label,shift in [('low',-.49),('mid',0),('high',.49)]:
        name=label+'_prior_restart_'+str(initial_restart)
        result=subprocess.run([str(out/'replay.exe'),str(shift),str(initial_restart)],
             input=inputs,text=True,capture_output=True,cwd=root)
        assert result.returncode==0,result.stderr
        old_result=subprocess.run([str(out/'baseline.exe'),str(shift),str(initial_restart)],
             input=inputs,text=True,capture_output=True,cwd=root)
        assert old_result.returncode==0,old_result.stderr
        assert old_result.stdout==result.stdout,(name,'Frozen .18/.19 RR/RF conditional command mismatch')
        (here/(name+'.csv')).write_text(result.stdout)
        rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(result.stdout))]
        assert len(rows)==127
        metrics={}
        for wheel,prefix in [('RR','rr'),('RF','rf')]:
            errors={k:[] for k in ('cps','ff_x10','p_x10','i_x10','pwm')}
            for row,f in zip(rows[1:],frames[1:]):
                for field in errors:
                    suffix={'cps':'measured','ff_x10':'ff','p_x10':'p','i_x10':'i','pwm':'pwm'}[field]
                    simulated=row[prefix+'_'+suffix]
                    printed=int(simulated*(10 if field.endswith('x10') else 1))
                    errors[field].append(abs(printed-f['wheels'][wheel][field]))
            metrics[wheel]={'max_printed_difference':{k:max(v) for k,v in errors.items()},
                            'exact_pwm_rows':sum(e==0 for e in errors['pwm']),
                            'post_seed_rows':126}
        entry=next(r for r in rows if r['rr_restart_active'])
        release=next(b for a,b in zip(rows,rows[1:]) if a['rr_restart_active'] and not b['rr_restart_active'])
        rr_peak=max(rows,key=lambda r:r['rr_measured'])
        replays[name]={'seed_shift':shift,'initial_finished_restart_hypothesis':bool(initial_restart),
          'frozen_0_18_18_RR_RF_csv_bit_identical':True,
          'metrics':metrics,'entry_ms':int(entry['t']),'release_ms':int(release['t']),
          'held_ms':int(release['t']-entry['t']),
          'RR_target_actual_cps':entry['rr_target'],
          'entry_bias_permille':entry['rr_bias'],
          'RR_max_positive_I_plus_legacy_bias_permille':max(max(0,r['rr_i'])+r['rr_bias'] for r in rows),
          'RR_max_abs_integral_permille':max(abs(r['rr_i']) for r in rows),
          'RR_peak':rr_peak,'RF_recovery_max_permille':max(r['rf_recovery'] for r in rows)}
        print(name,json.dumps({k:v for k,v in replays[name].items() if k!='RR_peak'}),flush=True)

mid=list(csv.DictReader((here/'mid_prior_restart_0.csv').open()))
mid=[{k:float(v) for k,v in row.items()} for row in mid]
groups=[]
for i,f in enumerate(frames):
    if f['wheels']['RR']['dc']==0:
        if not groups or groups[-1][-1]!=i-1: groups.append([])
        groups[-1].append(i)
assert [len(g) for g in groups]==[5,2]
window=[f for f in frames if 1078482<=f['t']<=1078982]
with (here/'event_all_wheels.csv').open('w',newline='') as output:
    fields=['t']+[w+'_'+k for w in ('LF','LR','RF','RR') for k in ('dc','cps','ff_x10','p_x10','i_x10','pwm')]
    writer=csv.DictWriter(output,fieldnames=fields); writer.writeheader()
    writer.writerows({'t':f['t'],**{w+'_'+k:f['wheels'][w][k]
      for w in ('LF','LR','RF','RR') for k in ('dc','cps','ff_x10','p_x10','i_x10','pwm')}} for f in window)
matched=all(max(m['max_printed_difference'].values())<=1 for r in replays.values() for m in r['metrics'].values())
branches_stable=all(r['entry_ms']==1078642 and r['release_ms']==1078722 for r in replays.values())
report={'kind':'OFFLINE_DIAGNOSIS_CONDITIONAL_C_REPLAY_NOT_FIRMWARE_OR_PLANT',
 'print_quantization_match_within_one_unit':matched,'six_seed_branch_times_stable':branches_stable,
 'conditional_replays':replays,
 'initial_seed_hypotheses':{'RR':'printed midpoint+small shift; prior motion true; initial legacy total bias=0; both finished-restart flags tested',
   'RF':'printed midpoint+small shift; recovery=0; FF excess assigned to adaptive load; moving40ms'},
 'RR_zero_episodes':[{'first_zero_row_ms':frames[g[0]]['t'],'last_zero_row_ms':frames[g[-1]]['t'],
   'covered_sample_intervals_ms':sum(frames[i]['dt'] for i in g)} for g in groups],
 'conditional_states':[r for r in mid if int(r['t']) in (1078542,1078562,1078622,1078642,1078662,1078702,1078722,1078802,1078902,1078942)],
 'snapshot_sha256':frozen,'input_capture_sha256':{p.name:sha(p) for p in capture.iterdir() if p.is_file()},
 'frozen_0_18_18_snapshot_sha256':{str(p.relative_to(root)):sha(p) for p in baseline_snapshot.iterdir()},
 'production_source_test_items_unchanged':149,'four_firmware_artifacts_unchanged':True,
 'unchanged_firmware_sha256':artifact['sha256'],
 'serial_opened':False,'motion_sent':False,'fault_cleared':False,'production_modified':False,
 'firmware_linked':False,'flashed':False,
 'limits':['Initial hidden states are inferred, not unique or directly measured.',
  'Six seed/history combinations are sensitivity checks, not exhaustive private-state reconstruction.',
  'Same prescribed counts do not predict wheel response after changing the controller.',
  'RR no-count gaps precede support; this replay cannot identify the first physical/electrical cause.',
  'Other wheels reacting together does not identify causality or exclude power/mechanical effects.',
  'The internal ring covers only the final 2.54 seconds, not startup or long-term stability.']}
unchanged()
(here/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print('QUANTIZATION_MATCH',matched,'BRANCH_TIMES_STABLE',branches_stable,flush=True)
assert matched and branches_stable,'Replay hypotheses do not yet match within printed resolution; inspect analysis.json'
