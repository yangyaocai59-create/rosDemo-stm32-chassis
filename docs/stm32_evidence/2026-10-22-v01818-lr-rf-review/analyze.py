"""Offline event analysis and conditional C replay; no hardware access."""
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Application').is_dir())
capture=here.parent/'2026-10-22-v01818-ground-left/capture'
manifest=json.loads((root/'build/firmware/0.18.18/source_manifest.json').read_text())
verification=json.loads((root/'build/firmware/0.18.18/verification.json').read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def check_unchanged():
    assert all(sha(root/p)==h for p,h in manifest['files'].items())
    assert all(sha(root/f'build/firmware/0.18.18/rosDemo-0.18.18.{s}')==h
               for s,h in verification['sha256'].items())
check_unchanged()
snapshot={}
for p in (here/'source_snapshot').iterdir():
    name=('Application/Diagnostics/' if p.name=='motion_trace.c' else 'Application/Chassis/')+p.name
    assert sha(p)==manifest['files'][name],name
    snapshot[name]=sha(p)
trace=json.loads((capture/'trace_parsed.json').read_text())
active=[f for f in trace if f['state']==2 and len(f['wheels'])==4]
assert len(active)==127 and all(b['t']-a['t']==20 for a,b in zip(active,active[1:]))
assert all(f['req_w']==300 and f['applied_w']==300 and f['stby']==1 and not f['fault']
           for f in active)
decoded=json.loads((capture/'decoded.json').read_text())
summary=json.loads((capture/'summary.json').read_text())
assert not summary['errors'] and not summary['host_crc_errors']
assert summary['final_status'][5]==1 and not any(summary['final_status'][10:16])
assert not summary['final_status'][6] and not summary['final_status'][8]
assert summary['final_status'][9]==0x150
assert not any(c['type']==3 for c in summary['tx_commands'])

out=root/'build/analysis/v01818_lr_rf'
out.mkdir(parents=True,exist_ok=True)
cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(here/'source_snapshot'),
     '-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','-IApplication/Safety',
     '-IApplication/Services','-IApplication/Diagnostics',str(here/'replay.c'),
     'Application/Chassis/chassis_runtime.c','Application/Chassis/chassis_kinematics.c',
     'Application/Chassis/chassis_motion_profile.c','Application/Diagnostics/motion_trace.c',
     '-lm','-o',str(out/'replay.exe')]
r=subprocess.run(cmd,cwd=root,capture_output=True,text=True)
(here/'build.log').write_text(r.stdout+r.stderr)
assert r.returncode==0,r.stdout+r.stderr
assert not re.search(r'\b(?:warning|error):',r.stdout+r.stderr,re.I)
inputs=''.join(f'{f["t"]} {f["wheels"]["LR"]["dc"]} {f["wheels"]["RF"]["dc"]} '
    f'{f["wheels"]["LR"]["cps"]} {f["wheels"]["LR"]["i_x10"]} {f["wheels"]["LR"]["pwm"]} '
    f'{f["wheels"]["RF"]["cps"]} {f["wheels"]["RF"]["ff_x10"]} '
    f'{f["wheels"]["RF"]["i_x10"]} {f["wheels"]["RF"]["pwm"]}\n' for f in active)
replays={}
for label,shift in [('low',-.49),('mid',0),('high',.49)]:
    result=subprocess.run([str(out/'replay.exe'),str(shift)],input=inputs,text=True,
                          capture_output=True,cwd=root)
    assert result.returncode==0,result.stderr
    (here/f'replay_{label}.csv').write_text(result.stdout)
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(result.stdout))]
    assert len(rows)==127
    metrics={}
    for wheel,prefix in [('LR','lr'),('RF','rf')]:
        errors={field:[] for field in ('pwm','ff_x10','p_x10','i_x10','cps')}
        for row,frame in zip(rows[1:],active[1:]):
            v=frame['wheels'][wheel]
            for field in errors:
                simulated=row[prefix+'_'+{'pwm':'pwm','ff_x10':'ff','p_x10':'p','i_x10':'i','cps':'measured'}[field]]
                printed=int(simulated*(10 if field.endswith('x10') else 1))
                errors[field].append(abs(printed-v[field]))
        metrics[wheel]={f'max_printed_{field}_difference':max(values) for field,values in errors.items()}
        metrics[wheel]['exact_pwm_rows']=sum(e==0 for e in errors['pwm'])
        assert max(errors['pwm'])<=1,(label,wheel,metrics[wheel])
        assert all(max(errors[k])<=1 for k in errors if k!='pwm'),(label,wheel,metrics[wheel])
    replays[label]={'post_seed_rows':126,'seed_shift':shift,'metrics':metrics,'rows':rows}
    assert int(next(v for v in rows if v['lr_restart_active'])['t'])==542863
    assert int(next(b for a,b in zip(rows,rows[1:]) if a['lr_restart_active'] and not b['lr_restart_active'])['t'])==542923
    assert max(v['rf_recovery'] for v in rows)==0
    assert int(next(v for v in rows if 542763<=v['t'] and v['rf_overspeed_ms']>0)['t'])==542783
    assert int(next(b for a,b in zip(rows,rows[1:]) if 542763<=b['t'] and b['rf_load']<a['rf_load'])['t'])==542823

mid=replays['mid']['rows']
zero=[f for f in active if f['wheels']['LR']['dc']==0]
assert len(zero)==8 and zero[-1]['t']-zero[0]['t']==140
lr_entry=next(r for r in mid if r['lr_restart_active'])
lr_release=next(b for a,b in zip(mid,mid[1:]) if a['lr_restart_active'] and not b['lr_restart_active'])
window=[f for f in active if 542643<=f['t']<=543203]
with (here/'event_window.csv').open('w',newline='') as fp:
    fields=['t']+[w+'_'+k for w in ('LF','LR','RF','RR') for k in ('dc','cps','ff_x10','p_x10','i_x10','pwm')]
    writer=csv.DictWriter(fp,fieldnames=fields)
    writer.writeheader()
    writer.writerows({'t':f['t'],**{w+'_'+k:f['wheels'][w][k]
        for w in ('LF','LR','RF','RR') for k in ('dc','cps','ff_x10','p_x10','i_x10','pwm')}} for f in window)
usb=[r for r in decoded['wheel'] if r['phase']=='LEFT_motion' and 542650<=r['values'][0]<=543200]
report={'kind':'OFFLINE_DIAGNOSIS_ONLY_CONDITIONAL_REPLAY_NOT_NEW_FIRMWARE',
 'capture_sha256':{p.name:sha(p) for p in capture.iterdir() if p.is_file()},
 'snapshot_sha256':snapshot,'production_source_test_hashes_unchanged':len(manifest['files']),
 'all_four_firmware_artifacts_unchanged':True,
 'LR_zero_interval':{'first_zero_sample_ms':zero[0]['t'],'last_zero_sample_ms':zero[-1]['t'],
   'frames':8,'covered_sample_period_ms':160,'last_motion_sample_ms':zero[0]['t']-20,
   'first_recovered_sample_ms':zero[-1]['t']+20,
   'printed_pwm_during_zero':[f['wheels']['LR']['pwm'] for f in zero],
   'conditional_restart_entry_ms':int(lr_entry['t']),
   'conditional_restart_release_ms':int(lr_release['t'])},
 'RF_conditional_branches':{'first_overspeed_qualification_ms':542783,
   'first_load_withdrawal_after_qualification_ms':542823,
   'recovery_term_max_permille':max(r['rf_recovery'] for r in mid),
   'load_min_max_permille':[min(r['rf_load'] for r in mid),max(r['rf_load'] for r in mid)]},
 'conditional_replay':{k:{n:v for n,v in value.items() if n!='rows'} for k,value in replays.items()},
 'conditional_event_states':[r for r in mid if int(r['t']) in
   [542723,542743,542763,542783,542803,542823,542843,542863,542883,542903,542923,543123,543203]],
 'usb_event_rows':usb,
 'serial_opened':False,'motion_sent':False,'fault_cleared':False,
 'production_modified':False,'firmware_linked':False,'flashed':False,
 'limits':['Initial hidden controller state is inferred, not a hardware measurement.',
  'Three quantization seeds are sensitivity checks, not enumeration of all hidden histories.',
  'Prescribed encoder counts are not a mechanical, electrical or motor-current model.',
  'A joint LR slowdown/RF acceleration does not uniquely prove load transfer or causal direction.',
  'The ring covers only the final 2.54 seconds, not first startup.']}
check_unchanged()
(here/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in
 ['capture_sha256','snapshot_sha256','usb_event_rows','conditional_event_states']},indent=2))
