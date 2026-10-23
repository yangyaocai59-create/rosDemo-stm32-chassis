"""Offline checks of archived captures, reference-window guards and artifacts."""
from pathlib import Path
import sys, json, math, struct, hashlib, zipfile, copy
root=Path.cwd();sys.path.insert(0,str(root/'Tools'))
from assess_existing_turn_yaw import assess as reference_assess
from assess_pivot_run import assess
evidence=root/'docs/stm32_evidence/2026-10-23-v0191-calibration-repeat-right'
first=root/'docs/stm32_evidence/2026-10-23-v0191-ground-left-400mrad-15s/capture'
results={}
for label,folder in (('first_left',first),('repeat_left',evidence/'left_repeat'),('right',evidence/'right')):
 r=assess(folder,.395,.065,1560)
 assert r['source_time_at_least_15s'] and r['final_stopped_verified'] and r['ports_closed']
 assert not r['capture_error'] and not r['trace_error'] and not r['rejected_delta'] and not r['host_crc_errors']
 assert r['protocol_counter_deltas']==[0,0,0] and r['imu_reference_count_gaps']==[]
 assert len([e for e in r['events_this_run'] if e['kind']==1])==1
 counts=r['net_counts'];direction=1 if label!='right' else -1
 assert all(direction*c<0 for c in counts[:2]) and all(direction*c>0 for c in counts[2:])
 hand=(sum(counts[2:])-sum(counts[:2]))*90*.065/(1560*.395)
 assert math.isclose(hand,r['encoder_angle_deg'],abs_tol=1e-10)
 results[label]=r
repeat_delta=results['repeat_left']['encoder_angle_deg']-results['first_left']['encoder_angle_deg']
cal=json.loads((evidence/'calibration/result.json').read_text(encoding='utf-8'))
reboot=json.loads((evidence/'reboot_readonly/result.json').read_text(encoding='utf-8'))
assert cal['pass'] and cal['calibration_commands']==1 and not cal['motion_sent'] and not cal['clear_sent']
assert reboot['imu_last'][0][12]==7 and not reboot.get('error') and not reboot['debug_error']
assert reboot['status_first_last'][0][0]<cal['status_first_last'][0][0]
assert reboot['wheel_first_last'][0][1:5]==[0,0,0,0]
# Real capture has one changed duplicate outside the reference window.
right=evidence/'right';actual=reference_assess(right)
assert len(actual['same_sample_updates_outside_reference_window'])==1
summary=json.loads((right/'summary.json').read_text(encoding='utf-8'))
frames=json.loads((right/'usb_frames.json').read_text(encoding='utf-8'))
changed=copy.deepcopy(frames)
for index,frame in enumerate(changed):
 if frame['type']==132:
  row=list(struct.unpack('<IIiiiiiiiIIBB',bytes.fromhex(frame['payload_hex'])))
  if summary['reference_start_source_ms']+100<row[0]<summary['reference_end_source_ms']:
   row[7]+=1;duplicate=dict(frame);duplicate['payload_hex']=struct.pack('<IIiiiiiiiIIBB',*row).hex()
   changed.insert(index+1,duplicate);break
else:raise AssertionError('No reference sample for guard check')
class FileText:
 def __init__(self,text):self.text=text
 def read_text(self,*args,**kwargs):return self.text
class AlteredFolder:
 def __truediv__(self,name):return FileText(json.dumps(changed if name=='usb_frames.json' else summary))
try:reference_assess(AlteredFolder())
except ValueError as error:assert 'within reference/baseline window' in str(error)
else:raise AssertionError('Changed duplicate inside reference window was accepted')
archive=root/'releases/0.19.1/rosDemo-0.19.1-existing-image.zip'
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 for name in z.namelist():
  if name!='SOURCE_REVISION.txt':assert z.read(name)==(root/'build/firmware/0.19.1'/name).read_bytes()
comparison={'runs':results,'left_repeat_encoder_delta_deg':repeat_delta,'left_repeat_encoder_delta_percent':100*repeat_delta/abs(results['first_left']['encoder_angle_deg']),'left_repeat_imu_delta_deg':results['repeat_left']['imu_reference_deg']-results['first_left']['imu_reference_deg'],'operator_reports_same_floor_payload_tyres':True,'navigation_acceptance':False}
(evidence/'comparison.json').write_text(json.dumps(comparison,indent=2)+'\n',encoding='utf-8')
checks={'three_capture_angle_duration_stop_protocol_checks':'PASS','independent_encoder_formula':'PASS','calibration_and_reboot_flags':'PASS','real_outside_window_duplicate_recorded':'PASS','in_window_changed_duplicate_rejected':'PASS','firmware_archive_exact_bytes':'PASS','left_repeat_encoder_delta_deg':repeat_delta,'left_repeat_encoder_delta_percent':comparison['left_repeat_encoder_delta_percent']}
(evidence/'offline_delivery_checks.json').write_text(json.dumps(checks,indent=2)+'\n',encoding='utf-8')
print(json.dumps(checks,indent=2))
