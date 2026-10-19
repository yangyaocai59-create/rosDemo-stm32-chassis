"""Parse the saved MCU trace; never accesses hardware."""
from pathlib import Path
import collections,csv,json,math,re,statistics
P=Path(__file__).resolve().parent
H=re.compile(r'TRACE t=(\d+) dt=(\d+) req=(-?\d+)/(-?\d+) applied=(-?\d+)/(-?\d+) state=(\d+) stop=(\d+) fault=(\d+) stby=(\d+)')
W=re.compile(r'TRACE (LF|LR|RF|RR) dc=(-?\d+) raw=(-?\d+) target=(-?\d+) cps=(-?\d+) ff_x10=(-?\d+) p_x10=(-?\d+) i_x10=(-?\d+) pwm=(-?\d+)')
text=(P/'uart_rx.bin').read_bytes().decode(errors='replace')
frames=[];frame=None;errors=[]
for line in text.splitlines():
 h=H.search(line);w=W.search(line)
 if h:
  if frame is not None:
   if len(frame['wheels'])!=4:errors.append({'t_ms':frame['t_ms'],'wheel_count':len(frame['wheels'])})
   frames.append(frame)
  keys=['t_ms','dt_ms','req_vx','req_wz','applied_vx','applied_wz','state','stop','fault','standby']
  frame=dict(zip(keys,map(int,h.groups())));frame['wheels']={}
 elif w and frame is not None:
  key=w[1]
  if key in frame['wheels']:errors.append({'t_ms':frame['t_ms'],'duplicate_wheel':key})
  frame['wheels'][key]=dict(zip(['delta_count','raw_cps','target_cps','filtered_cps','ff_x10','p_x10','i_x10','pwm_permille'],map(int,w.groups()[1:])))
if frame is not None:
 if len(frame['wheels'])!=4:errors.append({'t_ms':frame['t_ms'],'wheel_count':len(frame['wheels'])})
 frames.append(frame)
assert 'TRACE END' in text and frames and not errors,(len(frames),errors)
assert all(b['t_ms']>a['t_ms'] for a,b in zip(frames,frames[1:])), 'Trace timestamps not strictly increasing'
active=[f for f in frames if f['state']==2]
steady=[f for f in active if f['req_vx']==0 and f['req_wz']==200 and f['applied_wz']==200]
assert active and steady
stats={}
for wheel in ['LF','LR','RF','RR']:
 data=[f['wheels'][wheel] for f in steady]
 mean=lambda key:statistics.mean(d[key] for d in data)
 stats[wheel]={'samples':len(data),'mean_target_cps':mean('target_cps'),'mean_filtered_cps':mean('filtered_cps'),
   'mean_raw_cps':mean('raw_cps'),'mean_abs_pwm_permille':statistics.mean(abs(d['pwm_permille']) for d in data),
   'min_max_pwm_permille':[min(d['pwm_permille'] for d in data),max(d['pwm_permille'] for d in data)],
   'min_max_integrator_permille':[min(d['i_x10'] for d in data)/10,max(d['i_x10'] for d in data)/10],
   'fraction_integrator_at_50_permille':sum(abs(d['i_x10'])>=499 for d in data)/len(data),
   'fraction_pwm_at_300_permille':sum(abs(d['pwm_permille'])>=300 for d in data)/len(data),
   'fraction_abs_filtered_cps_le20':sum(abs(d['filtered_cps'])<=20 for d in data)/len(data)}
 # FF includes the reference controller's additional breakaway term. A testable
 # match against its documented table is evidence, not a separate logged flag.
 coeff={'LF':(46.3,.14677),'LR':(50.4,.14667),'RF':(54.7,.14703),'RR':(58.9,.15779)}[wheel]
 ff_extra=[]
 for d in data:
  sign=1 if d['target_cps']>=0 else -1
  base=sign*(coeff[0]+coeff[1]*abs(d['target_cps']))
  ff_extra.append(d['ff_x10']/10-base)
 stats[wheel]['inferred_ff_extra_range_permille']=[min(ff_extra),max(ff_extra)]
 stats[wheel]['inferred_ff_extra_abs_gt40_samples']=sum(abs(x)>40 for x in ff_extra)
 flags=[abs(x)>40 for x in ff_extra]
 stats[wheel]['inferred_assistance_transitions']=sum(a!=b for a,b in zip(flags,flags[1:]))
 stats[wheel]['mean_abs_measured_to_abs_target_ratio']=abs(mean('filtered_cps')/mean('target_cps'))
 stats[wheel]['maximum_pwm_change_between_steady_trace_frames_permille']=max(abs(b['pwm_permille']-a['pwm_permille']) for a,b in zip(data,data[1:]))
result={'trace_frames':len(frames),'trace_span_ms':frames[-1]['t_ms']-frames[0]['t_ms'],
 'first_last_source_ms':[frames[0]['t_ms'],frames[-1]['t_ms']],
 'max_trace_gap_ms':max(b['t_ms']-a['t_ms'] for a,b in zip(frames,frames[1:])),
 'active_samples':len(active),'steady_applied_200_samples':len(steady),
 'requested_commands':sorted(set((f['req_vx'],f['req_wz']) for f in active)),
 'applied_wz_range':[min(f['applied_wz'] for f in active),max(f['applied_wz'] for f in active)],
 'first_applied_200_from_first_record_ms':steady[0]['t_ms']-frames[0]['t_ms'],
 'any_fault':any(f['fault'] for f in frames),'last_frame':frames[-1],
 'steady_wheel_statistics':stats,
 'scope':'Actual UART-dumped on-board fields, integer-truncated by firmware printf; initial ring coverage must be checked against command duration. FF-extra is inferred from reference table, not a logged breakaway flag. Ground-truth body angle not measured.'}
(P/'recomputed_trace_frames.json').write_text(json.dumps(frames,indent=2)+'\n')
(P/'recomputed_trace_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
with (P/'recomputed_trace_wheels.csv').open('w') as out:
 writer=csv.writer(out,lineterminator="\n");writer.writerow(['t_ms','dt_ms','req_wz','applied_wz','state','wheel','target_cps','filtered_cps','raw_cps','delta_count','ff_permille','p_permille','i_permille','pwm_permille'])
 for f in frames:
  for name,d in f['wheels'].items():writer.writerow([f['t_ms'],f['dt_ms'],f['req_wz'],f['applied_wz'],f['state'],name,d['target_cps'],d['filtered_cps'],d['raw_cps'],d['delta_count'],d['ff_x10']/10,d['p_x10']/10,d['i_x10']/10,d['pwm_permille']])
print(json.dumps(result,indent=2))
