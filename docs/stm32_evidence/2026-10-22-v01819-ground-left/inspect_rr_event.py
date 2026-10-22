"""Inspect recorded RR count gaps; no serial and no physical replay."""
import csv
import json
from pathlib import Path

here=Path(__file__).resolve().parent
frames=[f for f in json.loads((here/'capture/trace_parsed.json').read_text()) if f['state']==2]
decoded=json.loads((here/'capture/decoded.json').read_text())
groups=[]
for index,f in enumerate(frames):
    if f['wheels']['RR']['dc']==0:
        if not groups or groups[-1][-1]!=index-1: groups.append([])
        groups[-1].append(index)
events=[]
csvrows=[]
for group in groups:
    first,last=group[0],group[-1]
    window=frames[max(0,first-4):min(len(frames),last+15)]
    rows=[]
    for f in window:
        r=f['wheels']['RR']
        row={'t_ms':f['t'],'rr_dc':r['dc'],'rr_cps':r['cps'],
             'rr_ff_permille':r['ff_x10']/10,'rr_i_permille':r['i_x10']/10,
             'rr_pwm_permille':r['pwm'],'lr_cps':f['wheels']['LR']['cps'],
             'rf_cps':f['wheels']['RF']['cps']}
        rows.append(row); csvrows.append(row)
    start,end=frames[first]['t'],frames[last]['t']
    usb=[{'t_ms':r['values'][0],'rr_count':r['values'][4],'rr_cps':r['values'][8]}
         for r in decoded['wheel'] if r['phase']=='LEFT_motion' and start-100<=r['values'][0]<=end+120]
    events.append({'zero_rows_t_ms':[start,end],
      'zero_intervals_ms':sum(frames[i]['dt'] for i in group),
      'pre_gap_pwm_permille':frames[first-1]['wheels']['RR']['pwm'] if first else None,
      'window':rows,'USB_RR_around_gap':usb})
report={'kind':'RECORDED_COUNTER_GAPS_NOT_CONFIRMED_PHYSICAL_STOPS',
 'RR_zero_frame_count':sum(map(len,groups)),
 'RR_zero_episode_count':len(events),'events':events,
 'notes':['RR legacy policy was not changed in .19; coupled physical response can still change.',
  'A short no-count interval alone does not identify encoder/electrical/friction/control root cause.',
  'The existing RR restart can explain a later command increase, not prove why counts first stopped.']}
(here/'rr_event.json').write_text(json.dumps(report,indent=2)+'\n')
with (here/'rr_event_window.csv').open('w',newline='') as output:
    writer=csv.DictWriter(output,fieldnames=list(csvrows[0])); writer.writeheader(); writer.writerows(csvrows)
for e in events:
    print('ZERO_EPISODE',e['zero_rows_t_ms'],e['zero_intervals_ms'],'ms')
    print(' t_ms     RRdc RRcps  FF    I    PWM   LRcps RFcps')
    for row in e['window']:
        print('{t_ms:8d} {rr_dc:4d} {rr_cps:5d} {rr_ff_permille:5.1f} {rr_i_permille:5.1f} {rr_pwm_permille:4d} {lr_cps:6d} {rf_cps:5d}'.format(**row))
    print('USB',e['USB_RR_around_gap'])
