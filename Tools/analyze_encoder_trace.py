"""Offline schema3 raw-counter consistency checks. No serial/HAL writes."""
import argparse,json
from pathlib import Path
from parse_motion_trace_v2 import parse

def delta(current,previous,bits):
    modulus=1<<bits
    result=(current-previous)&(modulus-1)
    return result-modulus if result>=modulus//2 else result

def analyze(capture):
    frames=capture['frames']
    if capture['header']['schema']!=3: raise ValueError('schema3 required')
    result={'header':capture['header'],'source_span_ms':[frames[0]['t'],frames[-1]['t']],
            'snapshot_flags':sorted({f['snapshot']['flags'] for f in frames}),'wheels':{}}
    for index,name in enumerate(('LF','LR','RF','RR')):
        bits=32 if index==1 else 16; sign=1 if index<2 else -1
        invalid=[]; sampled_total_mismatch=[]; live_sample_diffs=[]
        zero_runs=[]; run=[]; prior=None; configurations=[]
        for frame in frames:
            wheel=frame['wheels'][name]; enc=wheel['encoder']
            configurations.append(enc['bad'])
            if enc['bad']: invalid.append({'t':frame['t'],'bad':enc['bad']})
            live_difference=delta(enc['live'],enc['sampled'],bits)
            if live_difference: live_sample_diffs.append({'t':frame['t'],'raw_delta':live_difference})
            if prior is not None:
                p=prior['wheels'][name]
                expected=sign*delta(enc['sampled'],p['encoder']['sampled'],bits)
                got=wheel['total']-p['total']
                if expected!=got:
                    sampled_total_mismatch.append({'t':frame['t'],'raw_delta_normalized':expected,'total_delta':got})
            if frame['state']==2 and wheel['dc']==0:
                run.append(frame)
            else:
                if run: zero_runs.append(run);run=[]
            prior=frame
        if run: zero_runs.append(run)
        summaries=[]
        for group in zero_runs:
            if len(group)<3: continue
            values=[f['wheels'][name] for f in group]
            summaries.append({'first_t':group[0]['t'],'last_t':group[-1]['t'],'samples':len(group),
                'sum_sample_period_ms':sum(f['dt'] for f in group),
                'sampled_counts':sorted({v['encoder']['sampled'] for v in values}),
                'live_counts':sorted({v['encoder']['live'] for v in values}),
                'ab_values':sorted({v['encoder']['ab'] for v in values}),
                'config_bad_values':sorted({v['encoder']['bad'] for v in values}),
                'pwm_first_last':[values[0]['pwm'],values[-1]['pwm']],
                'ccr_first_last':[values[0]['ccr'],values[-1]['ccr']],
                'i_x10_first_last':[values[0]['i_x10'],values[-1]['i_x10']],
                'total_values':sorted({v['total'] for v in values})})
        result['wheels'][name]={'config_bad_values':sorted(set(configurations)),
            'config_bad_frames':invalid,'sampled_vs_total_mismatches':sampled_total_mismatch,
            'live_ahead_or_behind_frames':live_sample_diffs,'zero_runs_at_least_3_frames':summaries}
    return result

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log',type=Path);ap.add_argument('--output',type=Path)
    args=ap.parse_args()
    result=[analyze(c) for c in parse(args.log.read_text(encoding='utf-8'))]
    text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text,encoding='utf-8')
    print(text)
if __name__=='__main__':main()
