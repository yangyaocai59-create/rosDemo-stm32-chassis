"""Fit four-wheel bidirectional voltage feedforward from a steady-state CSV.

Offline only. Never opens a device or edits firmware parameters. Required CSV:
wheel,target_cps,measured_cps,pwm_permille,supply_mv,steady,valid
Use actual applied PWM and synchronized measured supply, not requested output
or an assumed battery voltage. Each fit describes the supplied load condition.
"""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import statistics

WHEELS=('LF','LR','RF','RR')
REQUIRED={'wheel','target_cps','measured_cps','pwm_permille','supply_mv','steady','valid'}


def linear_fit(points):
    if len(points)<30:
        raise ValueError('need at least 30 valid steady samples per wheel/direction')
    speeds=[p[0] for p in points]
    if max(speeds)-min(speeds)<100 or len({round(s/25) for s in speeds})<3:
        raise ValueError('insufficient speed coverage (span >=100cps and >=3 bins required)')
    def solve(rows):
        mx=statistics.fmean(x for x,y in rows)
        my=statistics.fmean(y for x,y in rows)
        variance=sum((x-mx)**2 for x,y in rows)
        if variance<=0: raise ValueError('singular speed samples')
        kv=sum((x-mx)*(y-my) for x,y in rows)/variance
        return my-kv*mx,kv
    kept=list(points)
    for _ in range(3):
        ks,kv=solve(kept)
        residual=[y-ks-kv*x for x,y in kept]
        centre=statistics.median(residual)
        mad=statistics.median(abs(r-centre) for r in residual)
        threshold=max(.03,4.5*1.4826*mad)
        candidate=[p for p,r in zip(kept,residual) if abs(r-centre)<=threshold]
        if len(candidate)<30: raise ValueError('too few samples after residual rejection')
        if len(candidate)==len(kept): break
        kept=candidate
    ks,kv=solve(kept)
    if ks<0 or kv<=0 or not math.isfinite(ks+kv):
        raise ValueError('nonphysical fitted coefficients; inspect samples/load/model')
    # Recheck coverage after outlier rejection rather than accepting a narrow fit.
    if max(x for x,y in kept)-min(x for x,y in kept)<100:
        raise ValueError('speed coverage lost during residual rejection')
    residual=[y-ks-kv*x for x,y in kept]
    mean=statistics.fmean(y for x,y in kept)
    total=sum((y-mean)**2 for x,y in kept)
    square=sum(r*r for r in residual)
    return {'ks_volts':ks,'kv_volts_per_cps':kv,'ka_enabled':False,
            'samples':len(kept),'residual_rejected':len(points)-len(kept),
            'speed_range_cps':[min(x for x,y in kept),max(x for x,y in kept)],
            'residual_rms_volts':math.sqrt(square/len(kept)),
            'r_squared':1-square/total if total>0 else None}


def fit_csv(path,condition):
    points=defaultdict(list)
    rejected=Counter()
    rows=0
    with Path(path).open(newline='',encoding='utf-8-sig') as stream:
        reader=csv.DictReader(stream)
        if not REQUIRED.issubset(reader.fieldnames or []):
            raise ValueError('missing required columns: '+','.join(sorted(REQUIRED-set(reader.fieldnames or []))))
        for row in reader:
            rows+=1
            wheel=row['wheel'].strip().upper()
            if wheel not in WHEELS: raise ValueError(f'row {rows}: invalid wheel')
            values=[float(row[k]) for k in ('target_cps','measured_cps','pwm_permille','supply_mv','steady','valid')]
            if not all(map(math.isfinite,values)): raise ValueError(f'row {rows}: nonfinite value')
            target,speed,pwm,supply,steady,valid=values
            if valid!=1 or steady!=1:
                rejected['not_valid_steady']+=1;continue
            if not 6000<=supply<=16000:
                rejected['supply_outside_3s_range']+=1;continue
            if not 0<abs(pwm)<300:
                rejected['zero_or_pwm_saturated']+=1;continue
            if abs(speed)<20 or target*speed<=0 or pwm*speed<=0:
                rejected['low_speed_or_direction']+=1;continue
            if abs(target-speed)>max(10,.2*abs(target)):
                rejected['tracking_error']+=1;continue
            # Additional source flags are optional, but a present fault is never ignored.
            flag_names=('quality_flags','output_flags','fault')
            if any(float(row.get(k,'0') or '0')!=0 for k in flag_names):
                rejected['source_fault_or_limit']+=1;continue
            points[(wheel,'forward' if speed>0 else 'reverse')].append(
                (abs(speed),abs(pwm)*supply/1000000))
    fits={}
    errors={}
    for wheel in WHEELS:
        fits[wheel]={}
        for direction in ('forward','reverse'):
            try: fits[wheel][direction]=linear_fit(points[(wheel,direction)])
            except ValueError as error: errors[wheel+'/'+direction]=str(error)
    return {'schema':1,'condition':condition,'source':str(Path(path).resolve()),
            'model':'U=ks*sign(speed)+kv*speed; steady-state only',
            'status':'OFFLINE_FIT_REQUIRES_REVIEW','firmware_parameters_changed':False,
            'automatic_voltage_compensation_enabled':False,
            'input_rows':rows,'rejected':dict(rejected),'fits':fits,'errors':errors}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path)
    parser.add_argument('--condition',required=True,help='e.g. unloaded or loaded_pivot_floorA')
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    try:
        result=fit_csv(args.csv,args.condition)
        # Fit reports are evidence: require a fresh output path.
        with args.output.open('x',encoding='utf-8') as stream:
            json.dump(result,stream,indent=2,allow_nan=False)
    except (ValueError,OSError) as error:
        parser.exit(1,str(error)+'\n')
    print(f'{"PASS" if not result["errors"] else "INCOMPLETE"}: {args.output}')
    if result['errors']: parser.exit(2)


if __name__=='__main__': main()
