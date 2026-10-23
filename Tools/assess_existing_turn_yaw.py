"""Offline exploratory wheel/IMU yaw comparison; no ground-truth acceptance.

Uses the capture's short stationary residual baseline and reports its observed
bias-valid flags. Does not alter counts, calibrate a device or select geometry.
"""
import argparse,json,math,statistics,struct
from pathlib import Path

def assess(folder):
    s=json.loads((folder/'summary.json').read_text())
    frames=json.loads((folder/'usb_frames.json').read_text())
    statuses=[struct.unpack('<IIIIHBBBBHhhhhhhIIII',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==129]
    start=s['initial_status'][0]
    active=[f for f in statuses if f[5]==2]
    if not active: raise ValueError('no ACTIVE status')
    stopped=next(f for f in statuses if f[0]>active[-1][0] and f[5]!=2)
    end=stopped[0]+1000
    imu=[struct.unpack('<IIiiiiiiiIIBB',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==132]
    unique=[];same_sample_updates=[]
    for row in imu:
        if unique and row[0:2]==unique[-1][0:2]:
            if row!=unique[-1]:
                if start-800<=row[0]<=end:
                    raise ValueError('same IMU sample ID has different payload within reference/baseline window')
                same_sample_updates.append({'time_ms':row[0],'previous':unique[-1],'updated':row})
                unique[-1]=row
            continue
        unique.append(row)
    imu=unique
    # Source timestamps, never use UART export timestamps for motion duration.
    pre=[i for i in imu if start-800<=i[0]<=start and i[11]==2 and i[12]&1]
    post=[i for i in imu if i[0]>=imu[-1][0]-5000 and i[11]==2 and i[12]&1]
    if len(pre)<20 or len(post)<20: raise ValueError('stationary baseline samples insufficient')
    baseline=statistics.mean(i[7] for i in pre)
    postmean=statistics.mean(i[7] for i in post)
    # Include settlement; samples must have contiguous source IDs and <=30ms gaps.
    samples=[i for i in imu if start<=i[0]<=end and i[11]==2 and i[12]&1]
    gaps=[b[0]-a[0] for a,b in zip(samples,samples[1:])]
    if not gaps or max(gaps)>30 or min(gaps)<=0: raise ValueError('invalid IMU time gaps')
    jumps=[{'t':b[0],'jump':b[1]-a[1]} for a,b in zip(samples,samples[1:]) if b[1]-a[1]!=1]
    if any(j['jump']<=0 or j['jump']>3 for j in jumps): raise ValueError('large/nonmonotonic sample gap')
    def integrate(bias):
        return sum(((a[7]+b[7])/2-bias)*(b[0]-a[0])/1e6 for a,b in zip(samples,samples[1:]))*180/math.pi
    net=[b-a for a,b in zip(s['initial_wheel'][1:5],s['final_wheel'][1:5])]
    diff=((net[2]+net[3])-(net[0]+net[1]))/2
    # Existing ROS effective track is a stated assumption, not fitted here.
    wheel_yaw=diff*(math.pi*.065/1560)/.395*180/math.pi
    estimate=integrate(baseline)
    return {'folder':str(folder),'motion_seconds':s['motion_seconds'],
        'net_counts':net,'assumed_wheel_diameter_m':.065,'assumed_effective_track_m':.395,
        'wheel_yaw_deg':wheel_yaw,'imu_yaw_short_prebaseline_deg':estimate,
        'wheel_minus_imu_deg':wheel_yaw-estimate,
        'pre_baseline_samples':len(pre),'pre_baseline_mrad_s':baseline,
        'pre_baseline_std_mrad_s':statistics.pstdev(i[7] for i in pre),
        'post_baseline_mrad_s':postmean,
        'imu_yaw_using_postbaseline_deg':integrate(postmean),
        'source_window_ms':[samples[0][0],samples[-1][0]],'max_imu_interval_ms':max(gaps),
        'imu_source_count_jumps':jumps,
        'same_sample_updates_outside_reference_window':same_sample_updates,
        'bias_valid_flags':sorted({bool(i[12]&2) for i in samples}),
        'external_angle_ground_truth_available':False,
        'acceptance_pass':None,'warning':'Exploratory reference comparison only; short baseline and no external angle truth. Bias validity is reported separately. Do not fit track or declare acceptance.'}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('folders',type=Path,nargs='+');ap.add_argument('--output',type=Path)
    args=ap.parse_args();result=[assess(p) for p in args.folders]
    text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text,encoding='utf-8')
    print(text)
if __name__=='__main__':main()
