"""Compare encoder and bias-corrected gyro yaw on identical source-time bounds.

Offline only. Encoder endpoints are actual telemetry counters; only gyro rate
is linearly interpolated at the window boundaries before trapezoid integration.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import statistics
import struct


def gyro_radians(rows, begin, end, bias):
    if not rows or rows[0][0] > begin or rows[-1][0] < end or end <= begin:
        raise ValueError("IMU does not cover the comparison window")
    total = 0.
    for a,b in zip(rows,rows[1:]):
        lo,hi = max(begin,a[0]),min(end,b[0])
        if hi <= lo:
            continue
        dt = b[0]-a[0]
        x = a[7]+(b[7]-a[7])*(lo-a[0])/dt
        y = a[7]+(b[7]-a[7])*(hi-a[0])/dt
        total += ((x+y)/2-bias)*(hi-lo)/1e6
    return total


def encoder_degrees(before, after, diameter=.065, counts=1560, track=.395):
    net = [b-a for a,b in zip(before[1:5],after[1:5])]
    radians = ((net[2]+net[3])-(net[0]+net[1]))/2*math.pi*diameter/counts/track
    return math.degrees(radians)


def compare(folder, plot=False):
    summary = json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    if not summary.get('compare_yaw') or summary['motion_attempts'] != 1 or summary['error']:
        raise ValueError("requires one successful motion with yaw baselines")
    frames = json.loads((folder/'usb_frames.json').read_text(encoding='utf-8'))
    log = (folder/'ch340.log').read_text(encoding='utf-8')
    events = [(int(t),int(k)) for t,k in re.findall(r'RUNTIME_EVENT seq=\d+ t=(\d+) kind=(\d+)',log)
              if int(t)>=summary['initial_wheel'][0]]
    start = next(t for t,k in events if k==1)
    stop = next(t for t,k in events if k==2 and t>start)
    unique = {}
    phases = {'baseline':{},'post_baseline':{}}
    wheels = {}
    for frame in frames:
        data = bytes.fromhex(frame['payload_hex'])
        if frame['type']==132:
            row = struct.unpack('<IIiiiiiiiIIBB',data)
            if row[0] in unique and unique[row[0]]!=row:
                raise ValueError('conflicting IMU duplicate')
            unique[row[0]] = row
            if frame['phase'] in phases:
                phases[frame['phase']][row[0]] = row
        if frame['type']==130:
            row = struct.unpack('<Iqqqqiiii',data)
            if row[0] in wheels and wheels[row[0]]!=row:
                raise ValueError('conflicting encoder duplicate')
            wheels[row[0]] = row
    rows = sorted(unique.values())
    before = tuple(summary['initial_wheel'])
    after = next(w for w in sorted(wheels.values()) if w[0]>=stop+1000 and not any(w[5:]))
    begin,end = before[0],after[0]
    window = [r for r in rows if begin-30<=r[0]<=end+30]
    if any(r[11]!=2 or not r[12]&1 or r[10] for r in window):
        raise ValueError('invalid IMU during comparison')
    intervals = [b[0]-a[0] for a,b in zip(window,window[1:])]
    jumps = [(b[1]-a[1])&0xffffffff for a,b in zip(window,window[1:])]
    if not intervals or min(intervals)<=0 or max(intervals)>30 or any(j<1 or j>3 for j in jumps):
        raise ValueError('IMU source gap/reset beyond comparison bounds')
    if len({r[9] for r in window})!=1:
        raise ValueError('IMU read-error count changed during comparison')
    baseline_info = {}
    for name,data in phases.items():
        vals = sorted(data.values())
        if len(vals)<250 or any(r[11]!=2 or not r[12]&1 or r[10] for r in vals):
            raise ValueError('insufficient/invalid '+name)
        if statistics.pstdev(r[7] for r in vals)>3:
            raise ValueError('unstable '+name)
        baseline_info[name] = dict(samples=len(vals),span_ms=vals[-1][0]-vals[0][0],
            mean_z_mrad_s=statistics.mean(r[7] for r in vals),std_z_mrad_s=statistics.pstdev(r[7] for r in vals))
    bias = baseline_info['baseline']['mean_z_mrad_s']
    post_bias = baseline_info['post_baseline']['mean_z_mrad_s']
    enc = encoder_degrees(before,after)
    imu = math.degrees(gyro_radians(rows,begin,end,bias))
    imu_post = math.degrees(gyro_radians(rows,begin,end,post_bias))
    difference = enc-imu
    report = dict(firmware_user_report=summary['firmware_user_report'],direction=summary['case'],
        requested_rad_s=summary['command_mm_s_mrad_s'][1]/1000,
        host_command_seconds=summary['motion_command_seconds'],mcu_active_seconds=(stop-start)/1000,
        mcu_start_ms=start,mcu_stop_ms=stop,comparison_window_ms=[begin,end],
        comparison_seconds_including_settling=(end-begin)/1000,
        geometry=dict(diameter_m=.065,counts_per_rev=1560,effective_track_m=.395),
        before_wheel=before,after_wheel=after,net_counts=[b-a for a,b in zip(before[1:5],after[1:5])],
        encoder_angle_deg=enc,imu_reference_angle_deg=imu,encoder_minus_imu_deg=difference,
        absolute_difference_deg=abs(difference),relative_difference_percent=100*abs(difference)/abs(imu),
        percent_denominator='absolute bias-corrected IMU reference angle',
        imu_uncorrected_angle_deg=math.degrees(gyro_radians(rows,begin,end,0)),
        imu_using_post_baseline_deg=imu_post,
        bias_sensitivity_deg=abs(imu_post-imu),baselines=baseline_info,
        imu_source_max_interval_ms=max(intervals),imu_source_missing_samples=sum(j-1 for j in jumps),
        imu_saved_bias_valid=all(bool(r[12]&2) for r in window),external_angle_truth_available=False,
        final_stopped_verified=summary['final_stopped_verified'],
        command_active_integral_deg=(None if summary.get('target_imu_deg') else
            math.degrees(summary['command_mm_s_mrad_s'][1]*(stop-start)/1e6)),
        target_imu_deg=summary.get('target_imu_deg'),angular_rate_schedule=summary.get('angular_rate_schedule','constant'))
    if summary.get('target_imu_deg'):
        report['imu_final_minus_target_deg'] = imu-(summary['target_imu_deg'] if summary['case']=='left' else -summary['target_imu_deg'])
    curve = []
    for row in sorted(wheels.values()):
        if not begin<=row[0]<=end:
            continue
        enc_at = encoder_degrees(before,row)
        imu_at = math.degrees(gyro_radians(rows,begin,row[0],bias)) if row[0]>begin else 0.
        curve.append(dict(elapsed_s=(row[0]-begin)/1000,encoder_deg=enc_at,imu_deg=imu_at,difference_deg=enc_at-imu_at))
    with (folder/'yaw_curve.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=curve[0].keys());writer.writeheader();writer.writerows(curve)
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True,layout='constrained')
        x=[r['elapsed_s'] for r in curve]
        axes[0].plot(x,[r['encoder_deg'] for r in curve],label='Encoders (effective track 395 mm)')
        axes[0].plot(x,[r['imu_deg'] for r in curve],label='IMU (stationary bias subtracted)')
        axes[0].set_ylabel('Yaw angle (deg)');axes[0].legend()
        axes[1].plot(x,[r['difference_deg'] for r in curve],color='tab:red',label='Encoders - IMU')
        axes[1].axhline(0,color='gray',linewidth=.7);axes[1].set_ylabel('Difference (deg)');axes[1].set_xlabel('MCU source time (s)')
        for ax in axes:
            ax.axvline((stop-begin)/1000,color='gray',linestyle='--',label='STOP')
            ax.grid(alpha=.25)
        if summary.get('target_imu_deg'):
            axes[0].axhline(summary['target_imu_deg'] if summary['case']=='left' else -summary['target_imu_deg'],color='green',linestyle=':',label='IMU target')
            axes[0].legend()
            fig.suptitle('0.20.2: IMU target 180 deg, '+summary['case']+' pivot, tapered speed')
        else:
            fig.suptitle('0.20.2: '+str(summary['planned_seconds'])+' s '+summary['case']+' pivot, 0.40 rad/s')
        fig.savefig(folder/'yaw_comparison.png',dpi=160)
        fig.savefig(folder/'yaw_comparison.svg')
        plt.close(fig)
    (folder/'yaw_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    return report


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('folder',type=Path,nargs='?');ap.add_argument('--plot',action='store_true');ap.add_argument('--self-test',action='store_true')
    args=ap.parse_args()
    if args.self_test:
        rows=[(t,0,0,0,0,0,0,440) for t in range(0,2101,10)]
        assert abs(gyro_radians(rows,50,2050,40)-.8)<1e-12
        assert abs(gyro_radians(rows,50,2050,440))<1e-12
        assert encoder_degrees((0,0,0,0,0),(0,-100,-100,100,100))>0
        print('PASS: bias subtraction, clipped common time window, units and yaw sign; no device access')
    elif args.folder:
        print(json.dumps(compare(args.folder,args.plot),indent=2))
    else:
        ap.error('folder or --self-test required')
