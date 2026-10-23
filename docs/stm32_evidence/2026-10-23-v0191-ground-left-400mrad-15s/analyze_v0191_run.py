"""Offline .19.1 angle/duration assessment. No serial imports or writes."""
from pathlib import Path
import json,struct,math,re,hashlib,statistics
p=Path('build/hardware_tests/v0191_400mrad_ground_left_20261023_091152')
s=json.loads((p/'summary.json').read_text());frames=json.loads((p/'usb_frames.json').read_text())
log=(p/'ch340.log').read_text();events=[]
for line in log.splitlines():
 m=re.search(r'RUNTIME_EVENT seq=(\d+) t=(\d+) kind=(\d+) state=(\d+) generation=(\d+) command=(\d+)',line)
 if m:events.append(dict(zip(['sequence','time_ms','kind','state','generation','command'],map(int,m.groups()))))
start=next(e for e in events if e['kind']==1 and e['time_ms']>=s['initial_wheel'][0]);stop=next(e for e in events if e['kind']==2 and e['time_ms']>start['time_ms'])
dt=((stop['time_ms']-start['time_ms'])&0xffffffff)/1000
net=[b-a for a,b in zip(s['initial_wheel'][1:5],s['final_wheel'][1:5])]
difference=((net[2]+net[3])-(net[0]+net[1]))/2
encoder=difference*(math.pi*.065/1560)/.395*180/math.pi
imu=s['imu_reference_final_deg'];cmd=s['planned_rad_s']*dt*180/math.pi
statuses=[struct.unpack('<IIIIHBBBBHhhhhhhIIII',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==129]
battery=[struct.unpack('<IIIHHHHBBHHH',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==131]
source=[struct.unpack('<IIiiiiiiiIIBB',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==132]
post=[i[7] for i in source if i[0]>source[-1][0]-3000 and i[11]==2 and i[12]&1]
r={'host_motion_seconds':s['motion_seconds'],'mcu_active_span_s':dt,'source_time_at_least_15s':dt>=15,'mcu_start_ms':start['time_ms'],'mcu_stop_request_ms':stop['time_ms'],'requested_rad_s':s['planned_rad_s'],'net_counts':net,'assumed_effective_track_m':.395,'encoder_angle_deg':encoder,'imu_reference_deg':imu,'encoder_minus_imu_deg':encoder-imu,'encoder_minus_imu_percent':100*(encoder-imu)/imu,'command_integral_source_deg':cmd,'command_integral_host_deg':s['planned_rad_s']*s['motion_seconds']*180/math.pi,'reference_mean_rad_s':imu*math.pi/180/dt,'encoder_mean_rad_s':encoder*math.pi/180/dt,'reference_baseline_mrad_s':s['reference_baseline_mrad_s'],'reference_baseline_std_mrad_s':s['reference_baseline_std_mrad_s'],'post_stationary_baseline_mrad_s':statistics.mean(post),'post_stationary_std_mrad_s':statistics.pstdev(post),'events':events,'new_accepted':statuses[-1][2]-statuses[0][2],'new_rejected':statuses[-1][3]-statuses[0][3],'protocol_counter_deltas':[statuses[-1][i]-statuses[0][i] for i in (17,18,19)],'host_crc_errors':s['host_crc_errors'],'vm_mV_range':[min(b[6] for b in battery),max(b[6] for b in battery)],'ports_closed':s['ports_closed'],'final_stopped_verified':s['final_stopped_verified'],'external_angle_ground_truth_available':False,'acceptance_threshold_selected':False}
(p/'angle_comparison.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
