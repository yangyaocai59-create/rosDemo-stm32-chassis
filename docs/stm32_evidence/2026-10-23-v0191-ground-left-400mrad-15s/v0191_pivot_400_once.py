import sys
from pathlib import Path
sys.path.insert(0,str(Path.cwd()/"Tools"))
"""One explicitly authorized left/right pivot, magnitude 0.40 rad/s.

Stops early on fault/transport/error. No automatic fault clears or motion retries.
Keeps CH340 open from baseline through TRACE END and saves raw USB bytes.
"""
import argparse
from datetime import datetime
import json
import math
import statistics
from pathlib import Path
import struct
import time
import serial
from debug_uart_capture import DebugUartCapture
from parse_motion_trace_v2 import parse
import usb_cdc_test as wire

def pivot_velocity_payload(direction):
    if direction not in ('left','right'): raise ValueError('invalid pivot direction')
    return struct.pack('<ii',0,400 if direction=='left' else -400)

class YawReference:
    def __init__(self,baseline):
        self.baseline=baseline;self.previous=None;self.radians=0.;self.gaps=[]
    def add(self,row):
        if row[11]!=2 or not row[12]&1: raise RuntimeError('IMU not valid READY')
        if self.previous is not None:
            previous=self.previous
            if row[:2]==previous[:2]:
                if row!=previous: raise RuntimeError('IMU duplicate source payload differs')
                return
            dt=(row[0]-previous[0])&0xffffffff
            jump=(row[1]-previous[1])&0xffffffff
            if dt==0 or dt>30 or jump==0 or jump>3:
                raise RuntimeError('IMU reference gap/reset exceeds bounds')
            if jump!=1:self.gaps.append((row[0],jump,dt))
            self.radians+=((row[7]+previous[7])/2-self.baseline)*dt/1e6
        self.previous=row

def yaw_self_test():
    def row(t,n,w):return (t,n,0,0,9800,0,0,w,25000,0,0,2,5)
    yaw=YawReference(40)
    for n in range(1001):yaw.add(row(n*10,n,340))
    assert abs(yaw.radians-3)<1e-10
    yaw.add(row(10000,1000,340));assert abs(yaw.radians-3)<1e-10
    wrap=YawReference(0);wrap.add(row(0xfffffffa,1,100));wrap.add(row(4,2,100))
    assert abs(wrap.radians-.001)<1e-10
    assert struct.unpack('<ii',pivot_velocity_payload('left'))==(0,400)
    assert struct.unpack('<ii',pivot_velocity_payload('right'))==(0,-400)
    right=YawReference(40)
    for n in range(1001):right.add(row(n*10,n,-260))
    assert abs(right.radians+3)<1e-10
    for invalid in (row(10100,1001,340),row(10010,1005,340)):
        try:yaw.add(invalid)
        except RuntimeError:pass
        else:raise AssertionError('reference gap accepted')
    print('PASS: reference bias, trapezoid integration, duplicates, time wrap and gap/reset bounds; no hardware opened')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--self-test',action='store_true')
    ap.add_argument('--seconds',type=float,default=15.2)
    ap.add_argument('--direction',choices=('left','right'),default='left')
    ap.add_argument('--target-imu-deg',type=float,choices=(180.,))
    ap.add_argument('--setup',choices=('lifted','ground'),default='lifted')
    ap.add_argument('--firmware',choices=('0.19.1',),default='0.19.1')
    ap.add_argument('--clear-known-poweroff-fault',action='store_true',
                    help='One zero-speed clear after user confirms VM restored')
    ap.add_argument('--clear-latched-fault-once',action='store_true',
                    help='One authorized zero-speed health-fault clear before repeat')
    ap.add_argument('--port',default='COM4')
    ap.add_argument('--debug-port',default='COM8')
    args=ap.parse_args()
    if args.self_test:yaw_self_test();return 0
    if args.seconds != 15.2:ap.error('this authorized capture allows exactly 15.2 seconds')
    if args.target_imu_deg is not None and args.setup!='ground':ap.error('angle reference test requires ground setup')
    if not args.execute: ap.error('--execute required after physical setup confirmation')
    direction_sign=1 if args.direction=='left' else -1
    velocity_payload=pivot_velocity_payload(args.direction)
    folder=Path.cwd()/'build/hardware_tests'/('v'+args.firmware.replace('.','')+'_400mrad_'+args.setup+'_'+args.direction+'_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True,exist_ok=False)
    capture=DebugUartCapture(args.debug_port,folder)
    usb=None; frames=[]; raw=bytearray(); parser=wire.StreamParser(); sequence=0
    status=None; status_host=0.; stop_ack=False; wheel=None; zero_since=None; battery=None
    imu_rows=[]; imu_host=0.;yaw=None; yaw_active=False
    summary={'planned_rad_s':direction_sign*.40,'direction':args.direction,'planned_seconds':args.seconds,'setup':args.setup+', user confirmed',
             'target_imu_reference_deg':args.target_imu_deg,
             'firmware_user_report':args.firmware,
             'fault_cleared':False,'motion_attempts':0,'error':None,'trace_error':None,
             'folder':str(folder),'final_stopped_verified':False}
    def send(kind,payload=b''):
        nonlocal sequence
        sequence=(sequence+1)&65535
        usb.write(wire.encode_frame(kind,sequence,payload))
        if kind==wire.CMD_VELOCITY: summary['velocity_packets']=summary.get('velocity_packets',0)+1
    def pump(seconds,check_motion=False):
        nonlocal status,status_host,stop_ack,wheel,zero_since,battery,imu_host
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            capture.check()
            if check_motion and parser.crc_errors: raise RuntimeError('host CRC error during motion')
            if check_motion and battery is not None and not 9000<=battery[6]<=16000: raise RuntimeError('VM outside test range')
            data=usb.read(8192)
            raw.extend(data)
            for f in parser.feed(data):
                frames.append({'host_monotonic':time.monotonic(),'type':f.message_type,
                               'sequence':f.sequence,'payload_hex':f.payload.hex()})
                if f.message_type==wire.MSG_BATTERY_STATE and len(f.payload)==28:
                    battery=struct.unpack('<IIIHHHHBBHHH',f.payload)
                imu=wire.decode_imu_state(f)
                if imu is not None:
                    imu_host=time.monotonic()
                    if not imu_rows or imu[:2]!=imu_rows[-1][:2]:imu_rows.append(imu)
                    if yaw_active:yaw.add(imu)
                s=wire.decode_status(f)
                if s is not None:
                    if check_motion and status is not None and ((s[0]-status[0])&0xffffffff)>10000: raise RuntimeError('MCU time reset/jump during motion')
                    if check_motion and any(s[i] for i in (17,18,19)): raise RuntimeError('device CRC/overflow/drop during motion')
                    status=s; status_host=time.monotonic()
                    if check_motion and (s[5] not in (1,2) or s[6] or s[8] or not (s[9]&0x10)):
                        raise RuntimeError('runtime/link fault while moving: '+str(s))
                w=wire.decode_wheel_state(f)
                if w is not None:
                    wheel=w
                    if any(w[5:]): zero_since=None
                    elif zero_since is None: zero_since=time.monotonic()
                if f.message_type==wire.MSG_ACK and len(f.payload)==4:
                    request,result,_,_=f.payload
                    if request==wire.CMD_STOP and result==0: stop_ack=True
                    if check_motion and request==wire.CMD_VELOCITY and result!=0:
                        raise RuntimeError('velocity rejected: '+f.payload.hex())
            if check_motion and time.monotonic()-status_host>.5:
                raise RuntimeError('STATUS stale for >500 ms')
            if check_motion and yaw is not None and time.monotonic()-imu_host>.2:
                raise RuntimeError('IMU reference stale for >200 ms')
    def stopped():
        return (status is not None and status[5]==1 and status[6]==0 and status[8]==0
                and not any(status[10:16]) and not(status[9]&0x20)
                and wheel is not None and not any(wheel[5:])
                and zero_since is not None and time.monotonic()-zero_since>=.3)
    def dump():
        if status is None or status[5] not in (1,3,4) or (status[9]&0x20):
            raise RuntimeError('trace request refused: controller not verified inactive')
        capture.port.write(b't')
        end=time.monotonic()+90; next_query=0
        while time.monotonic()<end:
            if time.monotonic()>=next_query:
                send(wire.CMD_GET_STATUS); next_query=time.monotonic()+.8
            pump(.05)
            log=(folder/'ch340_raw.bin').read_bytes()
            if b'TRACE END;' in log:
                decoded=parse(log.decode('utf-8',errors='strict'))
                expected_schema=2 if args.firmware=='0.18.23' else 3
                if decoded[-1]['header']['schema']!=expected_schema:
                    raise RuntimeError('trace schema differs from reported firmware')
                (folder/'trace_v2.json').write_text(json.dumps(decoded,indent=2)+'\n',encoding='utf-8')
                summary['trace_frames']=sum(len(c['frames']) for c in decoded)
                summary['trace_header']=decoded[-1]['header']
                print('TRACE complete:',summary['trace_header'],flush=True)
                return
        raise RuntimeError('TRACE END not received within 90 seconds')
    try:
        capture.start()
        usb=serial.Serial(port=None,baudrate=115200,timeout=.01,write_timeout=.2,dsrdtr=False,rtscts=False)
        usb.dtr=True; usb.rts=False; usb.port=args.port; usb.open()
        send(wire.CMD_GET_STATUS); pump(1.)
        summary['initial_battery']=battery
        if battery is None or not 9000 <= battery[6] <= 16000:
            raise RuntimeError('VM baseline outside 9..16 V test range: '+str(battery))
        if args.clear_known_poweroff_fault or args.clear_latched_fault_once:
            if not (status and status[5]==3 and status[6]==5 and not status[8]
                    and not any(status[10:16]) and not (status[9]&0x20)
                    and wheel and not any(wheel[5:]) and zero_since is not None
                    and time.monotonic()-zero_since>=.3):
                raise RuntimeError('zero-speed health-fault clear preconditions not met: '+str(status))
            summary['before_clear_status']=status
            summary['clear_reason']='known poweroff resolved' if args.clear_known_poweroff_fault else 'user authorized one repeat after latched fault'
            send(wire.CMD_CLEAR_FAULT); summary['fault_cleared']=True
            pump(.5); send(wire.CMD_GET_STATUS); pump(.3)
            print('VM in range; one authorized zero-speed clear sent. Verifying IDLE before motion.',flush=True)
        if not stopped(): raise RuntimeError('baseline not fault-free IDLE and stationary: '+str(status))
        if True: # User requested recording IMU reference, not an angle stop target.
            baseline_end=time.monotonic()+3.
            while time.monotonic()<baseline_end:
                send(wire.CMD_GET_STATUS);pump(.25)
                if not stopped():raise RuntimeError('chassis moved/state changed during stationary baseline')
            end_t=imu_rows[-1][0]
            baseline=[r[7] for r in imu_rows if r[0]>=end_t-3000 and r[11]==2 and r[12]&1]
            if len(baseline)<250 or statistics.pstdev(baseline)>3 or abs(statistics.mean(baseline))>100:
                raise RuntimeError('stationary IMU baseline insufficient/unstable')
            summary['reference_baseline_samples']=len(baseline)
            summary['reference_baseline_mrad_s']=statistics.mean(baseline)
            summary['reference_baseline_std_mrad_s']=statistics.pstdev(baseline)
            yaw=YawReference(statistics.mean(baseline));yaw.add(imu_rows[-1]);yaw_active=True
        summary['initial_status']=status; summary['initial_wheel']=wheel
        print('Baseline IDLE, stopped, fault=0/0. Starting ONE '+args.setup+
              ' '+args.direction+' pivot '+str(direction_sign*.40)+' rad/s, max '+str(args.seconds)+' s; IMU reference target='+str(args.target_imu_deg),flush=True)
        for countdown in (3,2,1):
            print('Starting in',countdown,flush=True); send(wire.CMD_GET_STATUS);pump(1.)
            if not stopped():raise RuntimeError('baseline changed during countdown')
        summary['initial_status']=status; summary['initial_wheel']=wheel
        yaw=YawReference(summary['reference_baseline_mrad_s']);yaw.add(imu_rows[-1]);yaw_active=True
        summary['reference_start_source_ms']=imu_rows[-1][0]
        start=time.monotonic(); summary['motion_attempts']=1
        try:
            next_send=start
            while time.monotonic()-start<args.seconds:
                if yaw is not None:
                    if direction_sign*yaw.radians < -math.pi/9:raise RuntimeError('IMU reference rotated opposite direction')
                    if args.target_imu_deg is not None and direction_sign*yaw.radians>=math.radians(args.target_imu_deg):
                        summary['imu_reference_target_reached']=True;break
                if time.monotonic()>=next_send:
                    send(wire.CMD_VELOCITY,velocity_payload); next_send=time.monotonic()+.05
                pump(.01,True)
            summary['motion_seconds']=time.monotonic()-start
        finally:
            summary.setdefault('motion_seconds',time.monotonic()-start)
            send(wire.CMD_STOP)
        pump(1.)
        send(wire.CMD_GET_STATUS); pump(.3)
        yaw_active=False
        summary['stop_ack']=stop_ack
        summary['final_stopped_verified']=stopped()
        if not stop_ack or not stopped(): raise RuntimeError('STOP not verified: '+str(status))
        print('Motion ended; STOP acknowledged, four wheels zero, controller inactive. Reading frozen trace.',flush=True)
        dump()
    except Exception as error:
        summary['error']=str(error)
        print('STOP/ERROR:',error,flush=True)
        if usb is not None and usb.is_open:
            try:
                send(wire.CMD_STOP); pump(1.); send(wire.CMD_GET_STATUS); pump(.3)
                yaw_active=False
                summary['final_stopped_verified']=bool(status and status[5] in (1,3,4)
                    and not(status[9]&0x20) and wheel and not any(wheel[5:]))
                if summary['motion_attempts']: dump()
            except Exception as nested: summary['trace_error']=str(nested)
    finally:
        summary['early_stop_checkpoint_verified']=summary['final_stopped_verified']
        summary['final_stopped_verified']=bool(status and status[5] in (1,3,4)
            and not(status[9]&0x20) and not any(status[10:16]) and wheel
            and not any(wheel[5:]) and zero_since is not None
            and time.monotonic()-zero_since>=.3)
        summary['final_status']=status; summary['final_wheel']=wheel
        summary['final_battery']=battery
        if yaw is not None:
            summary['imu_reference_final_deg']=math.degrees(yaw.radians)
            summary['reference_end_source_ms']=yaw.previous[0] if yaw.previous else None
            summary['imu_reference_count_gaps']=yaw.gaps
        if usb is not None: usb.close()
        capture.close()
        summary['capture_error']=capture.error
        summary['ports_closed']=True
        summary['host_crc_errors']=parser.crc_errors
        (folder/'usb_raw.bin').write_bytes(raw)
        (folder/'usb_frames.json').write_text(json.dumps(frames,indent=2)+'\n',encoding='utf-8')
        (folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(summary,indent=2),flush=True)
    return int(bool(summary['error'] or summary['trace_error'] or capture.error))

if __name__=='__main__': raise SystemExit(main())
