"""One existing IMU calibration command; GET_STATUS only otherwise, no motion."""
import sys, json, time, struct, statistics, re, argparse
from pathlib import Path
from datetime import datetime
import serial
from serial.tools import list_ports
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--execute',action='store_true')
ap.add_argument('--firmware',required=True)
args=ap.parse_args()
if not args.execute:ap.error('--execute required; one persistent calibration, no motion')
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'Tools'))
import usb_cdc_test as w
from debug_uart_capture import DebugUartCapture

out=root/'build/hardware_tests'/(('v'+args.firmware.replace('.','')+'_calibrate_once_')+datetime.now().strftime('%Y%m%d_%H%M%S'))
out.mkdir(parents=True,exist_ok=False)
frames=[];raw=bytearray();tx=[];parser=w.StreamParser();seq=19000;usb=None;capture=None
statuses=[];wheels=[];imus=[];last_status=last_imu=0.;last_query=0.;first_counts=None
result={'folder':str(out),'motion_sent':False,'clear_sent':False,'calibration_commands':0,'firmware_user_report':args.firmware,'error':None}
def send(kind):
 global seq
 seq+=1;usb.write(w.encode_frame(kind,seq));tx.append({'type':kind,'seq':seq,'host_monotonic':time.monotonic()});return seq
def pump(seconds):
 global last_query,last_status,last_imu,first_counts
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  now=time.monotonic()
  if now-last_query>.1:send(w.CMD_GET_STATUS);last_query=now
  data=usb.read(8192);raw.extend(data)
  for f in parser.feed(data):
   frames.append({'type':f.message_type,'sequence':f.sequence,'payload_hex':f.payload.hex(),'host_monotonic':time.monotonic()})
   s=w.decode_status(f)
   if s is not None:
    assert s[5]==1 and not s[6] and not s[8] and not any(s[10:16]) and not s[9]&0x20,'Not idle/stopped/healthy'
    assert not statuses or s[0]>=statuses[-1][0],'MCU reset during capture'
    assert not any(s[17:20]),'Device CRC/overflow/drop'
    statuses.append(s);last_status=time.monotonic()
   wheel=w.decode_wheel_state(f)
   if wheel is not None:
    assert not any(wheel[5:]),'Wheel moved'
    if first_counts is None:first_counts=wheel[1:5]
    assert wheel[1:5]==first_counts,'Encoder counts changed while stationary'
    wheels.append(wheel)
   imu=w.decode_imu_state(f)
   if imu is not None:
    assert imu[11]==2 and imu[12]&1 and not imu[10],'IMU not healthy READY'
    if not imus or imu[:2]!=imus[-1][:2]:imus.append(imu)
    last_imu=time.monotonic()
  assert parser.crc_errors==0,'Host CRC errors'
  if statuses:assert time.monotonic()-last_status<.6,'STATUS stale'
  if imus:assert time.monotonic()-last_imu<.3,'IMU stale'
  if capture is not None:capture.check()

try:
 ports={p.device:p for p in list_ports.comports()}
 assert (ports['COM4'].vid,ports['COM4'].pid)==(0x0483,0x5740)
 assert (ports['COM8'].vid,ports['COM8'].pid)==(0x1a86,0x7523)
 usb=serial.Serial(port=None,baudrate=115200,timeout=.02,write_timeout=.2,dsrdtr=False,rtscts=False)
 usb.dtr=True;usb.rts=False;usb.port='COM4';usb.open()
 pump(3)
 assert len(statuses)>3 and len(imus)>200 and wheels,'Missing static preflight telemetry'
 result['pre_debug_status']=statuses[-1]
 capture=DebugUartCapture('COM8',out);capture.start();pump(3)
 baseline=imus[-250:];result['before_gyro_mean_mrad_s']=[statistics.mean(x[i] for x in baseline) for i in (5,6,7)]
 result['before_gyro_std_mrad_s']=[statistics.pstdev(x[i] for x in baseline) for i in (5,6,7)]
 assert max(result['before_gyro_std_mrad_s'])<3,'Not sufficiently stationary'
 assert not imus[-1][12]&8,'Calibration already active'
 result['calibration_sequence']=send(w.CMD_CALIBRATE_IMU);result['calibration_commands']=1
 print('One CALIBRATE_IMU sent; no velocity or fault-clear commands. '+str(out),flush=True)
 pump(13)
 acks=[f for f in frames if f['type']==w.MSG_ACK and f['sequence']==result['calibration_sequence']]
 assert len(acks)==1 and bytes.fromhex(acks[0]['payload_hex'])==bytes((5,0,1,0)),'Calibration ACK not OK/IDLE'
 assert imus[-1][12]&0x3f==0x37,'Calibration not completed successfully'
 pump(5)
 post=imus[-500:];result['after_gyro_mean_mrad_s']=[statistics.mean(x[i] for x in post) for i in (5,6,7)]
 result['after_gyro_std_mrad_s']=[statistics.pstdev(x[i] for x in post) for i in (5,6,7)]
 assert max(abs(x) for x in result['after_gyro_mean_mrad_s'])<5,'Post calibration residual exceeds 5 mrad/s'
except Exception as e:result['error']=repr(e)
finally:
 if usb is not None:usb.close()
 if capture is not None:capture.close();result['debug_error']=capture.error
 text=(out/'ch340.log').read_text(encoding='utf-8') if (out/'ch340.log').exists() else ''
 result['flash_saved_lines']=[s for s in text.splitlines() if 'IMU bias Flash saved:' in s]
 result['persisted_logged']='gyro bias applied and persisted for future boots' in text
 result['precise_bias_milli_raw']=re.findall(r'IMU STATIC precise bias milli_raw=\[([^]]+)\]',text)
 result['calibration_pass_count']=text.count('IMU STATIC result=PASS_BIAS_APPLIED')
 result['host_crc_errors']=parser.crc_errors
 result['status_first_last']=statuses[::max(1,len(statuses)-1)]
 result['wheel_first_last']=wheels[::max(1,len(wheels)-1)]
 result['imu_last']=imus[-1:] if imus else []
 result['pass']=result['error'] is None and not result.get('debug_error') and result['persisted_logged'] and len(result['flash_saved_lines'])==1 and result['calibration_pass_count']==1
 (out/'usb_raw.bin').write_bytes(raw)
 (out/'usb_frames.json').write_text(json.dumps(frames,indent=2),encoding='utf-8')
 (out/'tx.json').write_text(json.dumps(tx,indent=2),encoding='utf-8')
 (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
 print(json.dumps(result,indent=2),flush=True)
sys.exit(0 if result['pass'] else 1)
