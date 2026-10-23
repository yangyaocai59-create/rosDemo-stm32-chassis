"""GET_STATUS-only preflight; debug UART read-only, opened only after IDLE."""
import sys,json,time,struct,serial
from pathlib import Path
from datetime import datetime,timezone,timedelta
root=Path.cwd();sys.path.insert(0,str(root/'Tools'))
import usb_cdc_test as w
from debug_uart_capture import DebugUartCapture
out=root/'build/hardware_tests'/('v0190_readonly_'+datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
frames=[];raw=bytearray();parser=w.StreamParser();seq=17000;capture=None;usb=None
result={'folder':str(out),'motion_sent':False,'clear_sent':False,'calibration_sent':False}
try:
 usb=serial.Serial(port=None,baudrate=115200,timeout=.02,write_timeout=.2,dsrdtr=False,rtscts=False);usb.dtr=True;usb.rts=False;usb.port='COM4';usb.open()
 def read(seconds):
  global seq
  end=time.monotonic()+seconds;next_query=0
  while time.monotonic()<end:
   if time.monotonic()>=next_query:
    seq+=1;usb.write(w.encode_frame(w.CMD_GET_STATUS,seq));next_query=time.monotonic()+.8
   data=usb.read(8192);raw.extend(data)
   for f in parser.feed(data):frames.append({'type':f.message_type,'seq':f.sequence,'payload_hex':f.payload.hex(),'host_monotonic_s':time.monotonic()})
 read(3)
 statuses=[struct.unpack('<IIIIHBBBBHhhhhhhIIII',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==w.MSG_STATUS]
 assert statuses and all(s[5]==1 and s[6]==0 and s[8]==0 and not any(s[10:16]) for s in statuses),'Pre-UART board not confirmed idle/zero/no-fault; no debug open'
 result['pre_debug_status']=statuses[-1]
 capture=DebugUartCapture('COM8',out);capture.start();read(10)
except Exception as e:result['error']=repr(e)
finally:
 if usb is not None:usb.close()
 if capture is not None:capture.close();result['debug_error']=capture.error
 (out/'usb_raw.bin').write_bytes(raw)
 (out/'usb_frames.json').write_text(json.dumps(frames,indent=2))
 result['frames']=len(frames)
 statuses=[struct.unpack('<IIIIHBBBBHhhhhhhIIII',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==w.MSG_STATUS]
 wheels=[struct.unpack('<Iqqqqiiii',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==w.MSG_WHEEL_STATE]
 battery=[struct.unpack('<IIIHHHHBBHHH',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==w.MSG_BATTERY_STATE]
 imu=[struct.unpack('<IIiiiiiiiIIBB',bytes.fromhex(f['payload_hex'])) for f in frames if f['type']==w.MSG_IMU_STATE]
 result.update(status_first_last=statuses[::max(1,len(statuses)-1)],wheel_first_last=wheels[::max(1,len(wheels)-1)],battery_supply_range_mV=[min(x[6] for x in battery),max(x[6] for x in battery)] if battery else [],imu_last=imu[-1:] )
 (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
