"""One zero-speed health-fault clear after operator reports obstruction removed.

No velocity command, reset, reconnect retry, or firmware operation is issued.
"""
from datetime import datetime
import json
from pathlib import Path
import struct
import time
import serial
from debug_uart_capture import DebugUartCapture
import usb_cdc_test as wire


def main():
    folder=Path(__file__).resolve().parents[1]/'build/hardware_tests'/('resolved_wire_clear_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True,exist_ok=False)
    capture=DebugUartCapture('COM8',folder);usb=None;parser=wire.StreamParser();sequence=0
    status=None;battery=None;wheels=[];frames=[];raw=bytearray();ack=False
    result=dict(reason='user reported wire blocking wheel; user now says continue',motion_sent=False,
                clear_sent=False,error=None,folder=str(folder))
    def send(kind):
        nonlocal sequence
        sequence=(sequence+1)&65535;usb.write(wire.encode_frame(kind,sequence))
    def observe(seconds):
        nonlocal status,battery,ack
        end=time.monotonic()+seconds;next_query=0
        while time.monotonic()<end:
            capture.check()
            if time.monotonic()>=next_query:
                send(wire.CMD_GET_STATUS);next_query=time.monotonic()+.1
            data=usb.read(8192);raw.extend(data)
            for f in parser.feed(data):
                frames.append(dict(type=f.message_type,sequence=f.sequence,payload_hex=f.payload.hex()))
                s=wire.decode_status(f)
                if s is not None:status=s
                w=wire.decode_wheel_state(f)
                if w is not None:wheels.append(w)
                if f.message_type==wire.MSG_BATTERY_STATE:battery=struct.unpack('<IIIHHHHBBHHH',f.payload)
                if f.message_type==wire.MSG_ACK and f.payload[0]==wire.CMD_CLEAR_FAULT:
                    if f.payload[1]!=0:raise RuntimeError('fault clear rejected: '+str(tuple(f.payload)))
                    ack=True
            if parser.crc_errors:raise RuntimeError('host CRC error')
    try:
        capture.start()
        usb=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.2,dsrdtr=False,rtscts=False)
        usb.dtr=True;usb.rts=False;usb.port='COM4';usb.open()
        observe(1.3)
        if not (status and status[5] in (1,3) and status[8]==0 and not status[9]&0x20
                and not any(status[10:16]) and battery and 9000<=battery[6]<=16000
                and battery[2]==0 and battery[9]==0 and len(wheels)>=10
                and all(not any(w[5:]) for w in wheels[-10:])
                and all(max(w[i] for w in wheels[-10:])-min(w[i] for w in wheels[-10:])<=2 for i in range(1,5))):
            raise RuntimeError('zero-speed/output/supply preconditions not met: '+str(status))
        result['before_status']=status;result['before_wheel']=wheels[-1];result['battery']=battery
        if status[5]==3:
            if status[6]!=5:raise RuntimeError('not the expected health-manager latch')
            send(wire.CMD_CLEAR_FAULT);result['clear_sent']=True
        observe(1.)
        if not (status[5]==1 and status[6]==0 and status[8]==0 and not status[9]&0xa0
                and not any(status[10:16]) and not any(wheels[-1][5:])
                and (ack or not result['clear_sent'])):
            raise RuntimeError('healthy IDLE not verified after clear: '+str(status))
        result['healthy_idle_verified']=True
    except Exception as e:result['error']=str(e)
    finally:
        if usb:usb.close()
        capture.close();result['capture_error']=capture.error;result['final_status']=status
        result['final_wheel']=wheels[-1] if wheels else None;result['ports_closed']=True
        (folder/'usb_raw.bin').write_bytes(raw)
        (folder/'usb_frames.json').write_text(json.dumps(frames,indent=2)+'\n',encoding='utf-8')
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(result,indent=2))
    return int(bool(result['error'] or capture.error))


if __name__=='__main__':raise SystemExit(main())
