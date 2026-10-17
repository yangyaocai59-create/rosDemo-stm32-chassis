#!/usr/bin/env python3
"""Stationary RAW-interface acceptance. Sends only GET_STATUS and SET_IMU_STREAM.

No motion/STOP/fault-clear/calibration/Flash commands. Device must be flashed
by the operator first. A PASS is interface acceptance, NOT a yaw-drift fix.
"""
import argparse
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import time

import usb_cdc_test as wire
from debug_uart_capture import DebugUartCapture
from imu_stationary_diagnostic import analyze_uart, analyze_usb
from imu_quality_analysis import analyze_quality


class Session:
    def __init__(self, port, folder, uart):
        self.port, self.folder, self.uart = port, folder, uart
        self.parser = wire.StreamParser()
        self.sequence = 0
        self.next_query = time.monotonic()
        self.tx = []
        self.frames = []
        self.remaining_frames = []
        self.last_status = None
        self.last_status_host = self.last_wheel_host = None
        self.first_wheel = self.last_wheel = None
        self.status_count = self.wheel_frames = 0
        self.rx = (folder/'usb_rx.bin').open('wb')
        self.log = (folder/'frames.jsonl').open('w', encoding='utf8')

    def send(self, kind, payload=b''):
        if kind not in (wire.CMD_GET_STATUS, wire.CMD_SET_IMU_STREAM):
            raise ValueError('command not allowed in stationary RAW acceptance')
        self.sequence = (self.sequence+1)&0xFFFF
        encoded = wire.encode_frame(kind, self.sequence, payload)
        self.tx.append(dict(type=kind, sequence=self.sequence, payload=payload.hex()))
        self.log.write(json.dumps(dict(kind='tx',host_monotonic=time.monotonic(),hex=encoded.hex()))+'\n')
        if self.port.write(encoded) != len(encoded): raise RuntimeError('partial command write')
        return self.sequence

    def poll(self, keepalive=True):
        # wait() can match in the middle of one USB read. Those later frames
        # were already logged/validated, but still belong to the next consumer.
        if self.remaining_frames:
            frames,self.remaining_frames=self.remaining_frames,[]
            return frames
        if self.uart: self.uart.check()
        if keepalive and time.monotonic() >= self.next_query:
            self.send(wire.CMD_GET_STATUS)
            self.next_query = time.monotonic()+.8
        data = self.port.read(8192)
        if not data: return []
        self.rx.write(data)
        frames = self.parser.feed(data)
        if self.parser.crc_errors: raise RuntimeError('CRC/version error')
        for frame in frames:
            self.log.write(json.dumps(dict(kind='rx',host_monotonic=time.monotonic(),
                type=frame.message_type,sequence=frame.sequence,payload=frame.payload.hex()))+'\n')
            state = wire.decode_status(frame)
            if state:
                self.last_status_host = time.monotonic()
                self.last_status = state; self.status_count += 1
                if state[5]!=wire.STATE_IDLE or state[6] or state[8] or any(state[10:12]):
                    raise RuntimeError('not fault-free IDLE with zero motion request')
            wheel = wire.decode_wheel_state(frame)
            if wheel:
                self.last_wheel_host = time.monotonic()
                self.wheel_frames += 1
                if self.first_wheel is None: self.first_wheel = wheel
                self.last_wheel = wheel
                if any(wheel[5:9]): raise RuntimeError('nonzero wheel speed; stationary test stopped')
                if wheel[1:5] != self.first_wheel[1:5]:
                    raise RuntimeError('wheel count changed; stationary test stopped')
            self.frames.append(frame)
        return frames

    def wait(self, predicate, timeout=3):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            frames=self.poll()
            for index,frame in enumerate(frames):
                if predicate(frame):
                    self.remaining_frames=frames[index+1:]
                    return frame
        raise RuntimeError('expected ACK/telemetry timeout')

    def command(self, payload, result=wire.RESULT_OK):
        seq = self.send(wire.CMD_SET_IMU_STREAM,payload)
        ack = self.wait(lambda f:f.message_type==wire.MSG_ACK and f.sequence==seq)
        if len(ack.payload)!=4 or tuple(ack.payload[:2])!=(wire.CMD_SET_IMU_STREAM,result):
            raise RuntimeError('unexpected SET_IMU_STREAM ACK: '+ack.payload.hex())

    def legacy(self):
        # Allow already in-flight RAW frames during a mode switch to drain.
        self.send(wire.CMD_GET_STATUS)
        self.wait(lambda f:wire.decode_imu_state(f) is not None)

    def close(self):
        self.rx.close(); self.log.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--debug-port')
    ap.add_argument('--duration', type=float, default=120)
    ap.add_argument('--vm-off-confirmed',action='store_true')
    ap.add_argument('--stationary-confirmed',action='store_true')
    ap.add_argument('--require-bias',action='store_true')
    ap.add_argument('--raw-version', type=int, choices=(1,2), default=2,
                    help='default 2: exact milli-RAW bias, requires firmware >=0.16; 1 is coarse compatibility')
    ap.add_argument('--require-quality',action='store_true',
                    help='0.14.4: require CH340 quality records and continuous source counts/time')
    args = ap.parse_args()
    decode_raw = wire.decode_imu_raw_v2 if args.raw_version == 2 else wire.decode_imu_raw_v1
    raw_to_si = wire.imu_raw_v2_to_si if args.raw_version == 2 else wire.imu_raw_v1_to_si
    selector = bytes([args.raw_version])
    if args.require_quality and not args.debug_port:
        ap.error('--require-quality requires --debug-port')
    if not args.vm_off_confirmed or not args.stationary_confirmed or not 10<=args.duration<=600:
        ap.error('confirm VM off/stationary; duration must be 10..600 s')
    import serial
    from serial.tools import list_ports
    devices={p.device.casefold():p for p in list_ports.comports()}
    dev=devices.get(args.port.casefold())
    if dev is None or (dev.vid,dev.pid)!=(0x0483,0x5740): ap.error('not STM32 CDC 0483:5740')
    if args.debug_port:
        dbg=devices.get(args.debug_port.casefold())
        if dbg is None or (dbg.vid,dbg.pid)!=(0x1A86,0x7523):ap.error('not CH340 1A86:7523')
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    folder=Path(__file__).resolve().parents[1]/'build'/'hardware_tests'/('imu_raw_'+stamp)
    folder.mkdir(parents=True,exist_ok=False)
    summary=dict(result='incomplete',firmware_flashing=False,motion_commands=0,
                 calibration_commands=0,port=dev.device,serial=dev.serial_number,
                 raw_format_version=args.raw_version,
                 note='interface acceptance only; no ROS residual compensation',errors=[])
    uart=DebugUartCapture(args.debug_port,folder) if args.debug_port else None
    port=None; session=None; samples=[]; si_samples=[]
    print('Evidence: '+str(folder),flush=True)
    try:
        if uart: uart.start()
        port=serial.Serial(port=None,baudrate=115200,timeout=.01,write_timeout=.2)
        port.dtr=True;port.rts=False;port.port=args.port;port.open()
        session=Session(port,folder,uart)
        session.legacy()
        if session.last_status is None or session.first_wheel is None:
            session.wait(lambda f:wire.decode_wheel_state(f) is not None and session.last_status is not None)
        session.command(b'',1)
        session.command(b'\x00\x01',1)
        session.command(b'\x03',2)
        session.legacy()
        summary['legacy_and_malformed_selectors']='PASS'
        session.command(selector)
        first=session.wait(lambda f:decode_raw(f) is not None)
        start=time.monotonic(); last_sample=None; next_progress=30
        last_imu_host=start
        print('RAW capture started; remain stationary.',flush=True)
        batch=[first]
        while time.monotonic()-start < args.duration:
            now=time.monotonic()
            if (session.last_status_host is None or now-session.last_status_host>.5 or
                session.last_wheel_host is None or now-session.last_wheel_host>.3 or now-last_imu_host>1):
                raise RuntimeError('STATUS/WHEEL/IMU data stale during RAW capture')
            for frame in batch:
                raw=decode_raw(frame)
                if raw is None: continue
                last_imu_host=time.monotonic()
                if raw[16]!=2 or raw[17]&5!=5 or raw[17]&8 or raw[13]:
                    raise RuntimeError('RAW sample invalid, calibrating or has consecutive errors')
                if args.require_bias and not raw[17]&2:raise RuntimeError('baseline bias not restored')
                if last_sample is not None:
                    if raw[1]==last_sample[1]:continue
                    if not 0<((raw[0]-last_sample[0])&0xFFFFFFFF)<=1000:
                        raise RuntimeError('RAW timestamp reset/stale/discontinuous')
                    if args.require_quality and (
                        ((raw[1]-last_sample[1])&0xFFFFFFFF)!=1 or
                        not 5<=((raw[0]-last_sample[0])&0xFFFFFFFF)<=15):
                        raise RuntimeError('RAW source count/time gap during quality acceptance')
                si=raw_to_si(raw)
                # Feed the common time-weighted integrator with unrounded SI.
                si_samples.append([raw[0],raw[1],*[v*1000 for v in si['accel_m_s2']],
                    *[v*1000 for v in si['angular_velocity_rad_s']],raw[11],raw[12],raw[13],raw[16],raw[17]])
                samples.append(raw);last_sample=raw
            if time.monotonic()-start>=next_progress:
                print(f'{next_progress}s unique RAW={len(samples)}',flush=True);next_progress+=30
            batch=session.poll()
        if len(samples)<args.duration*80:raise RuntimeError('RAW unique sample rate too low')
        summary['raw']=analyze_usb(si_samples)
        if not 95<=summary['raw']['source_rate_hz']<=105:raise RuntimeError('unexpected MCU RAW rate')
        if len({(r[14],r[15]) for r in samples})!=1:raise RuntimeError('scale changed during capture')
        if len({tuple(r[8:11]) for r in samples})!=1:raise RuntimeError('baseline changed during capture')
        if len({r[12] for r in samples})!=1:raise RuntimeError('IMU read errors increased during capture')
        summary['baseline']=list(samples[-1][8:11]);summary['flags']=sorted({r[17] for r in samples})
        summary['baseline_units']='milli-RAW' if args.raw_version == 2 else 'integer-RAW coarse view'
        from imu_raw_drift_report import analyze as analyze_raw_drift
        summary['raw_drift']=analyze_raw_drift(samples) if args.require_bias else None
        session.command(b'\x00');session.legacy()
        session.command(selector)
        session.wait(lambda f:decode_raw(f) is not None)
        # CDC DTR only: never touch CH340 modem lines or issue a reset pulse.
        # There is no motion request in this test and VM is operator-confirmed off.
        port.dtr=False
        closed_until=time.monotonic()+.25
        while time.monotonic()<closed_until:session.poll(keepalive=False)
        port.dtr=True
        session.legacy()
        summary['dtr_reset_to_legacy']='PASS'
        session.command(selector)
        session.wait(lambda f:decode_raw(f) is not None)
        # Expire the telemetry session without changing DTR or issuing STOP.
        expiry=time.monotonic()+1.6
        while time.monotonic()<expiry: session.poll(keepalive=False)
        session.legacy()  # Production firmware must reset selection on expiry.
        summary['switch_back_and_session_expiry']='PASS'
        summary['status_frames']=session.status_count;summary['wheel_frames']=session.wheel_frames
        summary['result']='PASS_INTERFACE_ONLY'
    except (Exception,KeyboardInterrupt) as error:
        summary['errors'].append(str(error) or 'interrupted')
    finally:
        if session:
            summary['tx']=session.tx;summary['crc_errors']=session.parser.crc_errors
            session.close()
        if port:port.close()
        if uart:
            uart.close()
            if uart.error:summary['errors'].append('UART: '+uart.error)
        (folder/'raw_samples.jsonl').write_text(''.join(json.dumps(dict(raw=list(r)))+'\n' for r in samples),encoding='utf8')
        if uart and (folder/'ch340.log').exists() and samples:
            text=(folder/'ch340.log').read_text(encoding='utf8')
            summary['uart'],pairs=analyze_uart(text,samples[0][1],samples[-1][1])
            summary['imu_quality']=analyze_quality(text,samples[0][0],samples[-1][0])
            if args.require_quality and summary['imu_quality']['result']!='PASS_OBSERVED_QUALITY':
                summary['errors'].append('IMU quality acceptance failed or records missing')
            by_count={r[1]:r for r in samples};match=bad=0
            for pair in pairs:
                raw=by_count.get(pair['sample_count'])
                if raw is None:continue
                match+=1
                if pair['gyro']!=list(raw[5:8]) or pair['accel']!=list(raw[2:5]):bad+=1
            summary['uart_raw_exact_matches']=match;summary['uart_raw_mismatches']=bad
            if not match or bad:summary['errors'].append('UART/USB raw sample match failed')
        elif args.require_quality:
            summary['errors'].append('no UART/RAW records for quality acceptance')
        if summary['errors']:summary['result']='INCOMPLETE_OR_FAILED'
        (folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
        print(json.dumps(summary,indent=2),flush=True)
    return 0 if not summary['errors'] and summary['result']=='PASS_INTERFACE_ONLY' else 1


if __name__=='__main__':raise SystemExit(main())
