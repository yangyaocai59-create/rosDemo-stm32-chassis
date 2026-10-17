"""Offline regression for mixed ACK + multiple RAW frames in a single read."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import usb_cdc_test as wire
from imu_raw_stream_test import Session


class FakePort:
    def __init__(self,data): self.data=data
    def read(self,size):
        data,self.data=self.data,b''
        return data


class MixedReadTest(unittest.TestCase):
    def test_ack_and_raw_suffix_preserved_exactly_once(self):
        ack=wire.encode_frame(wire.MSG_ACK,8,b'\x06\x00\x01\x00')
        raw=[]
        for count in (100,101,102):
            payload=wire.IMU_RAW_V1_STRUCT.pack(count*10,count,0,0,8192,
                -55,26,151,-55,26,150,29110,0,0,655,8192,2,7,1,0)
            raw.append(wire.encode_frame(wire.MSG_IMU_RAW_V1,count,payload))
        with tempfile.TemporaryDirectory(prefix='imu_raw_capture_test_') as folder:
            session=Session(FakePort(ack+b''.join(raw)),Path(folder),None)
            session.next_query=float('inf')
            try:
                frame=session.wait(lambda f:f.message_type==wire.MSG_ACK)
                self.assertEqual(frame.sequence,8)
                first=session.wait(lambda f:wire.decode_imu_raw_v1(f) is not None)
                tail=session.poll()
                counts=[wire.decode_imu_raw_v1(f)[1] for f in [first,*tail]]
                self.assertEqual(counts,[100,101,102])
                self.assertEqual(len(session.frames),4)  # not logged twice
                self.assertEqual(session.poll(),[])
            finally:session.close()


if __name__=='__main__':unittest.main()
