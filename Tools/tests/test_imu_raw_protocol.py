"""Offline reference codec tests. Does not import serial or open devices."""
import math
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import usb_cdc_test as wire


class ImuRawProtocolTest(unittest.TestCase):
    def values(self):
        return [0x12345678, 0xFEDCBA98, -32768, 32767, 8192,
                -57, 25, 151, -57, 25, 152, -10250, 123456, 2,
                655, 8192, 2, 7, 1, 0]

    def frame(self, values):
        return wire.Frame(wire.MSG_IMU_RAW_V1, 9, wire.IMU_RAW_V1_STRUCT.pack(*values))

    def test_signed_offsets_and_si(self):
        frame = self.frame(self.values())
        self.assertEqual(len(frame.payload), 52)
        self.assertEqual(frame.payload[8:12], b'\x00\x80\xff\x7f')
        self.assertEqual(struct.unpack_from('<hhh', frame.payload, 14), (-57,25,151))
        raw = wire.decode_imu_raw_v1(frame)
        self.assertEqual(raw, tuple(self.values()))
        si = wire.imu_raw_v1_to_si(raw)
        self.assertEqual(si['accel_m_s2'][2], 9.80665)
        self.assertAlmostEqual(si['angular_velocity_rad_s'][2], -math.pi/180/65.5)
        self.assertEqual(si['angular_velocity_rad_s'][:2], [0.0,0.0])
        self.assertEqual(si['temperature_c'], -10.25)
        self.assertEqual((si['state'],si['flags']), (2,7))

    def test_fragmentation_crc_sequence(self):
        frame = self.frame(self.values())
        encoded = wire.encode_frame(frame.message_type, 0xFFFF, frame.payload)
        self.assertEqual(len(encoded), 62)
        for split in range(len(encoded)+1):
            parser = wire.StreamParser()
            result = parser.feed(encoded[:split]) + parser.feed(encoded[split:])
            self.assertEqual(len(result),1)
            self.assertEqual(result[0].sequence,0xFFFF)
            self.assertEqual(wire.decode_imu_raw_v1(result[0]),tuple(self.values()))
        parser = wire.StreamParser()
        bad = bytearray(encoded); bad[15] ^= 1
        self.assertEqual(parser.feed(bytes(bad)),[])
        self.assertEqual(parser.crc_errors,1)

    def test_invalid_schema_scale_bias(self):
        for index,value in ((14,0),(15,0),(18,2),(19,1)):
            raw = self.values();raw[index]=value
            with self.assertRaises(ValueError): wire.decode_imu_raw_v1(self.frame(raw))
        raw = self.values();raw[17]=5
        with self.assertRaises(ValueError): wire.decode_imu_raw_v1(self.frame(raw))
        raw[8:11]=[0,0,0]
        self.assertEqual(wire.decode_imu_raw_v1(self.frame(raw)),tuple(raw))
        with self.assertRaises(ValueError):
            wire.decode_imu_raw_v1(wire.Frame(wire.MSG_IMU_RAW_V1,1,bytes(51)))
        self.assertIsNone(wire.decode_imu_raw_v1(wire.Frame(wire.MSG_IMU_STATE,1,bytes(46))))

    def test_legacy_and_selector_unchanged(self):
        self.assertEqual(wire.decode_imu_state(wire.Frame(wire.MSG_IMU_STATE,1,bytes(46))),
                         (0,)*13)
        for mode in (0,1):
            frame = wire.StreamParser().feed(wire.encode_frame(wire.CMD_SET_IMU_STREAM,1,bytes([mode])))[0]
            self.assertEqual((frame.message_type,frame.payload),(6,bytes([mode])))


if __name__ == '__main__':
    unittest.main()
