"""Exact fixed-point baseline codec and offline drift regression; no ports."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import usb_cdc_test as wire
from imu_raw_drift_report import analyze
from imu_stationary_diagnostic import analyze_uart


def values(count=1, t=100):
    return [t, count, -32768, 32767, 8192, -57, 25, 146,
            -57125, 25625, 145500, 27200, 0, 0, 655, 8192, 2, 7, 2, 0]


class RawV2Test(unittest.TestCase):
    def frame(self, row):
        return wire.Frame(wire.MSG_IMU_RAW_V2, 0xFFFF, wire.IMU_RAW_V2_STRUCT.pack(*row))

    def test_exact_signed_baseline_and_si(self):
        frame = self.frame(values())
        row = wire.decode_imu_raw_v2(frame)
        self.assertEqual(len(frame.payload), 52)
        self.assertEqual(row[8:11], (-57125, 25625, 145500))
        si = wire.imu_raw_v2_to_si(row)
        for axis, residual in enumerate((0.125, -0.625, 0.5)):
            self.assertAlmostEqual(si['angular_velocity_rad_s'][axis], residual / 65.5 * math.pi / 180)
        self.assertIsNone(wire.decode_imu_raw_v1(frame))

    def test_schema_separation_and_range(self):
        for index, value in ((18, 1), (19, 1), (14, 0), (15, 0),
                             (8, 1000001), (10, -1000001), (17, 5)):
            row = values()
            row[index] = value
            with self.assertRaises(ValueError):
                wire.decode_imu_raw_v2(self.frame(row))
        row = values()
        row[8:11] = [0, 0, 0]
        row[17] = 5
        self.assertEqual(wire.decode_imu_raw_v2(self.frame(row)), tuple(row))

    def test_all_splits_crc(self):
        frame = self.frame(values())
        encoded = wire.encode_frame(frame.message_type, frame.sequence, frame.payload)
        for split in range(len(encoded) + 1):
            parser = wire.StreamParser()
            frames = parser.feed(encoded[:split]) + parser.feed(encoded[split:])
            self.assertEqual(len(frames), 1)
            self.assertEqual(wire.decode_imu_raw_v2(frames[0]), tuple(values()))
        bad = bytearray(encoded)
        bad[30] ^= 1
        parser = wire.StreamParser()
        self.assertEqual(parser.feed(bad), [])
        self.assertEqual(parser.crc_errors, 1)

    def test_v2_drift_uses_milli_baseline(self):
        a, b, c = values(1, 100), values(2, 110), values(3, 120)
        a[7], b[7], c[7] = 145, 146, 145
        r = analyze([a, b, c])
        self.assertAlmostEqual(r['observed_integral_deg'], 0)
        self.assertAlmostEqual(r['equivalent_yaw_deg_min'], 0)
        self.assertEqual(r['baseline_raw'], [-57.125, 25.625, 145.5])
        self.assertEqual(r['raw_format_version'], 2)

    def test_mixed_units_rejected(self):
        a, b = values(), values(2, 110)
        b[18] = 1
        with self.assertRaises(ValueError):
            analyze([a, b])

    def test_uart_precise_restore_not_integer_inference(self):
        log = ('IMU gyro bias restored from Flash: milli_raw=[-57125,25625,145500] '
               'calibration_temp=27.20C\n'
               '[0000000100] [INFO ] IMU RAW: a=[0,0,8192] temp=0 g=[-57,25,146] samples=1 errors=0\n'
               '[0000000100] [INFO ] IMU SI: a_mmps2=[0,0,9807] w_mrads=[0,0,0] temp_mC=27200 bias=1\n')
        report, _ = analyze_uart(log)
        self.assertEqual(report['bias_milli_raw'], [-57125, 25625, 145500])
        self.assertEqual(report['bias_raw'], [-57.125, 25.625, 145.5])
        self.assertEqual(report['raw_si_arithmetic_mismatches'], 0)


if __name__ == '__main__':
    unittest.main()
