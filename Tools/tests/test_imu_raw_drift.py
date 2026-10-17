"""Offline RAW drift arithmetic, duplicate handling and capture validity."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_raw_drift_report import analyze


def row(t, count, z=145):
    return [t, count, 0, 0, 8192, -52, 19, z, -52, 19, 146,
            27200, 0, 0, 655, 8192, 2, 7, 1, 0]


class RawDriftTest(unittest.TestCase):
    def test_exact_conversion_and_duplicates(self):
        a, b = row(100, 1), row(1100, 101)
        r = analyze([a, {'raw': a}, b])
        self.assertAlmostEqual(r['equivalent_yaw_deg_min'], -60 / 65.5)
        self.assertAlmostEqual(r['observed_integral_deg'], -1 / 65.5)
        self.assertEqual(r['duplicate_frames_excluded'], 1)
        self.assertFalse(r['strict_source_continuity'])

    def test_fractional_mean_not_rounded_si(self):
        r = analyze([row(100, 1, 145), row(110, 2, 146)])
        self.assertAlmostEqual(r['equivalent_yaw_deg_min'], -30 / 65.5)
        self.assertTrue(r['strict_source_continuity'])

    def test_counter_and_timestamp_wrap(self):
        r = analyze([row(0xFFFFFFFA, 0xFFFFFFFF), row(4, 0)])
        self.assertAlmostEqual(r['duration_s'], 0.01)
        self.assertTrue(r['strict_source_continuity'])

    def test_reject_changed_baseline_or_error(self):
        for index in (10, 12, 13, 14):
            b = row(110, 2)
            b[index] += 1
            with self.assertRaises(ValueError):
                analyze([row(100, 1), b])

    def test_reject_reset_stale_inconsistent_invalid(self):
        bad = [row(90, 2), row(110, 0), row(110, 1)]
        invalid = row(110, 2)
        invalid[17] = 5  # no valid bias
        bad.append(invalid)
        for b in bad:
            with self.assertRaises(ValueError):
                analyze([row(100, 1), b])


if __name__ == '__main__':
    unittest.main()
