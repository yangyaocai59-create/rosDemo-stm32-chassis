"""Quality analysis accepts stable counts, rejects silent recovery/loss."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_quality_analysis import analyze_quality


def line(t, epoch=1, drdy=100, missed=2, race=0, timeout=0, persist=2):
    return (f'[{t:010d}] [INFO ] IMU QUALITY: epoch={epoch} drdy={drdy} '
            f'missed={missed} race={race} timeout={timeout} cal=2 persist={persist}\r\n')


class QualityTest(unittest.TestCase):
    def test_stable_window_not_absolute_zero_or_flash_claim(self):
        result = analyze_quality(line(1000)+line(2000,drdy=200),900,2100)
        self.assertEqual(result['result'],'PASS_OBSERVED_QUALITY')
        self.assertEqual(result['deltas']['missed'],0)
        self.assertEqual(result['first']['missed'],2)
        result = analyze_quality(line(1000,persist=3)+line(2000,drdy=200,persist=3),900,2100)
        self.assertEqual(result['last']['persist'],3)  # Explicitly RAM-only.

    def test_failures_and_missing(self):
        for mutation in ({'epoch':2},{'missed':3},{'race':1},{'timeout':1}):
            self.assertEqual(analyze_quality(line(1000)+line(2000,drdy=200,**mutation),900,2100)
                             ['result'],'FAIL_QUALITY_CHANGED')
        self.assertEqual(analyze_quality('',0,10000)['result'],'INSUFFICIENT_QUALITY_RECORDS')

    def test_tick_and_counter_wrap_and_window(self):
        text=line(0xFFFFFFF8,drdy=0xFFFFFFF0)+line(10,drdy=10)+line(100,epoch=2)
        result=analyze_quality(text,0xFFFFFFF0,20)
        self.assertEqual(result['result'],'PASS_OBSERVED_QUALITY')
        self.assertEqual(result['records'],2)
        self.assertEqual(result['deltas']['drdy'],26)


if __name__=='__main__':unittest.main()
