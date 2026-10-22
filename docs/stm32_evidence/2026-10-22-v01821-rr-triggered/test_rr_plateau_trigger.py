import json
from pathlib import Path
import unittest
from rr_plateau_trigger import RrPlateauTrigger

def w(t,count,cps=240):return [t,0,0,0,count,0,0,0,cps]
class Tests(unittest.TestCase):
    def test_actual_previous_gap(self):
        root=Path(__file__).resolve().parents[2]
        saved=json.loads((root/'docs/stm32_evidence/2026-10-22-v01821-ground-left-once/capture/decoded.json').read_text())
        detector=RrPlateauTrigger()
        for row in saved['wheel']:
            if row['phase']=='LEFT_motion':
                hit=detector.feed(row['values'],row['elapsed_s'])
                if hit:break
        self.assertEqual((hit['first_source_ms'],hit['last_source_ms'],hit['same_net_count']),(156803,156903,939))
        self.assertEqual(hit['span_ms'],100)
    def test_startup_no_prior_motion(self):
        d=RrPlateauTrigger()
        for t in range(0,6001,50):self.assertIsNone(d.feed(w(t,0,0),t/1000))
    def test_continuous(self):
        d=RrPlateauTrigger()
        for t in range(0,6001,50):self.assertIsNone(d.feed(w(t,t//5),t/1000))
    def test_short_gap_and_duplicate(self):
        d=RrPlateauTrigger()
        for t,c in ((1000,1),(1050,11),(1100,21),(1150,21),(1150,21),(1199,21),(1200,22)):
            self.assertIsNone(d.feed(w(t,c),t/1000))
    def test_wrap_threshold_and_latch(self):
        d=RrPlateauTrigger()
        for t,c in ((0xffffff00,1),(0xffffff32,11),(0xffffff64,21),(0xffffff96,21)):
            self.assertIsNone(d.feed(w(t,c),2))
        hit=d.feed(w(0xffffffc8,21),2.1)
        self.assertEqual(hit['span_ms'],100)
        self.assertIs(d.feed(w(0x2c,99),2.2),hit)
        d=RrPlateauTrigger()
        for t,c in ((0xffffffb4,1),(0xffffffe6,11),(0x18,21),(0x4a,21)):
            self.assertIsNone(d.feed(w(t,c),2))
        self.assertEqual(d.feed(w(0x7c,21),2)['span_ms'],100)
    def test_inactive_and_discontinuity(self):
        d=RrPlateauTrigger()
        for t in (0,1000,2000):self.assertIsNone(d.feed(w(t,0),t/1000,False))
        d.feed(w(3000,0),3)
        with self.assertRaises(ValueError):d.feed(w(2999,0),3)
        d=RrPlateauTrigger();d.feed(w(0,0),0)
        with self.assertRaises(ValueError):d.feed(w(201,0),.201)
if __name__=='__main__':unittest.main(verbosity=2)
