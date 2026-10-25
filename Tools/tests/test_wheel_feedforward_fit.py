"""Synthetic identification checks; no hardware accuracy claim."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

SPEC=importlib.util.spec_from_file_location('fit',Path(__file__).resolve().parents[1]/'fit_wheel_feedforward.py')
FIT=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FIT)


class IdentificationTests(unittest.TestCase):
    def test_known_bidirectional_voltage_model(self):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'steady.csv'
            with path.open('w',newline='') as stream:
                writer=csv.writer(stream)
                writer.writerow(('wheel','target_cps','measured_cps','pwm_permille','supply_mv','steady','valid'))
                for w,wheel in enumerate(FIT.WHEELS):
                    for sign in (-1,1):
                        ks=.45+w*.02+(sign<0)*.05
                        kv=.0015+w*.00003
                        for speed in (100,200,400,800):
                            for k in range(20):
                                supply=10.5+k*.05
                                volts=ks+kv*speed
                                writer.writerow((wheel,sign*speed,sign*speed,sign*volts/supply*1000,supply*1000,1,1))
                writer.writerow(('LF',200,200,300,11000,1,1))
                writer.writerow(('RF',200,200,100,11000,0,1))
            report=FIT.fit_csv(path,'synthetic')
            self.assertFalse(report['errors'])
            self.assertEqual(report['rejected']['zero_or_pwm_saturated'],1)
            self.assertEqual(report['rejected']['not_valid_steady'],1)
            for w,wheel in enumerate(FIT.WHEELS):
                for direction in ('forward','reverse'):
                    fit=report['fits'][wheel][direction]
                    self.assertAlmostEqual(fit['ks_volts'],.45+w*.02+(direction=='reverse')*.05,places=9)
                    self.assertAlmostEqual(fit['kv_volts_per_cps'],.0015+w*.00003,places=9)
            self.assertFalse(report['firmware_parameters_changed'])

    def test_insufficient_or_nonphysical_data_rejected(self):
        for points in ([],[(100,.5)]*40,[(s,3-.001*s) for s in range(100,900,10)]):
            with self.assertRaises(ValueError): FIT.linear_fit(points)

    def test_robust_residuals(self):
        points=[(s,.5+.0015*s) for s in range(100,900,10)]
        points[10]=(points[10][0],4)
        result=FIT.linear_fit(points)
        self.assertAlmostEqual(result['ks_volts'],.5,places=9)
        self.assertEqual(result['residual_rejected'],1)


if __name__=='__main__': unittest.main()
