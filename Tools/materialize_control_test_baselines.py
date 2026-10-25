"""Seal the minimal historical C fixtures for portable differential tests.

Existing fixtures are verified, never refreshed. Only initial materialization
requires the original repository and its historical evidence directory.
"""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'Tools/tests/baselines'


def sha(data):return hashlib.sha256(data).hexdigest()


def main():
    for version,reference,names in (
        ('v01825','f62c66d',('wheel_speed_controller.c','wheel_speed_controller.h',
                            'wheel_speed_control_config.h','unified_pivot_control.h')),
        ('v01821','docs/stm32_evidence/2026-10-22-unified-pivot-v1/baseline_0.18.21',
         ('wheel_speed_controller.c','wheel_speed_controller.h','wheel_speed_control_config.h'))):
        folder=BASE/version
        if folder.exists():
            manifest=json.loads((folder/'manifest.json').read_text())
            for name,expected in manifest['files'].items():
                assert sha((folder/name).read_bytes())==expected,version+'/'+name
            print(version+': sealed fixtures verified');continue
        files={}
        for name in names:
            files[name]=(subprocess.check_output(['git','show',reference+':Application/Chassis/'+name],cwd=ROOT)
                         if version=='v01825' else (ROOT/reference/name).read_bytes())
        folder.mkdir(parents=True,exist_ok=False)
        for name,data in files.items():
            with (folder/name).open('xb') as stream:stream.write(data)
        with (folder/'manifest.json').open('x',encoding='utf-8') as stream:
            json.dump({'reference':reference,'files':{n:sha(b) for n,b in files.items()}},stream,indent=2)
        print(version+': sealed '+str(len(files))+' fixtures')


if __name__=='__main__':main()
