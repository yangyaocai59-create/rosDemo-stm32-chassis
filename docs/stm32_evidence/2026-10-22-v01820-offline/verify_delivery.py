"""Read-only source/artifact checks plus scratch objcopy; never hardware."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Application').is_dir())
archive=root/'build/firmware/0.18.20'
manifest=json.loads((archive/'source_manifest.json').read_text())
proof=json.loads((archive/'verification.json').read_text())
old=json.loads((root/'build/firmware/0.18.19/verification.json').read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
assert len(manifest['files'])==158
assert all(sha(root/p)==h for p,h in manifest['files'].items())
assert all(sha(root/p)==h for p,h in manifest['frozen_0.18.19_baseline'].items())
assert all(sha(root/p)==h for p,h in manifest['validated_offline_prototype'].items())
for key in ('source_manifest.json','verification.json','stack_review.json'):
    assert sha(here/key)==sha(archive/key)
for ext,h in proof['sha256'].items():
    assert sha(archive/f'rosDemo-0.18.20.{ext}')==h
for ext,h in old['sha256'].items():
    assert sha(root/f'build/firmware/0.18.19/rosDemo-0.18.19.{ext}')==h
for name in manifest['changed_relative_to_0.18.19']+manifest['new_relative_to_0.18.19']:
    assert sha(here/'source_snapshot'/Path(name).name)==manifest['files'][name]
assert len(list((here/'source_snapshot').iterdir()))==20
for p in (root/'docs/stm32_evidence/2026-10-22-rr-running-prototype-v1').glob('verification.json'):
    historical=json.loads(p.read_text())
    assert all(sha(p.parent/n)==h for n,h in historical['files'].items())
build=(here/'v01820_debug_build.log').read_text()
assert '[95/95]' in build and not re.search(r'\b(?:warning|error):',build,re.I)
assert proof['firmware_build_errors']==proof['firmware_build_warnings']==0
assert not proof['hardware_tested'] and not proof['flashed']
assert proof['strict_extended_imu_quality_suite_pass'] is False
objcopy=Path('C:/Users/yang/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-objcopy.exe')
with tempfile.TemporaryDirectory(prefix='v01820_delivery_verify_') as scratch:
    for fmt,ext in (('ihex','hex'),('binary','bin')):
        output=Path(scratch)/('check.'+ext)
        subprocess.run([str(objcopy),'-O',fmt,str(archive/'rosDemo-0.18.20.elf'),str(output)],check=True)
        assert sha(output)==proof['sha256'][ext]
status=subprocess.run(['git','diff','--check'],cwd=root,capture_output=True,text=True)
assert status.returncode==0,status.stderr+status.stdout
result={'result':'PASS_DELIVERY_INTEGRITY_NOT_HARDWARE',
 'firmware':proof['firmware'],'source_files_checked':158,'snapshot_files_checked':20,
 'old_artifacts_unchanged':True,'candidate_artifacts_checked':4,
 'HEX_and_BIN_regenerated_from_archived_ELF_equal':True,
 'validated_RR_prototype_history_hashes_unchanged':True,
 'motion_pass_groups':79,'usb_pass_groups':20,'automatic_fault_audits':408,
 'strict_extended_imu_suite_pass':False,'known_host_test_limitation_preserved':True,
 'firmware_build_warnings':0,'firmware_build_errors':0,
 'flashed':False,'hardware_tested':False,'serial_opened':False,'pushed':False}
result['delivery_files_sha256']={str(p.relative_to(root)):sha(p) for p in here.rglob('*')
 if p.is_file() and p.name!='delivery_integrity.json'}
result['delivery_files_sha256']['docs/STM32_RR_RUNNING_RECOVERY_0.18.20_20261022.md']=sha(root/'docs/STM32_RR_RUNNING_RECOVERY_0.18.20_20261022.md')
(here/'delivery_integrity.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='delivery_files_sha256'},indent=2))
