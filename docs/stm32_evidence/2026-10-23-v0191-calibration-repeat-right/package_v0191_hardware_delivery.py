"""Archive authorized .19.1 captures and the existing image; no device access."""
from pathlib import Path
import hashlib, json, shutil, subprocess, zipfile

root=Path.cwd()
hardware=root/'build/hardware_tests'
target=root/'docs/stm32_evidence/2026-10-23-v0191-calibration-repeat-right'
target.mkdir(parents=True,exist_ok=True)
mapping={
 'calibration':'v0191_calibrate_once_20261023_092853',
 'reboot_readonly':'v0191_readonly_20261023_093026',
 'left_repeat':'v0191_400mrad_ground_left_20261023_093049',
 'right':'v0191_400mrad_ground_right_20261023_093219',
}
for name,source in mapping.items():
 dst=target/name
 if dst.exists():
  expected={p.relative_to(hardware/source).as_posix():p.read_bytes() for p in (hardware/source).rglob('*') if p.is_file()}
  actual={p.relative_to(dst).as_posix():p.read_bytes() for p in dst.rglob('*') if p.is_file()}
  assert actual==expected,'Existing capture differs; refusing overwrite: '+str(dst)
 else:shutil.copytree(hardware/source,dst)
for name in ('v0191_calibrate_once.py','v0191_readonly.py','v0191_pivot_400_once.py'):
 shutil.copy2(root/'build'/name,target/name)
shutil.copy2(Path(__file__),target/Path(__file__).name)
source=root/'build/firmware/0.19.1'
manifest=json.loads((source/'source_manifest.json').read_text(encoding='utf-8-sig'))
mismatches=[name for name,sha in manifest['files'].items() if hashlib.sha256((root/name).read_bytes()).hexdigest()!=sha]
assert mismatches==['Tools/assess_existing_turn_yaw.py'],mismatches
assert all(hashlib.sha256((root/name).read_bytes()).hexdigest()==sha for name,sha in manifest['files'].items() if not name.startswith('Tools/'))
verification=json.loads((source/'verification.json').read_text(encoding='utf-8-sig'))
for suffix,sha in verification['sha256'].items():
 assert hashlib.sha256((source/('rosDemo-0.19.1.'+suffix)).read_bytes()).hexdigest()==sha
release=root/'releases/0.19.1';release.mkdir(parents=True,exist_ok=True)
archive=release/'rosDemo-0.19.1-existing-image.zip'
assert not archive.exists(),'Refusing to overwrite firmware archive'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as z:
 for name in ('rosDemo-0.19.1.hex','rosDemo-0.19.1.bin','rosDemo-0.19.1.elf','rosDemo-0.19.1.map','source_manifest.json','verification.json'):
  z.write(source/name,name)
 z.writestr('SOURCE_REVISION.txt','68b4597 Align skid-steer command and feedback geometry with effective track\nBinary unchanged after hardware testing; no new build or flash.\nverification.json retains the original offline-stage status. See current evidence for hardware results.\n')
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 for name in z.namelist():
  if name!='SOURCE_REVISION.txt':assert z.read(name)==(source/name).read_bytes()
(release/'SHA256SUMS.txt').write_text(hashlib.sha256(archive.read_bytes()).hexdigest()+'  '+archive.name+'\n'+verification['sha256']['hex']+'  rosDemo-0.19.1.hex (inside archive)\n',encoding='utf-8')
record={'firmware_source_commit':'68b4597','source_manifest_files_checked':len(manifest['files']),'source_manifest_mismatches':mismatches,'offline_tool_change':'Reference analysis now reports bias flags correctly and records same-sample updates outside the integration window; rejects differing duplicates inside it. No firmware input changed.','firmware_sha256':verification['sha256'],'firmware_archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'capture_mapping':mapping,'operator_report':'After full power cycle: ground, VM on, surrounding area clear. No material change to floor, payload or tyres between left runs.','production_source_changed_this_session':False,'new_firmware_flash_this_session':False,'navigation_acceptance':False}
(target/'delivery_verification.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
print(json.dumps(record,indent=2))
