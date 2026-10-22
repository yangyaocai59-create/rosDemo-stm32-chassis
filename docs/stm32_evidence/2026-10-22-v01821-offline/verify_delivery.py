"""Independent delivery checks and scratch ELF objcopy; no hardware/ports."""
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import tempfile

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Application').is_dir())
archive=root/'build/firmware/0.18.21'
manifest=json.loads((archive/'source_manifest.json').read_text())
proof=json.loads((archive/'verification.json').read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert len(manifest['files'])==167
assert all(sha(root/n)==h for n,h in manifest['files'].items())
assert all(sha(root/n)==h for n,h in manifest['frozen_0.18.20_baseline'].items())
assert all(sha(root/n)==h for n,h in manifest['validated_offline_prototype'].items())
for n in ('source_manifest.json','verification.json','stack_review.json'):
    assert sha(here/n)==sha(archive/n)
for ext,h in proof['sha256'].items():assert sha(archive/f'rosDemo-0.18.21.{ext}')==h
for version in ('0.18.19','0.18.20'):
    old=json.loads((root/f'build/firmware/{version}/verification.json').read_text())
    for ext,h in old['sha256'].items():
        assert sha(root/f'build/firmware/{version}/rosDemo-{version}.{ext}')==h
changes=manifest['changed_relative_to_0.18.20']+manifest['new_relative_to_0.18.20']
assert len(changes)==17 and len(list((here/'source_snapshot').iterdir()))==17
for n in changes:assert sha(here/'source_snapshot'/Path(n).name)==manifest['files'][n]
# Verify original experiments remain sealed, including their source/fixture
# snapshots, not merely their comparison output CSVs.
for directory,report in (('2026-10-22-rr-exit-prototype-v1','verification.json'),
                         ('2026-10-22-v01820-running-review','analysis.json')):
    folder=here.parent/directory
    old=json.loads((folder/report).read_text())
    if 'files' in old:assert all(sha(folder/n)==h for n,h in old['files'].items())
capture=here.parent/'2026-10-22-v01820-ground-left'
sealed=json.loads((capture/'capture_manifest.json').read_text())
assert len(sealed)==21 and all(sha(capture/n)==h for n,h in sealed.items())
old20=json.loads((root/'build/firmware/0.18.20/source_manifest.json').read_text())
unchanged_paths=[n for n in old20['files'] if n not in manifest['changed_relative_to_0.18.20']]
assert all(sha(root/n)==old20['files'][n] for n in unchanged_paths)
for prefix in ('Application/Sensors/','Application/Storage/','Application/Communication/',
               'Application/Safety/','Core/','USB_DEVICE/'):
    assert all(sha(root/n)==h for n,h in old20['files'].items() if n.startswith(prefix))
assert sha(root/'Application/Chassis/wheel_speed_estimator.c')==old20['files']['Application/Chassis/wheel_speed_estimator.c']
build=(here/'v01821_debug_build.log').read_text()
assert '[95/95]' in build and not re.search(r'\b(?:warning|error):',build,re.I)
assert proof['firmware_build_errors']==proof['firmware_build_warnings']==0
assert proof['motion_pass_lines']==96 and proof['usb_pass_lines']==20
assert proof['rr_exit_regression']['automatic_fault_pairs']==408
assert proof['rr_exit_regression']['max_added_stall_delay_ms']==0
assert proof['strict_extended_imu_quality_suite_pass'] is False
assert not any(proof[n] for n in ('hardware_tested','flashed','serial_opened','motion_sent','pushed'))
objcopy=Path('C:/Users/yang/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-objcopy.exe')
with tempfile.TemporaryDirectory(prefix='v01821_delivery_verify_') as scratch:
    for fmt,ext in (('ihex','hex'),('binary','bin')):
        p=Path(scratch)/('verify.'+ext)
        subprocess.run([str(objcopy),'-O',fmt,str(archive/'rosDemo-0.18.21.elf'),str(p)],check=True)
        assert sha(p)==proof['sha256'][ext]
binary=(archive/'rosDemo-0.18.21.bin').read_bytes()
assert len(binary)==183024
assert b'0.18.21-rr-recovery-exit' in binary and b'0.18.20-rr-running-recovery' not in binary
assert struct.unpack_from('<II',binary)==(0x20020000,0x0800F439)
status=subprocess.run(['git','diff','--check'],cwd=root,capture_output=True,text=True)
assert status.returncode==0,status.stderr+status.stdout
result=dict(result='PASS_DELIVERY_INTEGRITY_NOT_HARDWARE',firmware=proof['firmware'],
    source_files_checked=167,snapshot_files_checked=17,frozen_baseline_files_checked=19,
    candidate_artifacts_checked=4,old_19_and_20_artifacts_unchanged=True,
    sealed_capture_files_checked=21,approved_prototype_history_unchanged=True,
    unchanged_IMU_USB_encoder_safety_and_Core=True,
    HEX_and_BIN_regenerated_from_archived_ELF_equal=True,motion_pass_groups=96,
    usb_pass_groups=20,new_RR_automatic_fault_pairs=408,RTOS_timing_measured=False,
    strict_extended_imu_suite_pass=False,known_host_test_limitation_preserved=True,
    firmware_build_warnings=0,firmware_build_errors=0,flashed=False,hardware_tested=False,
    serial_opened=False,pushed=False)
result['delivery_files_sha256']={str(p.relative_to(root)):sha(p) for p in here.rglob('*')
  if p.is_file() and p.name!='delivery_integrity.json'}
doc='docs/STM32_RR_RECOVERY_EXIT_0.18.21_20261022.md'
result['delivery_files_sha256'][doc]=sha(root/doc)
(here/'delivery_integrity.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='delivery_files_sha256'},indent=2))
