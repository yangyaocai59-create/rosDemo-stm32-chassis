"""Regress/build/verify/package .17 offline. Never opens ports or flashes."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess

root = next(p for p in Path(__file__).resolve().parents
            if (p/'Tools/tests/run_motion_tests.ps1').is_file())
version = '0.18.17-rf-support-coordination'
bundle = Path('C:/Users/yang/AppData/Local/stm32cube/bundles')
bins = [bundle/'gnu-tools-for-stm32/14.3.1+st.2/bin',
        bundle/'cmake/4.3.1+st.1/bin', bundle/'ninja/1.13.2+st.1/bin']
env = {**os.environ, 'PATH': os.pathsep.join(map(str, bins))+os.pathsep+os.environ['PATH']}
logs = root/'build/hardware_tests'
archive = root/'build/firmware/0.18.17'
assert not archive.exists(), 'Refuse to overwrite existing candidate'
baseline = json.loads((root/'build/firmware/0.18.16/source_manifest.json').read_text())
new_files = ['Tools/tests/motion/run_rf_isolation.py', 'Tools/tests/motion/test_rf_isolation.c']
files = sorted(list(baseline['files'])+new_files)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

source_hashes = {name: sha(root/name) for name in files}
changed = sorted(name for name, old in baseline['files'].items() if source_hashes[name] != old)
expected = {'Application/Chassis/wheel_speed_controller.c',
            'Application/Chassis/wheel_speed_control_config.h', 'Application/System/firmware_info.h',
            'Tools/tests/motion/test_front_pivot.c', 'Tools/tests/motion/test_front_support_handoff.c',
            'Tools/tests/motion/test_front_load_confirmation.c', 'Tools/tests/motion/test_lf_coordination.c',
            'Tools/tests/run_motion_tests.ps1'}
assert set(changed) == expected, changed
golden = root/'docs/stm32_evidence/2026-10-21-v01816-rf-rr-cycle-review/source_snapshot'
golden_hashes = {str(p.relative_to(root)): sha(p) for p in golden.iterdir() if p.is_file()}
assert len(golden_hashes) == 5
assert all(value == baseline['files']['Application/Chassis/'+Path(name).name]
           for name, value in golden_hashes.items())
assert sha(root/'build/firmware/0.18.16/rosDemo-0.18.16.hex') == 'e67a36800caf447839c2a68a5bcdcf8fac18286085b37251844e97954cab2ede'

def run(label, command):
    result = subprocess.run(command, cwd=root, env=env, capture_output=True,
                            text=True, encoding='utf-8', errors='replace')
    output = result.stdout+result.stderr
    (logs/f'v01817_{label}.log').write_text(output, encoding='utf-8')
    print(f'{label}: exit={result.returncode}, PASS lines={len(re.findall(r"^PASS:", output, re.M))}', flush=True)
    if result.returncode:
        print(output[-6000:], flush=True)
        raise RuntimeError(label+' failed; no artifact package')
    return output

motion = run('motion_regression', ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass',
                                  '-File', 'Tools/tests/run_motion_tests.ps1'])
usb = run('usb_regression', ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass',
                            '-File', 'Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:', motion, re.M)) == 43
assert len(re.findall(r'^PASS:', usb, re.M)) == 20
build = run('debug_build', [str(bins[1]/'cmake.exe'), '--build', '--preset', 'Debug', '--clean-first'])
assert not re.search(r'\b(?:warning|error):', build, re.I)
memory = {name: int(value) for name, value in
          re.findall(r'^\s*(RAM|CCMRAM|FLASH):\s+(\d+)\s+B\b', build, re.M)}
assert set(memory) == {'RAM', 'CCMRAM', 'FLASH'}
assert memory['RAM'] < 128*1024 and memory['FLASH'] < 768*1024
assert {name: sha(root/name) for name in files} == source_hashes
stack_lines = []
for p in (root/'build/Debug').rglob('*.su'):
    stack_lines += [line for line in p.read_text(errors='replace').splitlines()
                    if any(n+chr(9) in line for n in ('WheelSpeedController_FrontSupport',
                      'WheelSpeedController_UpdateWithMode', 'FaultManager_ObserveMotion', 'StartSafetyTask'))]
assert len(stack_lines) == 4

image = root/'build/Debug/rosDemo'
subprocess.run([str(bins[0]/'arm-none-eabi-objcopy.exe'), '-O', 'binary',
                str(image.with_suffix('.elf')), str(image.with_suffix('.bin'))], check=True, env=env)
data, upper, records, eof = {}, 0, 0, False
for line in image.with_suffix('.hex').read_text().splitlines():
    assert not eof and line.startswith(':')
    record = bytes.fromhex(line[1:])
    assert len(record) == record[0]+5 and sum(record) & 255 == 0
    length, offset, kind = record[0], int.from_bytes(record[1:3], 'big'), record[3]
    payload = record[4:4+length]
    records += 1
    if kind == 4:
        assert length == 2 and offset == 0
        upper = int.from_bytes(payload, 'big') << 16
    elif kind == 0:
        for i, byte in enumerate(payload):
            address = upper+offset+i
            assert address not in data
            data[address] = byte
    elif kind == 1:
        assert length == 0 and offset == 0
        eof = True
    elif kind == 5:
        assert length == 4
    else:
        raise AssertionError(kind)
assert eof and min(data) == 0x08000000 and max(data) < 0x080C0000
binary = image.with_suffix('.bin').read_bytes()
assert len(binary) == max(data)-min(data)+1 == memory['FLASH']
assert all(binary[address-0x08000000] == byte for address, byte in data.items())
gaps = [hex(address) for address in range(min(data), max(data)+1) if address not in data]
assert all(binary[int(address,16)-0x08000000] == 0 for address in gaps)
stack, reset = struct.unpack_from('<II', binary)
assert stack == 0x20020000 and reset & 1 and 0x08000000 <= reset & ~1 <= max(data)
assert version.encode() in binary
assert b'0.18.16-lf-support-coordination' not in binary

archive.mkdir()
hashes = {}
for suffix in ('hex', 'bin', 'elf', 'map'):
    destination = archive/f'rosDemo-0.18.17.{suffix}'
    shutil.copy2(image.with_suffix('.'+suffix), destination)
    hashes[suffix] = sha(destination)
manifest = {'firmware': version, 'git_base': baseline['git_base'], 'dirty_worktree': True,
            'files': source_hashes, 'frozen_0.18.16_test_baseline': golden_hashes,
            'changed_relative_to_0.18.16': changed, 'new_relative_to_0.18.16': new_files,
            'note': 'Cumulative uncommitted .6..17; base commit is not candidate identity. Vendor/toolchain not embedded.'}
(archive/'source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
verification = {'result': 'PASS', 'firmware': version, 'build': 'Debug clean, GNU STM32 14.3.1+st.2',
    'motion_pass_lines': 43, 'usb_pass_lines': 20, 'errors': 0, 'warnings': 0,
    'memory_bytes': memory, 'static_stack_lines': stack_lines, 'hex_records': records,
    'hex_data_bytes': len(data), 'hex_bin_explicit_data_equal': True, 'bytes': len(binary),
    'binary_gap_bytes': len(gaps), 'binary_gap_addresses': gaps,
    'binary_gap_fill': '0x00 default objcopy; HEX leaves alignment bytes unwritten',
    'highest': hex(max(data)), 'stack': hex(stack), 'reset': hex(reset),
    'sha256': hashes, 'flashed': False, 'hardware_tested': False}
(archive/'verification.json').write_text(json.dumps(verification, indent=2)+'\n')
print(json.dumps(verification, indent=2), flush=True)
print('ARTIFACT '+str(archive), flush=True)
