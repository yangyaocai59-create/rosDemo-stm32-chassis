"""Build, regress and verify a candidate; never opens ports or flashes."""
import hashlib
import json
import os
import pathlib
import re
import shutil
import struct
import subprocess

root = next(p for p in pathlib.Path(__file__).resolve().parents
            if (p/'Tools/tests/run_motion_tests.ps1').is_file())
version = '0.18.16-lf-support-coordination'
bundle = pathlib.Path('C:/Users/yang/AppData/Local/stm32cube/bundles')
bins = [bundle/'gnu-tools-for-stm32/14.3.1+st.2/bin',
        bundle/'cmake/4.3.1+st.1/bin', bundle/'ninja/1.13.2+st.1/bin']
env = {**os.environ, 'PATH': os.pathsep.join(map(str, bins)) + os.pathsep + os.environ['PATH']}
logs = root/'build/hardware_tests'
archive = root/'build/firmware/0.18.16'
assert not archive.exists(), 'Refuse to overwrite an existing candidate directory'


def run(label, command):
    result = subprocess.run(command, cwd=root, env=env, capture_output=True,
                            text=True, encoding='utf-8', errors='replace')
    output = result.stdout + result.stderr
    (logs/f'v01816_{label}.log').write_text(output, encoding='utf-8')
    print(f'{label}: exit={result.returncode}, PASS lines={len(re.findall(r"^PASS:", output, re.M))}', flush=True)
    if result.returncode:
        print(output[-6000:], flush=True)
        raise RuntimeError(f'{label} failed; artifact not packaged')
    return output


motion = run('motion_regression', ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', 'Tools/tests/run_motion_tests.ps1'])
usb = run('usb_regression', ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', 'Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:', motion, re.M)) == 42
assert len(re.findall(r'^PASS:', usb, re.M)) == 20
build = run('debug_build', [str(bins[1]/'cmake.exe'), '--build', '--preset', 'Debug', '--clean-first'])
assert not re.search(r'\b(?:warning|error):', build, re.I)
memory = {k: int(v) for k, v in re.findall(r'^\s*(RAM|CCMRAM|FLASH):\s+(\d+)\s+B\b', build, re.M)}
assert set(memory) == {'RAM', 'CCMRAM', 'FLASH'}, build[-2000:]
assert memory['FLASH'] < 768*1024 and memory['RAM'] < 128*1024

baseline = json.loads((root/'build/firmware/0.18.15/source_manifest.json').read_text(encoding='utf-8'))
new_files = ['Tools/tests/motion/test_lf_coordination.c', 'Tools/tests/motion/test_lf_isolation.c', 'Tools/tests/motion/run_lf_isolation.py']
files = list(baseline['files']) + new_files
source_hashes = {f: hashlib.sha256((root/f).read_bytes()).hexdigest() for f in sorted(files)}
changed = sorted(f for f, old in baseline['files'].items() if source_hashes[f] != old)
assert set(changed) == {
    'Application/Chassis/wheel_speed_control_config.h', 'Application/Chassis/wheel_speed_controller.c',
    'Application/System/firmware_info.h',
    'Tools/tests/motion/test_front_pivot.c', 'Tools/tests/motion/test_front_support_handoff.c',
    'Tools/tests/motion/test_front_load_confirmation.c',
    'Tools/tests/run_motion_tests.ps1'}, changed

golden_folder = root/'docs/stm32_evidence/2026-10-21-v01815-lf-cycle-review/source_snapshot'
golden_hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in golden_folder.iterdir() if p.is_file()}
assert len(golden_hashes) == 5
assert all(h == baseline['files']['Application/Chassis/'+pathlib.Path(p).name]
           for p,h in golden_hashes.items())
prototype_c = root/'docs/stm32_evidence/2026-10-21-lf-coordination-prototype-v1/prototype/wheel_speed_controller.c'
assert hashlib.sha256(prototype_c.read_bytes()).hexdigest() == source_hashes['Application/Chassis/wheel_speed_controller.c']
old_hex = root/'build/firmware/0.18.15/rosDemo-0.18.15.hex'
assert hashlib.sha256(old_hex.read_bytes()).hexdigest() == '55e8b5b9563822dcf1595b1fa86d321dc6ebeaea81f7608c7ffbdbef184b2ba5'
stack_lines = []
for stack_file in (root/'build/Debug').rglob('*.su'):
    for line in stack_file.read_text(encoding='utf-8', errors='replace').splitlines():
        if any(n+chr(9) in line for n in ('WheelSpeedController_FrontSupport', 'WheelSpeedController_UpdateWithMode',
                                          'FaultManager_ObserveMotion', 'StartSafetyTask')):
            stack_lines.append(line)
assert len(stack_lines) == 4, stack_lines

image = root/'build/Debug/rosDemo'
subprocess.run([str(bins[0]/'arm-none-eabi-objcopy.exe'), '-O', 'binary', str(image.with_suffix('.elf')),
                str(image.with_suffix('.bin'))], check=True, cwd=root, env=env)
data, upper, records, eof = {}, 0, 0, False
for line in image.with_suffix('.hex').read_text(encoding='ascii').splitlines():
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
            address = upper + offset + i
            assert address not in data
            data[address] = byte
    elif kind == 1:
        assert length == 0 and offset == 0
        eof = True
    elif kind == 5:
        assert length == 4
    else:
        raise AssertionError(f'Unexpected HEX record type {kind}')
assert eof and min(data) == 0x08000000 and max(data) < 0x080C0000
binary = image.with_suffix('.bin').read_bytes()
assert len(binary) == max(data)-min(data)+1 == memory['FLASH']
assert all(binary[address-0x08000000] == byte for address, byte in data.items())
stack, reset = struct.unpack_from('<II', binary)
assert stack == 0x20020000 and reset & 1 and 0x08000000 <= (reset & ~1) <= max(data)
assert version.encode('ascii') in binary
gaps = [hex(address) for address in range(min(data), max(data)+1) if address not in data]
assert all(binary[int(address, 16)-0x08000000] == 0 for address in gaps)

archive.mkdir()
hashes = {}
for suffix in ('hex', 'bin', 'elf', 'map'):
    source = image.with_suffix('.'+suffix)
    destination = archive/f'rosDemo-0.18.16.{suffix}'
    shutil.copy2(source, destination)
    hashes[suffix] = hashlib.sha256(destination.read_bytes()).hexdigest()
manifest = {'firmware': version, 'git_base': baseline['git_base'], 'dirty_worktree': True,
            'files': source_hashes, 'frozen_0.18.15_test_baseline': golden_hashes, 'changed_relative_to_0.18.15': changed,
            'new_relative_to_0.18.15': new_files,
            'note': 'User-owned source/config and relevant tests hashed. Cumulative uncommitted v0.18.6..0.18.16; base commit is not candidate identity. Vendor libraries/toolchain not embedded.'}
(archive/'source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
verification = {'result': 'PASS', 'firmware': version, 'build': 'Debug clean, GNU STM32 14.3.1+st.2',
                'motion_pass_lines': 42, 'usb_pass_lines': 20,
                'hex_records': records, 'hex_data_bytes': len(data), 'hex_bin_explicit_data_equal': True,
                'binary_gap_bytes': len(gaps), 'binary_gap_fill': '0x00 default objcopy; HEX leaves unused alignment bytes unwritten',
                'binary_gap_addresses': gaps, 'bytes': len(binary), 'highest': hex(max(data)),
                'stack': hex(stack), 'reset': hex(reset), 'errors': 0, 'warnings': 0,
                'memory_bytes': memory, 'static_stack_lines': stack_lines, 'sha256': hashes, 'hardware_tested': False, 'flashed': False}
(archive/'verification.json').write_text(json.dumps(verification, indent=2)+'\n', encoding='utf-8')
print(json.dumps(verification, indent=2), flush=True)
print('ARTIFACT '+str(archive), flush=True)
