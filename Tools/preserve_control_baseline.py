"""Preserve exact source bytes and every existing firmware/release artifact.

Host-only. Existing snapshots are verified, never updated or replaced.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/backups/v0191-before-v0200'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify():
    manifest = json.loads((OUT / 'manifest.json').read_text(encoding='utf-8'))
    for name, expected in manifest['artifacts'].items():
        assert digest(ROOT / name) == expected, 'Old artifact changed: ' + name
    assert digest(OUT / 'source.zip') == manifest['source_zip_sha256']
    with zipfile.ZipFile(OUT / 'source.zip') as archive:
        for name, expected in manifest['source'].items():
            assert hashlib.sha256(archive.read(name)).hexdigest() == expected
    print(f'PASS: {len(manifest["source"])} source files, '
          f'{len(manifest["artifacts"])} old artifacts preserved')
    return manifest


if __name__ == '__main__':
    if not OUT.exists():
        OUT.mkdir(parents=True, exist_ok=False)
        tracked = subprocess.check_output(
            ['git', 'ls-files', '-z'], cwd=ROOT).decode().split('\0')
        folders = ('Application/', 'Core/', 'USB_DEVICE/', 'cmake/',
                   'Drivers/', 'Middlewares/', 'Tools/')
        top = {'CMakeLists.txt', 'CMakePresets.json', 'rosDemo.ioc',
               'STM32F407xx_FLASH.ld', 'startup_stm32f407xx.s'}
        paths = sorted(name for name in tracked if name and
                       (name.startswith(folders) or name in top))
        source = {name: digest(ROOT / name) for name in paths}
        artifacts = {}
        for folder in (ROOT / 'build/firmware', ROOT / 'releases'):
            if folder.exists():
                for path in sorted(folder.rglob('*')):
                    if path.is_file():
                        artifacts[path.relative_to(ROOT).as_posix()] = digest(path)
        with zipfile.ZipFile(OUT / 'source.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
            for name in paths:
                archive.write(ROOT / name, name)
        manifest = {'head': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
            'source': source, 'artifacts': artifacts,
            'source_zip_sha256': digest(OUT / 'source.zip')}
        with (OUT / 'manifest.json').open('x', encoding='utf-8') as output:
            json.dump(manifest, output, indent=2)
    verify()
