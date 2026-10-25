"""Publish successful builds and explicitly reported flashes to VS Code.

No serial, flashing, old-image replacement, or VS Code private database edits.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
STATUS=ROOT/'.vscode/firmware-status.json'

def publish(args):
    data=json.loads(STATUS.read_text(encoding='utf-8')) if STATUS.exists() else {}
    now=datetime.now(timezone.utc).isoformat()
    if args.elf:
        elf=args.elf.resolve()
        if not elf.is_relative_to(ROOT/'build') or not elf.is_file():
            raise ValueError('ELF must be an existing project build artifact')
        version=args.version or ('0.19.1' if args.revision==191 else
                                 f'0.{args.revision//10}.{args.revision%10}')
        image=elf.read_bytes()
        if image[:4]!=b'\x7fELF' or (version+'-').encode() not in image:
            raise ValueError('ELF identity does not match the requested version')
        hex_path=elf.with_suffix('.hex')
        if not hex_path.is_file():raise ValueError('matching HEX is missing')
        preset=args.preset or ('WheelV'+version.replace('.','')+args.configuration)
        presets=json.loads((ROOT/'CMakePresets.json').read_text(encoding='utf-8'))
        if preset not in {x['name'] for x in presets['configurePresets']}:
            raise ValueError('unknown configure preset')
        data['build']=dict(version=version,configuration=args.configuration,preset=preset,
            elf=str(elf),hex=str(hex_path),sha256=hashlib.sha256(image).hexdigest(),
            completed_at=now,elf_modified_at=datetime.fromtimestamp(elf.stat().st_mtime,timezone.utc).isoformat())
        cpp=ROOT/'.vscode/c_cpp_properties.json'
        commands=elf.parent/'compile_commands.json'
        if cpp.exists() and commands.exists():
            properties=json.loads(cpp.read_text(encoding='utf-8'))
            for configuration in properties.get('configurations',[]):
                if 'compileCommands' in configuration:
                    configuration['compileCommands']='${workspaceFolder}/'+commands.relative_to(ROOT).as_posix()
                    configuration['configurationProvider']='ms-vscode.cmake-tools'
            cpp.write_text(json.dumps(properties,indent=4)+'\n',encoding='utf-8')
    if args.flashed:
        data['flash']=dict(version=args.flashed,source='user_report',reported_at=now,
            independently_verified=False)
    if args.hardware_note:data['hardware_note']=args.hardware_note
    if not data:raise ValueError('provide a build or explicit user flash report')
    data['updated_at']=now
    STATUS.parent.mkdir(exist_ok=True)
    temporary=STATUS.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    temporary.replace(STATUS)
    print('VS Code firmware status updated: '+str(STATUS))
    return data

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf',type=Path)
    parser.add_argument('--revision',type=int,default=205)
    parser.add_argument('--version')
    parser.add_argument('--preset')
    parser.add_argument('--configuration',choices=('Debug','Release'),default='Release')
    parser.add_argument('--flashed',help='Use only after the operator explicitly reports this version flashed')
    parser.add_argument('--hardware-note')
    publish(parser.parse_args())
