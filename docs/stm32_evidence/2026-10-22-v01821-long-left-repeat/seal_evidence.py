"""Seal the completed one-turn repeat evidence; no hardware access."""
import hashlib
import json
from pathlib import Path

here=Path(__file__).resolve().parent
target=here/'capture_manifest.json'
assert not target.exists(),'Refuse overwrite'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files={p.relative_to(here).as_posix():sha(p) for p in sorted(here.rglob('*'))
       if p.is_file() and p!=target and '__pycache__' not in p.parts}
target.write_text(json.dumps(files,indent=2)+'\n',encoding='utf-8')
assert all(sha(here/n)==h for n,h in files.items())
print(f'Sealed {len(files)} files; no hardware access or movement.')
