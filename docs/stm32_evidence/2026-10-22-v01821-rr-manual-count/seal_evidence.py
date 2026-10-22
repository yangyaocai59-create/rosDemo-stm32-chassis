"""Seal finished manual measurement; no hardware access."""
import hashlib
import json
from pathlib import Path
here=Path(__file__).resolve().parent
target=here/'capture_manifest.json'
assert not target.exists(),'Refuse overwrite'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files={str(p.relative_to(here)):sha(p) for p in here.rglob('*') if p.is_file() and p!=target}
target.write_text(json.dumps(files,indent=2)+'\n')
assert all(sha(here/n)==h for n,h in json.loads(target.read_text()).items())
print(f'Sealed {len(files)} files, NO movement or serial commands.')
