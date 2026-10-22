"""Seal this finished capture only; never opens ports or controls hardware."""
import hashlib
import json
from pathlib import Path

here=Path(__file__).resolve().parent
target=here/'capture_manifest.json'
assert not target.exists(),'Refuse to overwrite sealed evidence'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files={str(p.relative_to(here)):sha(p) for p in here.rglob('*')
       if p.is_file() and p!=target}
target.write_text(json.dumps(files,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
assert all(sha(here/n)==h for n,h in json.loads(target.read_text(encoding='utf-8')).items())
summary=json.loads((here/'capture/summary.json').read_text(encoding='utf-8'))
assert not summary['errors'] and summary['final_status'][5]==1
assert not any(summary['final_status'][10:16])
print(f'Sealed and verified {len(files)} evidence files; chassis stopped, ports closed by completed runner; NO repeat.')
