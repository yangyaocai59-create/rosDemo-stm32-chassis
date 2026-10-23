"""Download pinned primary-source excerpts for read-only protection review."""
from concurrent.futures import ThreadPoolExecutor
import hashlib,json,base64
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError
HERE=Path(__file__).resolve().parent
def get(url):
    return urlopen(Request(url,headers={'User-Agent':'rosDemo-source-review'}),timeout=30).read()
def fetch(item):
    repo,ref,paths=item
    commit=json.loads(get(f'https://api.github.com/repos/{repo}/commits/{ref}'))['sha']
    files=[]
    for name in paths:
        url=f'https://raw.githubusercontent.com/{repo}/{commit}/{name}'
        data=get(url);dest=HERE/'upstream'/repo.replace('/','_')/name
        dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
        files.append({'file':name,'url':url,'sha256':hashlib.sha256(data).hexdigest()})
    license_status='downloaded'
    try:
        license_info=json.loads(get(f'https://api.github.com/repos/{repo}/license?ref={commit}'))
        license_data=base64.b64decode(license_info['content'])
        license_path=HERE/'upstream'/repo.replace('/','_')/license_info['path']
        license_path.parent.mkdir(parents=True,exist_ok=True);license_path.write_bytes(license_data)
        files.append({'file':license_info['path'],'url':license_info['html_url'],
                      'sha256':hashlib.sha256(license_data).hexdigest(),'kind':'upstream_license'})
    except HTTPError as error:
        if error.code!=404:raise
        root=json.loads(get(f'https://api.github.com/repos/{repo}/contents?ref={commit}'))
        candidates=[f for f in root if f['type']=='file' and
                    (f['name'].lower().startswith('license') or f['name'].lower().startswith('copying'))]
        license_status='root_file_downloaded' if candidates else 'not_found_in_api_or_root_do_not_assume_reuse_permission'
        for f in candidates:
            data=get(f'https://raw.githubusercontent.com/{repo}/{commit}/{f["path"]}')
            dest=HERE/'upstream'/repo.replace('/','_')/f['path'];dest.write_bytes(data)
            files.append({'file':f['path'],'url':f['html_url'],'sha256':hashlib.sha256(data).hexdigest(),'kind':'upstream_license'})
    return {'repo':repo,'ref':ref,'commit':commit,'files':files,'license_status':license_status}
if __name__=='__main__':
    items=[('linorobot/linorobot2_hardware','jazzy',['firmware/src/firmware.ino']),
           ('fictionlab/diff_drive_lib','1.5',['src/wheel_controller.cpp','src/diff_drive_controller.cpp']),
           ('vedderb/bldc','master',['motor/mc_interface.c','timeout.c']),
           ('ArduPilot/ardupilot','master',['Rover/crash_check.cpp','Rover/Parameters.cpp'])]
    with ThreadPoolExecutor(max_workers=3) as pool:result=list(pool.map(fetch,items))
    (HERE/'upstream_manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))
