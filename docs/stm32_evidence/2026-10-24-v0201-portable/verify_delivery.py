"""Independent archive/source verification; never accesses a serial device."""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess
import sys
import zipfile
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parents[3]
EVIDENCE=Path(__file__).resolve().parent
ARCHIVE=ROOT/'build/firmware/0.20.1'
WORK=ROOT/'build/verification/v0201-portable-source-epoch'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    verification=json.loads((ARCHIVE/'verification.json').read_text())
    manifest=json.loads((ARCHIVE/'source_manifest.json').read_text())
    for name,expected in manifest['files'].items():
        assert sha(ROOT/name)==expected,'Working source mismatch: '+name
    for group in ('old_artifacts_preserved','previous_candidate_artifacts_preserved'):
        for name,expected in verification[group].items():assert sha(ROOT/name)==expected,name
    for name,expected in verification['sha256'].items():assert sha(ARCHIVE/name)==expected,name
    package=ROOT/'releases/0.20.1/rosDemo-0.20.1-offline-candidate.zip'
    assert sha(package)==(package.parent/'SHA256SUMS.txt').read_text().split()[0]
    with zipfile.ZipFile(package) as archive:
        assert archive.testzip() is None
        for name in archive.namelist():
            assert hashlib.sha256(archive.read(name)).hexdigest()==sha(ARCHIVE/name),name
    assert sha(ARCHIVE/'source.zip')==verification['source_zip_sha256']
    original=(ARCHIVE/'Release/rosDemo-0.20.1.bin').read_bytes()
    stamp=re.search(rb'([0-2]\d:[0-5]\d:[0-5]\d)\x00([A-Z][a-z]{2} [ \d]\d \d{4})\x00Build =',original)
    assert stamp is not None,'Build banner timestamp missing'
    # GCC emits __DATE__/__TIME__ in UTC when SOURCE_DATE_EPOCH is provided.
    # Treat the archived banner text as the desired label, without claiming
    # this reconstructed epoch is the original build's wall-clock instant.
    label=stamp.group(2).decode()+' '+stamp.group(1).decode()
    epoch=int(datetime.strptime(label,'%b %d %Y %H:%M:%S').replace(tzinfo=timezone.utc).timestamp())
    uncontrolled=ROOT/'build/verification/v0201-portable-source/build/WheelV0201Release/rosDemo-0.20.1.bin'
    differences=[]
    if uncontrolled.exists():
        other=uncontrolled.read_bytes();assert len(other)==len(original)
        differences=[i for i,(a,b) in enumerate(zip(original,other)) if a!=b]
        assert all(stamp.start(1)<=i<stamp.end(2) for i in differences),'Non-timestamp code/data difference'
    assert not WORK.exists(),'Refusing to replace a portable verification workspace'
    WORK.mkdir(parents=True,exist_ok=False)
    with zipfile.ZipFile(ARCHIVE/'source.zip') as archive:
        assert archive.testzip() is None
        for name in archive.namelist():
            assert (WORK/name).resolve().is_relative_to(WORK.resolve()),name
            assert hashlib.sha256(archive.read(name)).hexdigest()==manifest['files'][name],name
        archive.extractall(WORK)
    bundles=Path('C:/Users/yang/AppData/Local/stm32cube/bundles')
    environment={**os.environ,'GIT_CEILING_DIRECTORIES':str(WORK.parent),
        'SOURCE_DATE_EPOCH':str(epoch),
        'PATH':os.pathsep.join((str(bundles/'gnu-tools-for-stm32/14.3.1+st.2/bin'),
                               str(bundles/'ninja/1.13.2+st.1/bin'),os.environ['PATH']))}
    def run(name,args):
        print(name+': running',flush=True)
        p=subprocess.run(list(map(str,args)),cwd=WORK,env=environment,capture_output=True,
                         text=True,encoding='utf-8',errors='replace')
        output=p.stdout+p.stderr
        (EVIDENCE/(name+'.log')).write_text(output,encoding='utf-8')
        assert p.returncode==0 and not re.search(r'\b(?:warning|error):',output,re.I),output[-6000:]
        print(name+': PASS',flush=True)
        return output
    tests=run('portable_repro_host',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',
                              'Tools/tests/run_wheel_v0200_tests.ps1','-ControlRevision','201'])
    assert 'SKIP old-artifact verification' in tests
    cmake=bundles/'cmake/4.3.1+st.1/bin/cmake.exe'
    run('portable_repro_configure',[cmake,'--preset','WheelV0201Release'])
    run('portable_repro_build',[cmake,'--build','--preset','WheelV0201Release'])
    rebuilt=WORK/'build/WheelV0201Release/rosDemo-0.20.1.hex'
    assert sha(rebuilt)==verification['sha256']['Release/rosDemo-0.20.1.hex'],'Portable HEX differs'
    result={'source_files_verified':len(manifest['files']),
        'old_artifacts_verified':len(verification['old_artifacts_preserved']),
        'previous_candidate_files_verified':len(verification['previous_candidate_artifacts_preserved']),
        'portable_host_regressions':'PASS_WITHOUT_GIT_HISTORY',
        'portable_release_hex_identical':True,'release_hex_sha256':sha(rebuilt),
        'archived_build_banner_label':label,'reproduction_source_date_epoch':epoch,
        'uncontrolled_timestamp_difference_bytes':len(differences),
        'release_zip_sha256':sha(package),'hardware_operations':False}
    (EVIDENCE/'verification.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
