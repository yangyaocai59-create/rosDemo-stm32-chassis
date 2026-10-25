"""Package the local workspace status extension; no network or device access."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'Tools/vscode-firmware-status'
package=json.loads((source/'package.json').read_text(encoding='utf-8'))
out=ROOT/'build/vscode'/(package['publisher']+'-'+package['name']+'-'+package['version']+'.vsix')
out.parent.mkdir(parents=True,exist_ok=True)
manifest=f'''<?xml version="1.0" encoding="utf-8"?>
<PackageManifest Version="2.0.0" xmlns="http://schemas.microsoft.com/developer/vsx-schema/2011">
<Metadata><Identity Language="en-US" Id="{package['name']}" Version="{package['version']}" Publisher="{package['publisher']}"/>
<DisplayName>rosDemo Firmware Status</DisplayName><Description xml:space="preserve">Local successful build and reported flash status</Description>
<Properties><Property Id="Microsoft.VisualStudio.Code.Engine" Value="^1.94.0"/><Property Id="Microsoft.VisualStudio.Code.ExtensionKind" Value="workspace"/></Properties></Metadata>
<Installation><InstallationTarget Id="Microsoft.VisualStudio.Code"/></Installation><Dependencies/>
<Assets><Asset Type="Microsoft.VisualStudio.Code.Manifest" Path="extension/package.json" Addressable="true"/></Assets></PackageManifest>'''
types='''<?xml version="1.0" encoding="utf-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="json" ContentType="application/json"/><Default Extension="js" ContentType="application/javascript"/><Default Extension="vsixmanifest" ContentType="text/xml"/></Types>'''
if not out.exists():
    with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr('extension.vsixmanifest',manifest)
        archive.writestr('[Content_Types].xml',types)
        for path in sorted(source.iterdir()):
            if path.is_file():archive.write(path,'extension/'+path.name)
else:
    with zipfile.ZipFile(out) as archive:
        assert all(archive.read('extension/'+p.name)==p.read_bytes() for p in source.iterdir() if p.is_file()),'Existing VSIX differs; bump extension version'
print(str(out))
print('SHA256 '+hashlib.sha256(out.read_bytes()).hexdigest())
