# All tests are host-only. No COM port, motor movement or flashing.
param([ValidateSet('200','201','202','203','204','205')][string]$ControlRevision = '200')
$ErrorActionPreference = 'Stop'
$controlProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location -LiteralPath $controlProjectRoot
try {
    & python -B Tools/tests/motion/run_v0200_tests.py --revision $ControlRevision
    if ($LASTEXITCODE -ne 0) { throw 'revision 200 regression failed' }
    & python -B Tools/tests/test_wheel_feedforward_fit.py
    if ($LASTEXITCODE -ne 0) { throw 'feedforward identification regression failed' }
    & powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_motion_tests.ps1
    if ($LASTEXITCODE -ne 0) { throw 'revision 191 compatibility regression failed' }
    foreach ($usbControlRevision in @('191', $ControlRevision)) {
        & powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_usb_tx_tests.ps1 -ControlRevision $usbControlRevision
        if ($LASTEXITCODE -ne 0) { throw "USB regression failed: $usbControlRevision" }
    }
    if (Test-Path -LiteralPath 'build/backups/v0191-before-v0200/manifest.json') {
        & python -B Tools/preserve_control_baseline.py
        if ($LASTEXITCODE -ne 0) { throw 'old firmware preservation verification failed' }
    } else {
        Write-Host 'SKIP old-artifact verification: original workspace backup is not present'
    }
} finally { Pop-Location }
