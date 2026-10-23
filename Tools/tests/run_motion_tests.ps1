# Current production policy only. Historical waveform tests remain in the Git baseline.
# Host-only; never opens ports, moves motors or flashes firmware.
$ErrorActionPreference = 'Stop'
$motionRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location -LiteralPath $motionRoot
try {
    & python -B Tools/tests/motion/run_simplified_tests.py
    if ($LASTEXITCODE -ne 0) { throw 'simplified motion regression failed' }
} finally { Pop-Location }
