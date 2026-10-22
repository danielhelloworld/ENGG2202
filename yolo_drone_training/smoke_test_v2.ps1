$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $here '.venv\Scripts\python.exe'
$env:YOLO_CONFIG_DIR = Join-Path $here '.ultralytics'
$env:MPLCONFIGDIR = Join-Path $here '.mplconfig'
& $python (Join-Path $here 'smoke_test_v2.py')
if ($LASTEXITCODE -ne 0) { throw "GPU smoke test exited with code $LASTEXITCODE" }
