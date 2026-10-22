$ErrorActionPreference = 'Stop'
throw 'RGB-T expansion is still under visual review. This legacy RGB-only entry point is intentionally disabled. See HANDOFF_2026-10-22.md before preparing formal training.'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $here '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) { throw "Virtual environment missing: $python" }
$env:VIRTUAL_ENV = Join-Path $here '.venv'
$env:PATH = "$(Join-Path $env:VIRTUAL_ENV 'Scripts');$env:PATH"
$env:YOLO_CONFIG_DIR = Join-Path $here '.ultralytics'
$env:MPLCONFIGDIR = Join-Path $here '.mplconfig'
& $python (Join-Path $here 'train_v2.py')
if ($LASTEXITCODE -ne 0) { throw "Training exited with code $LASTEXITCODE" }
