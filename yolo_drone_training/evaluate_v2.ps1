param(
  [ValidateSet('val','final_test','regression_test')][string]$Split = 'val',
  [string]$Weights = ''
)
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $here '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) { throw "Virtual environment missing: $python" }
$env:YOLO_CONFIG_DIR = Join-Path $here '.ultralytics'
$env:MPLCONFIGDIR = Join-Path $here '.mplconfig'
$argv = @((Join-Path $here 'evaluate_v2.py'), '--split', $Split)
if ($Weights) { $argv += @('--weights', $Weights) }
& $python @argv
if ($LASTEXITCODE -ne 0) { throw "Evaluation exited with code $LASTEXITCODE" }
