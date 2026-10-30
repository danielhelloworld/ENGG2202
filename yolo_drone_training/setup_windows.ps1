param(
  [Parameter(Mandatory=$true)][string]$UsbRoot,
  [string]$WorkDir = "$env:USERPROFILE\Documents\ENGG2202_lab"
)
$ErrorActionPreference = 'Stop'
$UsbRoot = (Resolve-Path $UsbRoot).Path
function Run-Checked {
  param([string]$Exe, [string[]]$Arguments)
  & $Exe @Arguments
  if ($LASTEXITCODE -ne 0) { throw "Command failed: $Exe $($Arguments -join ' ')" }
}
# Use a normal user install; no permanent execution-policy changes.
$Py = $null
if (Get-Command py -ErrorAction SilentlyContinue) {
  $candidate = & py -3.11 -c "import sys; print(sys.executable)" 2>$null
  if ($LASTEXITCODE -eq 0) { $Py = ($candidate | Select-Object -Last 1).Trim() }
}
if (-not $Py) {
  $candidate = "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe"
  if (Test-Path $candidate) { $Py = $candidate }
}
if (-not $Py) {
  [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
  $installer = Join-Path $env:TEMP 'engg2202-python311.exe'
  Invoke-WebRequest 'https://www.python.org/ftp/python/3.11.9/python-3.11.9-amd64.exe' -OutFile $installer -UseBasicParsing
  $process = Start-Process $installer -ArgumentList '/quiet InstallAllUsers=0 Include_pip=1 Include_test=0 PrependPath=0' -Wait -PassThru
  if ($process.ExitCode -ne 0) { throw 'Python 3.11 installation failed' }
  $Py = "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe"
}
Run-Checked -Exe $Py -Arguments @('-c', 'import sys; assert sys.version_info[:2] == (3,11), sys.version')
New-Item -ItemType Directory -Force $WorkDir | Out-Null
$Repo = Join-Path $WorkDir 'ENGG2202'
if (-not (Test-Path "$Repo\yolo_drone_training\lab.py")) {
  if (Test-Path $Repo) { throw "Choose an empty WorkDir; existing folder: $Repo" }
  if (Get-Command git -ErrorAction SilentlyContinue) {
    & git clone --single-branch --branch feature/yolo-drone-model-v1 https://github.com/danielhelloworld/ENGG2202.git $Repo
    if ($LASTEXITCODE -ne 0) {
      if (Test-Path $Repo) { Remove-Item -Recurse -Force $Repo }
      Expand-Archive (Join-Path $UsbRoot 'code\ENGG2202_source.zip') -DestinationPath $WorkDir
      Write-Host 'Using the matching USB source snapshot.'
    }
  } else {
    Expand-Archive (Join-Path $UsbRoot 'code\ENGG2202_source.zip') -DestinationPath $WorkDir
  }
}
$Project = Join-Path $Repo 'yolo_drone_training'
$Venv = Join-Path $Project '.venv'
if (-not (Test-Path "$Venv\Scripts\python.exe")) { Run-Checked -Exe $Py -Arguments @('-m','venv',$Venv) }
$Python = "$Venv\Scripts\python.exe"
Run-Checked -Exe $Python -Arguments @('-m','pip','install','--upgrade','pip')
Run-Checked -Exe $Python -Arguments @('-m','pip','install','torch==2.6.0','torchvision==0.21.0','--index-url','https://download.pytorch.org/whl/cu124')
Run-Checked -Exe $Python -Arguments @('-m','pip','install','-r',"$Project\requirements.txt")
Run-Checked -Exe $Python -Arguments @('-m','pip','check')
$Data = Join-Path $WorkDir 'dataset_v3'
Run-Checked -Exe $Python -Arguments @("$Project\lab.py",'restore','--package',$UsbRoot,'--dest',$Data)
Run-Checked -Exe $Python -Arguments @("$Project\lab.py",'check','--data',$Data,'--full')
Run-Checked -Exe $Python -Arguments @("$Project\lab.py",'doctor','--require-cuda')
Run-Checked -Exe $Python -Arguments @("$Project\lab.py",'smoke','--data',$Data,'--model',"$UsbRoot\models\v1_best.pt",'--device','0')
@{python=$Python;data=$Data;package=$UsbRoot;project=$Project} | ConvertTo-Json | Set-Content (Join-Path $WorkDir 'lab_paths.json') -Encoding UTF8
Write-Host "READY. Environment, data and GPU smoke test complete. Training has not started."
Write-Host "Project: $Project"
Write-Host "Use START_HERE.md for the baseline and formal training commands."
