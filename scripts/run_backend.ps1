param(
    [int]$ApiPort = 8000,
    [int]$ControlPort = 8766,
    [int]$CapturePort = 8767,
    [string]$DataRoot = ''
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if (!$DataRoot) { $DataRoot = Join-Path $projectRoot 'data\episodes' }
$env:PYTHONPATH = Join-Path $projectRoot 'backend'
Set-Location $projectRoot
python -m space_arm_platform.main --port $ApiPort --simulation-port $ControlPort `
    --capture-port $CapturePort --data-root ([IO.Path]::GetFullPath($DataRoot))

