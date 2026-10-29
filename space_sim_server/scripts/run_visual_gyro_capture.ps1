param(
    [string]$Python = '',
    [Parameter(Mandatory=$true)][string]$VisionPython,
    [Parameter(Mandatory=$true)][string]$UnrealRoot,
    [Parameter(Mandatory=$true)][string]$OutputDirectory,
    [ValidateRange(1024,65530)][int]$PortBase = 19100
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'python_runtime.ps1')
$runtime=Resolve-SpaceSimPython -RepositoryRoot $root -RequestedPython $Python -RequiredModules @('Basilisk.simulation.mujoco','fastapi','psutil')
$powershell=(Get-Process -Id $PID).Path
& $runtime (Join-Path $root 'tools/run_visual_gyro_capture.py') --vision-python $VisionPython `
    --unreal-root $UnrealRoot --output $OutputDirectory --port-base $PortBase --powershell $powershell
if ($LASTEXITCODE -ne 0) { throw 'Visual/gyro capture regression failed; inspect the output directory.' }
