$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\run_spacecraft_arm_grasp.ps1'
& $target @args
exit $LASTEXITCODE
