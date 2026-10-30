$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\run_spacecraft_arm.ps1'
& $target @args
exit $LASTEXITCODE
