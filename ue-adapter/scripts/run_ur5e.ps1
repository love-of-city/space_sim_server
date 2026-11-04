$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\run_ur5e.ps1'
& $target @args
exit $LASTEXITCODE
