$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\run_orbital_grasp.ps1'
& $target @args
exit $LASTEXITCODE
