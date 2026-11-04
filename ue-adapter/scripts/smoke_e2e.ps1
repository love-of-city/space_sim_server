$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\smoke_e2e.ps1'
& $target @args
exit $LASTEXITCODE
