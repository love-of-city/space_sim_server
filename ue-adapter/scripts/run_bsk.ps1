$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\run_bsk.ps1'
& $target @args
exit $LASTEXITCODE
