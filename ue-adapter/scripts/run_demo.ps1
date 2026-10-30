$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\run_demo.ps1'
& $target @args
exit $LASTEXITCODE
