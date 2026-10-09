$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\test_demo8.ps1'
& $target @args
exit $LASTEXITCODE
