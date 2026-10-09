$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\test.ps1'
& $target @args
exit $LASTEXITCODE
