$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\run_mock.ps1'
& $target @args
exit $LASTEXITCODE
