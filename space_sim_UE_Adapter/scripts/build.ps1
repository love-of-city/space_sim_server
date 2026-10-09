$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\build.ps1'
& $target @args
exit $LASTEXITCODE
