$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\package.ps1'
& $target @args
exit $LASTEXITCODE
