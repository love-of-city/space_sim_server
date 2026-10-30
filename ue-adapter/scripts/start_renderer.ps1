$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\start_renderer.ps1'
& $target @args
exit $LASTEXITCODE
