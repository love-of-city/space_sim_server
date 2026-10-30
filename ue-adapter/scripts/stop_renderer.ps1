$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\stop_renderer.ps1'
& $target @args
exit $LASTEXITCODE
