$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\prepare_mjcf_assets.ps1'
& $target @args
exit $LASTEXITCODE
