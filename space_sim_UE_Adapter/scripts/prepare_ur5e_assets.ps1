$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\prepare_ur5e_assets.ps1'
& $target @args
exit $LASTEXITCODE
