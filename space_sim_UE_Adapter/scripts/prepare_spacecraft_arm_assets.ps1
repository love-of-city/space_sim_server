$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\prepare_spacecraft_arm_assets.ps1'
& $target @args
exit $LASTEXITCODE
