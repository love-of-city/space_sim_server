$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\receive_camera_products.ps1'
& $target @args
exit $LASTEXITCODE
