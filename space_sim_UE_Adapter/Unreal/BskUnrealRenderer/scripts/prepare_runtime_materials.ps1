param(
    [string]$UnrealRoot = '',
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
# Isolated foil assets have their own source fingerprint and rebuild cache.
& (Join-Path $PSScriptRoot 'prepare_foil_material.ps1') -UnrealRoot $ue -Force:$Force
& (Join-Path $PSScriptRoot 'prepare_sarm_blanket.ps1') -UnrealRoot $ue -Force:$Force

# Optional account-licensed PBR0110 preview; build only when selected on the arm satellite.
$gameConfig = Get-Content -LiteralPath (Join-Path $ProjectRoot 'Config\DefaultGame.ini') -Raw
if ($gameConfig -match '(?m)^/Game/BSK/Generated/SARM/base_link\.base_link=/Game/BSK/VisualOverlays/SarmMLI_PBR0110/') {
    & (Join-Path $PSScriptRoot 'prepare_pbr0110_material.ps1') -UnrealRoot $ue -Force:$Force
}

$opaque = Join-Path $ProjectRoot 'Content\BSK\M_BskPbrOpaque.uasset'
$translucent = Join-Path $ProjectRoot 'Content\BSK\M_BskPbrTranslucent.uasset'
if (!$Force -and (Test-Path -LiteralPath $opaque) -and (Test-Path -LiteralPath $translucent)) {
    Write-Output 'BSK PBR runtime materials are ready.'
    exit 0
}
$editorCmd = Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$script = Join-Path $PSScriptRoot 'create_pbr_materials.py'
Write-Output 'Creating BSK PBR runtime material assets ...'
& $editorCmd $ProjectFile -run=pythonscript "-script=$script" -unattended -nop4 -nosplash
if ($LASTEXITCODE -ne 0) { throw "PBR material creation failed with exit code $LASTEXITCODE." }
if (!(Test-Path -LiteralPath $opaque) -or !(Test-Path -LiteralPath $translucent)) {
    throw 'PBR material commandlet completed without creating both assets.'
}
Write-Output 'BSK PBR runtime materials created successfully.'
