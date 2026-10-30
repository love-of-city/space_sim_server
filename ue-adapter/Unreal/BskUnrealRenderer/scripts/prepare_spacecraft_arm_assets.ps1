param(
    [string]$ModelRoot = '',
    [ValidateSet('combined', 'grasp')]
    [string]$Variant = 'combined',
    [string]$UnrealRoot = '',
    [ValidateSet('auto', 'preserve', 'recompute')]
    [string]$NormalMode = 'auto',
    [ValidateRange(0.0, 180.0)]
    [double]$StlSmoothingAngle = 60.0,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')

if (!$ModelRoot) {
    $candidates = @(
        (Join-Path $WorkspaceRoot 'test\model\spacecraft_and_arm'),
        (Join-Path (Split-Path -Parent $WorkspaceRoot) 'test\model\spacecraft_and_arm')
    )
    $ModelRoot = $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (!$ModelRoot -or !(Test-Path -LiteralPath $ModelRoot)) {
    throw 'spacecraft_and_arm model was not found. Pass -ModelRoot explicitly.'
}
$resolvedRoot = (Resolve-Path -LiteralPath $ModelRoot).Path
if ($Variant -eq 'grasp') {
    $mjcf = Join-Path $resolvedRoot 'assets\cubesat_so101_grasp\cubesat_so101_grasp.xml'
    $destination = '/Game/BSK/Generated/CubeSatSO101Grasp'
    $catalogName = 'cubesat_so101_grasp.catalog.json'
} else {
    $mjcf = Join-Path $resolvedRoot 'assets\cubesat_so101\cubesat_so101.xml'
    $destination = '/Game/BSK/Generated/CubeSatSO101'
    $catalogName = 'cubesat_so101.catalog.json'
}
if (!(Test-Path -LiteralPath $mjcf)) { throw "MJCF file does not exist: $mjcf" }
$catalog = Join-Path $ProjectRoot "Saved\AssetImport\$catalogName"

& (Join-Path $PSScriptRoot 'prepare_mjcf_assets.ps1') -MjcfPath $mjcf `
    -Destination $destination -CatalogPath $catalog -UnrealRoot $UnrealRoot `
    -NormalMode $NormalMode -StlSmoothingAngle $StlSmoothingAngle -Force:$Force
if ($LASTEXITCODE -ne 0) { throw 'CubeSat + SO-101 STL asset preparation failed.' }
Write-Output "CubeSat + SO-101 asset catalog: $catalog"
