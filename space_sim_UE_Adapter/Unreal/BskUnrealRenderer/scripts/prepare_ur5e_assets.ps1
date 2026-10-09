param(
    [string]$UnrealRoot = '',
    [ValidateSet('auto', 'preserve', 'recompute')]
    [string]$NormalMode = 'auto',
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$mjcf = Join-Path $WorkspaceRoot 'test\model\arm\universal_robots_ur5e\scene.xml'
$catalog = Join-Path $ProjectRoot 'Config\BskAssets\ur5e.json'
& (Join-Path $PSScriptRoot 'prepare_mjcf_assets.ps1') -MjcfPath $mjcf `
    -Destination '/Game/BSK/Generated/UR5e' -CatalogPath $catalog -UnrealRoot $UnrealRoot -NormalMode $NormalMode -Force:$Force
