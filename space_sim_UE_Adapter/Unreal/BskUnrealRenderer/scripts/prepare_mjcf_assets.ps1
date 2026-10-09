param(
    [string]$Python = '',
    [Parameter(Mandatory=$true)]
    [string]$MjcfPath,
    [Parameter(Mandatory=$true)]
    [string]$Destination,
    [string]$CatalogPath = '',
    [double]$BuildScale = 100.0,
    [double]$ComponentScale = 1.0,
    [string]$UnrealRoot = '',
    [ValidateSet('auto', 'preserve', 'recompute')]
    [string]$NormalMode = 'auto',
    [ValidateRange(0.0, 180.0)]
    [double]$StlSmoothingAngle = 60.0,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Set-BskPythonPath
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy')
$ue = Resolve-UnrealRoot $UnrealRoot
$resolvedMjcf = [IO.Path]::GetFullPath($MjcfPath)
if (!(Test-Path -LiteralPath $resolvedMjcf)) { throw "MJCF file does not exist: $resolvedMjcf" }
if (!$Destination.StartsWith('/Game/')) { throw '-Destination must start with /Game/.' }
if (!$CatalogPath) {
    $safeName = [IO.Path]::GetFileNameWithoutExtension($resolvedMjcf) -replace '[^A-Za-z0-9_-]', '_'
    $CatalogPath = Join-Path $ProjectRoot "Config\BskAssets\$safeName.json"
}
$resolvedCatalog = [IO.Path]::GetFullPath($CatalogPath)
$safeImportName = ([IO.Path]::GetFileNameWithoutExtension($resolvedCatalog) -replace '[^A-Za-z0-9_-]', '_')
$importSettings = Join-Path $ProjectRoot "Saved\AssetImport\$safeImportName.import.json"
$meshCache = Join-Path $ProjectRoot "Saved\AssetImport\${safeImportName}_mesh_cache"
$importSignatureMarker = Join-Path $ProjectRoot "Saved\AssetImport\$safeImportName.import_signature"
$meshSettingsMarker = Join-Path $ProjectRoot "Saved\AssetImport\$safeImportName.mesh_settings"
$meshSettingsSignature = "$NormalMode|stlAngle=$StlSmoothingAngle|build=$BuildScale|component=$ComponentScale|nanite=off|lod0=full|v=3"

& $pythonExe (Join-Path $PSScriptRoot 'generate_mjcf_asset_catalog.py') `
    --mjcf $resolvedMjcf --destination $Destination --catalog $resolvedCatalog --import-settings $importSettings `
    --mesh-cache $meshCache --normal-mode $NormalMode --stl-smoothing-angle $StlSmoothingAngle `
    --build-scale $BuildScale --component-scale $ComponentScale
if ($LASTEXITCODE -ne 0) { throw 'Failed to generate the MJCF asset catalog.' }

$catalogData = Get-Content -Raw -LiteralPath $resolvedCatalog | ConvertFrom-Json
$expectedMeshes = @($catalogData.assets.PSObject.Properties).Count
$expectedTextures = if ($catalogData.textures) { @($catalogData.textures.PSObject.Properties).Count } else { 0 }
$expectedAssets = $expectedMeshes + $expectedTextures
$expectedObjectPaths = @($catalogData.assets.PSObject.Properties | ForEach-Object { $_.Value.asset_path })
if ($catalogData.textures) {
    $expectedObjectPaths += @($catalogData.textures.PSObject.Properties | ForEach-Object { $_.Value })
}
$expectedAssetFiles = @($expectedObjectPaths | ForEach-Object {
    $packagePath = ([string]$_).Split('.', 2)[0]
    if (!$packagePath.StartsWith('/Game/')) { throw "Catalog asset path must start with /Game/: $_" }
    Join-Path (Join-Path $ProjectRoot 'Content') ($packagePath.Substring('/Game/'.Length).Replace('/', '\') + '.uasset')
})
$missingAssetFiles = @($expectedAssetFiles | Where-Object { !(Test-Path -LiteralPath $_) })
$existingAssets = $expectedAssets - $missingAssetFiles.Count
$catalogHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $resolvedCatalog).Hash
$importSignature = "$catalogHash|destination=$Destination|v=2"
$configuredImportSignature = if (Test-Path -LiteralPath $importSignatureMarker) { (Get-Content -Raw -LiteralPath $importSignatureMarker).Trim() } else { '' }
$editorCmd = Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$needsImport = $Force -or $missingAssetFiles.Count -gt 0 -or $configuredImportSignature -ne $importSignature
if ($needsImport) {
    $stlMeshes = @($catalogData.assets.PSObject.Properties | Where-Object { $_.Value.source_format -eq 'stl' }).Count
    Write-Output "Importing $expectedMeshes MJCF meshes ($stlMeshes STL) and $expectedTextures textures into $Destination ..."
    & $editorCmd $ProjectFile -run=ImportAssets "-importsettings=$importSettings" "-dest=$Destination" -replaceexisting -nosourcecontrol -unattended -nop4 -nosplash
    $importExitCode = $LASTEXITCODE
    $missingAssetFiles = @($expectedAssetFiles | Where-Object { !(Test-Path -LiteralPath $_) })
    $existingAssets = $expectedAssets - $missingAssetFiles.Count
    if ($importExitCode -ne 0 -or $missingAssetFiles.Count -gt 0) {
        $firstMissing = if ($missingAssetFiles.Count) { $missingAssetFiles[0] } else { '<none>' }
        throw "UE asset import exited with $importExitCode; expected $expectedAssets assets, found $existingAssets. First missing: $firstMissing"
    }
    Set-Content -LiteralPath $importSignatureMarker -Value $importSignature -Encoding ascii
} else {
    Write-Output "MJCF assets already imported and source fingerprints match ($existingAssets .uasset files); applying normal mode '$NormalMode'."
}
$configuredSignature = if (Test-Path -LiteralPath $meshSettingsMarker) { (Get-Content -Raw -LiteralPath $meshSettingsMarker).Trim() } else { '' }
if ($needsImport -or $configuredSignature -ne $meshSettingsSignature) {
    $env:BSK_MJCF_ASSET_CATALOG = $resolvedCatalog
    $env:BSK_MJCF_NORMAL_MODE = $NormalMode
    try {
        $meshConfigScript = Join-Path $PSScriptRoot 'configure_imported_meshes.py'
        # StaticMeshEditorSubsystem rejects changes from PythonScriptCommandlet.
        # Execute in a headless editor session so LOD build settings are really
        # written, rebuilt, and saved rather than only logged as configured.
        & $editorCmd $ProjectFile "-ExecutePythonScript=$meshConfigScript" -nullrhi -unattended -nop4 -nosplash
        if ($LASTEXITCODE -ne 0) { throw "MJCF mesh build-setting configuration failed with exit code $LASTEXITCODE." }
        Set-Content -LiteralPath $meshSettingsMarker -Value $meshSettingsSignature -Encoding ascii
    } finally {
        Remove-Item Env:\BSK_MJCF_ASSET_CATALOG -ErrorAction SilentlyContinue
        Remove-Item Env:\BSK_MJCF_NORMAL_MODE -ErrorAction SilentlyContinue
    }
} else {
    Write-Output "MJCF mesh settings are already applied: $meshSettingsSignature"
}
Write-Output "MJCF assets imported successfully ($existingAssets .uasset files): $resolvedCatalog"
