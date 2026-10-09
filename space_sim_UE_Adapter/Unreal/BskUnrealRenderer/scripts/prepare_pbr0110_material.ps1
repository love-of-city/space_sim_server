param(
    [string]$UnrealRoot = '',
    [string]$Python = '',
    [string]$DerivedDataCacheGraph = 'InstalledNoZenLocalFallback',
    [int]$TimeoutSeconds = 300,
    [switch]$Force
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$source = Join-Path $ProjectRoot 'ContentSource\PBR0110'
$script = Join-Path $PSScriptRoot 'create_pbr0110_material.py'
$original = Join-Path $ProjectRoot 'Content\BSK\VisualOverlays\SarmMLI\SM_SarmMLI.uasset'
$parent = Join-Path $ProjectRoot 'Content\BSK\VisualOverlays\SarmMLI\M_SarmMLI_UV.uasset'
$expected = @('albedo','normal','roughness','metallic','ao' | ForEach-Object { 'Materials\PBR0110\T_PBR0110_' + $_ + '.uasset' })
$expected += @('SM_SarmMLI_PBR0110','MI_PBR0110_MliFace','MI_PBR0110_MliHem','MI_PBR0110_MliBacking' | ForEach-Object { 'VisualOverlays\SarmMLI_PBR0110\' + $_ + '.uasset' })
$saved = Join-Path $ProjectRoot 'Saved\AssetImport'
New-Item -ItemType Directory -Path $saved -Force | Out-Null
$geometry = Join-Path $source 'Continuous'
$generator = Join-Path $PSScriptRoot 'generate_pbr0110_blanket.py'
$recipe = Join-Path $geometry 'recipe.json'
$geometryFiles = @('SM_SarmMLI_PBR0110.obj','SM_SarmMLI_PBR0110.mtl','mesh_manifest.json' | ForEach-Object { Join-Path $geometry $_ })
$geometryMarker = Join-Path $saved 'pbr0110_geometry.signature'
$geometrySignature = ((@($generator,$recipe) | ForEach-Object { (Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash }) -join '|')
$oldGeometrySignature = if (Test-Path -LiteralPath $geometryMarker) { (Get-Content -LiteralPath $geometryMarker -Raw).Trim() } else { '' }
if ($Force -or $oldGeometrySignature -ne $geometrySignature -or @($geometryFiles | Where-Object { !(Test-Path -LiteralPath $_) }).Count -gt 0) {
    $pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy')
    & $pythonExe $generator --recipe $recipe --output $geometry
    if ($LASTEXITCODE -ne 0) { throw 'Continuous PBR0110 geometry generation failed.' }
    foreach ($path in $geometryFiles) { if (!(Test-Path -LiteralPath $path)) { throw "Missing continuous mesh source: $path" } }
    Set-Content -LiteralPath $geometryMarker -Value $geometrySignature -Encoding ascii
}
$marker = Join-Path $saved 'pbr0110.signature'
$manifestPath = Join-Path $source 'source.json'
$config = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
$inputs = @($script,$manifestPath,$original,$parent,$generator,$recipe) + $geometryFiles + @($config.textures | ForEach-Object { Join-Path $source $_.file })
foreach ($path in $inputs) {
    if (!(Test-Path -LiteralPath $path -PathType Leaf)) { throw "PBR0110 local source missing: $path. Run git lfs pull to restore the tracked PBR0110 assets." }
}
$signature = (($inputs | ForEach-Object { (Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash }) -join '|')
$old = if (Test-Path -LiteralPath $marker) { (Get-Content -LiteralPath $marker -Raw).Trim() } else { '' }
$missing = @($expected | Where-Object { !(Test-Path -LiteralPath (Join-Path (Join-Path $ProjectRoot 'Content\BSK') $_) -PathType Leaf) })
if (!$Force -and $signature -eq $old -and $missing.Count -eq 0) { Write-Output 'PBR0110 SARM material preview assets are ready.'; return }
$log = Join-Path $saved 'pbr0110_build.log'
$arguments = @(('"' + $ProjectFile + '"'), ('-ExecutePythonScript="' + $script + '"'), '-unattended','-nop4','-nosplash','-RenderOffscreen', ('-abslog="' + $log + '"'))
if ($DerivedDataCacheGraph) { $arguments += ('-ddc=' + $DerivedDataCacheGraph) }
$p = Start-Process -FilePath (Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe') -ArgumentList $arguments -WindowStyle Hidden -PassThru
try {
    if (!$p.WaitForExit($TimeoutSeconds * 1000)) { throw "PBR0110 build timed out; see $log" }
    $p.Refresh()
    if ($p.ExitCode -ne 0) { throw "PBR0110 build failed ($($p.ExitCode)); see $log" }
} finally {
    $p.Refresh()
    if (!$p.HasExited) { Stop-Process -Id $p.Id }
}
$content = Get-Content -LiteralPath $log -Raw
if ($content -notmatch 'PBR0110_BUILD_OK' -or $content -match 'LogPython: Error:|Failed to compile Material') { throw "PBR0110 build validation failed; see $log" }
foreach ($path in $expected) { if (!(Test-Path -LiteralPath (Join-Path (Join-Path $ProjectRoot 'Content\BSK') $path))) { throw "Missing PBR0110 generated asset: $path" } }
Set-Content -LiteralPath $marker -Value $signature -Encoding ascii
Write-Output 'PBR0110 SARM material preview assets created and verified.'
