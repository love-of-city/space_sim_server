param(
    [string]$UnrealRoot = '',
    [switch]$Force
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$sourceDir = Join-Path $ProjectRoot 'ContentSource\Foil002'
$script = Join-Path $PSScriptRoot 'create_foil_material.py'
$assetDir = Join-Path $ProjectRoot 'Content\BSK\Materials\Foil002'
$expected = @('M_BskFoil002', 'MI_BskFoil002', 'T_Foil002_Color', 'T_Foil002_NormalDX', 'T_Foil002_Roughness', 'T_Foil002_Metalness', 'T_Foil002_AmbientOcclusion')
$missing = @($expected | Where-Object { !(Test-Path -LiteralPath (Join-Path $assetDir ($_ + '.uasset')) -PathType Leaf) })
$saved = Join-Path $ProjectRoot 'Saved\AssetImport'
New-Item -ItemType Directory -Path $saved -Force | Out-Null
$marker = Join-Path $saved 'foil002.signature'
$inputs = @($script) + @(Get-ChildItem -LiteralPath $sourceDir -File | Sort-Object Name | Select-Object -ExpandProperty FullName)
$signature = (($inputs | ForEach-Object { (Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash }) -join '|')
$old = if (Test-Path -LiteralPath $marker) { (Get-Content -LiteralPath $marker -Raw).Trim() } else { '' }
if (!$Force -and $missing.Count -eq 0 -and $old -eq $signature) {
    Write-Output 'Foil002 visual trial assets are ready.'
    return
}
$log = Join-Path $saved 'foil002_material_build.log'
$editor = Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$args = @(('"' + $ProjectFile + '"'), '-run=pythonscript', ('-script="' + $script + '"'), '-unattended', '-nop4', '-nosplash', ('-abslog="' + $log + '"'))
$p = Start-Process -FilePath $editor -ArgumentList $args -WindowStyle Hidden -PassThru
$p.WaitForExit()
if ($p.ExitCode -ne 0) { throw "Foil002 asset build failed ($($p.ExitCode)); see $log" }
if (!(Test-Path -LiteralPath $log)) { throw 'Foil002 asset build produced no verification log.' }
$content = Get-Content -LiteralPath $log -Raw
if ($content -notmatch 'FOIL002_BUILD_OK' -or $content -match 'Failed to compile Material.*M_BskFoil002' -or $content -match 'LogPython: Error:') {
    throw "Foil002 material build/compilation was not successful; see $log"
}
foreach ($name in $expected) {
    if (!(Test-Path -LiteralPath (Join-Path $assetDir ($name + '.uasset')) -PathType Leaf)) { throw "Missing generated Foil002 asset: $name" }
}
Set-Content -LiteralPath $marker -Value $signature -Encoding ascii
Write-Output 'Foil002 visual trial assets created successfully.'
