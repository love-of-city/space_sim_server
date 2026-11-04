param(
    [string]$UnrealRoot = '',
    [string]$Python = '',
    [switch]$Force
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
& (Join-Path $PSScriptRoot 'prepare_foil_material.ps1') -UnrealRoot $ue
$source = Join-Path $ProjectRoot 'ContentSource\SarmMLI'
$script = Join-Path $PSScriptRoot 'create_sarm_blanket_assets.py'
$generator = Join-Path $PSScriptRoot 'generate_sarm_blanket.py'
$assets = Join-Path $ProjectRoot 'Content\BSK\VisualOverlays\SarmMLI'
$expected = @('SM_SarmMLI', 'M_SarmMLI_UV', 'MI_SarmMliFace', 'MI_SarmMliHem', 'MI_SarmMliBacking')
$saved = Join-Path $ProjectRoot 'Saved\AssetImport'
New-Item -ItemType Directory -Path $saved -Force | Out-Null
$marker = Join-Path $saved 'sarm_mli.signature'
function Get-MliSignature {
    $files = @($script, $generator, (Join-Path $PSScriptRoot 'create_foil_material.py')) + @(Get-ChildItem -LiteralPath $source -File | Sort-Object Name | Select-Object -ExpandProperty FullName)
    return (($files | ForEach-Object { (Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash }) -join '|')
}
$signature = Get-MliSignature
$previous = if (Test-Path -LiteralPath $marker) { (Get-Content -LiteralPath $marker -Raw).Trim() } else { '' }
$missing = @($expected | Where-Object { !(Test-Path -LiteralPath (Join-Path $assets ($_ + '.uasset')) -PathType Leaf) })
if (!$Force -and $signature -eq $previous -and $missing.Count -eq 0) {
    Write-Output 'SARM independent UV blanket assets are ready.'
    return
}
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy')
& $pythonExe $generator
if ($LASTEXITCODE -ne 0) { throw 'SARM blanket source generation failed.' }
$log = Join-Path $saved 'sarm_mli_build.log'
# StaticMeshEditorSubsystem requires editor mode, not PythonScriptCommandlet.
$args = @(('"' + $ProjectFile + '"'), ('-ExecutePythonScript="' + $script + '"'), '-unattended', '-nop4', '-nosplash', '-RenderOffscreen', ('-abslog="' + $log + '"'))
$p = Start-Process -FilePath (Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe') -ArgumentList $args -WindowStyle Hidden -PassThru
$p.WaitForExit()
if ($p.ExitCode -ne 0) { throw "SARM blanket asset build failed ($($p.ExitCode)); see $log" }
$content = Get-Content -LiteralPath $log -Raw
$finalGraphLog = ($content -split 'SARM_MLI_GRAPH_READY', 2)[-1]
if ($content -notmatch 'SARM_MLI_BUILD_OK' -or $content -match 'LogPython: Error:' -or $finalGraphLog -match 'Failed to compile Material') {
    throw "SARM blanket asset validation failed; see $log"
}
foreach ($name in $expected) {
    if (!(Test-Path -LiteralPath (Join-Path $assets ($name + '.uasset')) -PathType Leaf)) { throw "Missing SARM blanket asset: $name" }
}
Set-Content -LiteralPath $marker -Value (Get-MliSignature) -Encoding ascii
Write-Output 'SARM independent UV blanket assets created and validated.'
