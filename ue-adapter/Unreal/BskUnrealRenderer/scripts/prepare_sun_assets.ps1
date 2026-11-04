param(
    [string]$UnrealRoot = '',
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$textureAsset = Join-Path $ProjectRoot 'Content\Planets\Sun\8k_sun.uasset'
$materialAsset = Join-Path $ProjectRoot 'Content\Planets\Sun\M_Sun.uasset'
if (!$Force -and (Test-Path -LiteralPath $textureAsset) -and (Test-Path -LiteralPath $materialAsset)) {
    Write-Output 'Solar System Scope Sun assets are ready.'
    exit 0
}

$sourceDirectory = Join-Path $ProjectRoot 'Saved\AssetSources\SolarSystemScope'
$sourceTexture = Join-Path $sourceDirectory '8k_sun.jpg'
New-Item -ItemType Directory -Force -Path $sourceDirectory | Out-Null
Write-Output 'Downloading Solar System Scope 8k Sun texture (4096x2048) ...'
Invoke-WebRequest `
    -Uri 'https://www.solarsystemscope.com/textures/download/8k_sun.jpg' `
    -Headers @{ 'User-Agent' = 'Mozilla/5.0'; 'Referer' = 'https://www.solarsystemscope.com/textures/' } `
    -OutFile $sourceTexture

$expectedSha256 = 'F22B1CFB306DDCE72A7E3B628668A0175B745038CE6268557CB2F7F1BDF98B9D'
$actualSha256 = (Get-FileHash -LiteralPath $sourceTexture -Algorithm SHA256).Hash
if ($actualSha256 -ne $expectedSha256) {
    throw "Solar texture checksum changed. Expected $expectedSha256 but downloaded $actualSha256."
}

$editorCmd = Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$script = Join-Path $PSScriptRoot 'create_sun_assets.py'
Write-Output 'Importing Sun texture and creating M_Sun ...'
& $editorCmd $ProjectFile -run=pythonscript "-script=$script" -unattended -nop4 -nosplash
if ($LASTEXITCODE -ne 0) { throw "Sun asset creation failed with exit code $LASTEXITCODE." }
if (!(Test-Path -LiteralPath $textureAsset) -or !(Test-Path -LiteralPath $materialAsset)) {
    throw 'Sun asset commandlet completed without creating both assets.'
}
Write-Output 'Solar System Scope Sun assets created successfully.'
