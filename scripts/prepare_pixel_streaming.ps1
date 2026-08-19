param([string]$UnrealRoot = 'E:\UE5.6')

$ErrorActionPreference = 'Stop'

function Resolve-UeRoot([string]$RequestedRoot) {
    foreach ($candidate in @($RequestedRoot, (Join-Path $RequestedRoot 'UE_5.6'), 'E:\UE5.6\UE_5.6')) {
        if (!$candidate) { continue }
        $editor = Join-Path $candidate 'Engine\Binaries\Win64\UnrealEditor.exe'
        if (Test-Path -LiteralPath $editor) { return (Resolve-Path -LiteralPath $candidate).Path }
    }
    throw "Unreal Engine 5.6 was not found below $RequestedRoot"
}

$ue = Resolve-UeRoot $UnrealRoot
$webServers = Join-Path $ue 'Engine\Plugins\Media\PixelStreaming2\Resources\WebServers'
$download = Join-Path $webServers 'get_ps_servers.bat'
$server = Join-Path $webServers 'SignallingWebServer'
if (!(Test-Path -LiteralPath (Join-Path $server 'package.json'))) {
    if (!(Test-Path -LiteralPath $download)) { throw 'The UE PixelStreaming2 server downloader is missing.' }
    & $download /v 5.6
    if ($LASTEXITCODE -ne 0) { throw 'Epic Pixel Streaming Infrastructure download failed.' }
}

$setup = Join-Path $server 'platform_scripts\cmd\setup.bat'
$node = Join-Path $server 'platform_scripts\cmd\node\node.exe'
$npm = Join-Path $server 'platform_scripts\cmd\node\npm.cmd'
$dist = Join-Path $server 'dist\index.js'
$player = Join-Path $server 'www\player.html'
if (!(Test-Path -LiteralPath $node) -or !(Test-Path -LiteralPath $player)) {
    & $setup
    if ($LASTEXITCODE -ne 0) { throw 'Pixel Streaming Node/frontend preparation failed.' }
}
if (!(Test-Path -LiteralPath $dist)) {
    & $npm run build
    if ($LASTEXITCODE -ne 0) { throw 'Pixel Streaming signalling server build failed.' }
}
foreach ($path in @($node, $dist, $player)) {
    if (!(Test-Path -LiteralPath $path)) { throw "Pixel Streaming prerequisite is missing: $path" }
}

$frontend = Join-Path (Split-Path -Parent $PSScriptRoot) 'frontend'
$frontendPackage = Join-Path $frontend 'package.json'
if (Test-Path -LiteralPath $frontendPackage) {
    Push-Location $frontend
    try {
        if (!(Test-Path -LiteralPath (Join-Path $frontend 'node_modules'))) {
            npm.cmd install --no-audit --no-fund
            if ($LASTEXITCODE -ne 0) { throw 'Operator console npm dependency installation failed.' }
        }
        npm.cmd run build
        if ($LASTEXITCODE -ne 0) { throw 'Operator console Pixel Streaming SDK build failed.' }
    } finally {
        Pop-Location
    }
}

Write-Output "Pixel Streaming 2 infrastructure is ready: $server"
