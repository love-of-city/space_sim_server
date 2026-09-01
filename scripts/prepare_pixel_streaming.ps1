param([string]$UnrealRoot = 'E:\UE5.6')

$ErrorActionPreference = 'Stop'

function Convert-ToShortWindowsPath([string]$Path) {
    # Epic's downloader uses unquoted %~dp0-derived paths and fails below paths such as "Program Files".
    $shortPath = (& $env:ComSpec /d /c "for %I in (`"$Path`") do @echo %~sI" | Select-Object -First 1).Trim()
    if (!$shortPath) { throw "Unable to resolve a short Windows path for: $Path" }
    return $shortPath
}

function Resolve-UeRoot([string]$RequestedRoot) {
    foreach ($candidate in @($RequestedRoot, ([IO.Path]::Combine($RequestedRoot, 'UE_5.6')), 'E:\UE5.6\UE_5.6')) {
        if (!$candidate) { continue }
        $editor = [IO.Path]::Combine($candidate, 'Engine', 'Binaries', 'Win64', 'UnrealEditor.exe')
        if (Test-Path -LiteralPath $editor -PathType Leaf) { return (Resolve-Path -LiteralPath $candidate).Path }
    }
    throw "Unreal Engine 5.6 was not found below $RequestedRoot"
}

$ue = Resolve-UeRoot $UnrealRoot
$webServers = Join-Path $ue 'Engine\Plugins\Media\PixelStreaming2\Resources\WebServers'
$download = Join-Path $webServers 'get_ps_servers.bat'
$server = Join-Path $webServers 'SignallingWebServer'
if (!(Test-Path -LiteralPath (Join-Path $server 'package.json') -PathType Leaf)) {
    if (!(Test-Path -LiteralPath $download -PathType Leaf)) { throw 'The UE PixelStreaming2 server downloader is missing.' }

    # Use the short path only for Epic's batch downloader. The infrastructure itself is built with system npm.
    $webServersForBatch = Join-Path (Convert-ToShortWindowsPath $ue) 'Engine\Plugins\Media\PixelStreaming2\Resources\WebServers'
    & (Join-Path $webServersForBatch 'get_ps_servers.bat') /v 5.6
    if ($LASTEXITCODE -ne 0) { throw 'Epic Pixel Streaming Infrastructure download failed.' }
}

$dist = Join-Path $server 'dist\index.js'
$player = Join-Path $server 'www\player.html'
if (!(Test-Path -LiteralPath $dist -PathType Leaf) -or !(Test-Path -LiteralPath $player -PathType Leaf)) {
    $npmCommand = Get-Command npm.cmd -ErrorAction SilentlyContinue
    if (!$npmCommand) { throw 'System Node.js/npm was not found. Install Node.js and make npm.cmd available on PATH.' }

    Push-Location $webServers
    try {
        & $npmCommand.Source install --no-audit --no-fund
        if ($LASTEXITCODE -ne 0) { throw 'Pixel Streaming Infrastructure dependency installation failed.' }

        & $npmCommand.Source run build:all:cjs
        if ($LASTEXITCODE -ne 0) { throw 'Pixel Streaming Infrastructure build failed.' }
    } finally {
        Pop-Location
    }
}
foreach ($path in @($dist, $player)) {
    if (!(Test-Path -LiteralPath $path -PathType Leaf)) { throw "Pixel Streaming prerequisite is missing: $path" }
}

$frontend = Join-Path (Split-Path -Parent $PSScriptRoot) 'frontend'
$frontendPackage = Join-Path $frontend 'package.json'
if (Test-Path -LiteralPath $frontendPackage -PathType Leaf) {
    $npmCommand = Get-Command npm.cmd -ErrorAction SilentlyContinue
    if (!$npmCommand) { throw 'System Node.js/npm was not found. Install Node.js and make npm.cmd available on PATH.' }

    Push-Location $frontend
    try {
        if (!(Test-Path -LiteralPath (Join-Path $frontend 'node_modules') -PathType Container)) {
            & $npmCommand.Source install --no-audit --no-fund
            if ($LASTEXITCODE -ne 0) { throw 'Operator console npm dependency installation failed.' }
        }
        & $npmCommand.Source run build
        if ($LASTEXITCODE -ne 0) { throw 'Operator console Pixel Streaming SDK build failed.' }
    } finally {
        Pop-Location
    }
}

Write-Output "Pixel Streaming 2 infrastructure is ready: $server"
