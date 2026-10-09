param(
    [string]$UnrealRoot = '',
    [string]$ListenAddress = '127.0.0.1',
    [int]$Port = 5558,
    [int]$Width = 1280,
    [int]$Height = 720,
    [string]$ReplayPath = '',
    [double]$ReplayRate = 1.0,
    [string]$ScreenshotPath = '',
    [string]$CaptureDirectory = '',
    [string[]]$CaptureProducts = @(),
    [ValidateRange(0.0, 60.0)]
    [double]$CaptureRate = 0.0,
    [ValidateRange(0.0, 60.0)]
    [double]$PreviewRate = 0.0,
    [string]$PixelStreamingURL = '',
    [string]$PixelStreamingId = 'BskRenderer',
    [ValidateRange(1, 120)]
    [int]$PixelStreamingFps = 90,
    [ValidateRange(0, 100)]
    [int]$EncoderMinQuality = 60,
    [string[]]$PixelStreamingCameraIds = @(),
    [ValidateRange(160, 1920)]
    [int]$PixelStreamingCameraWidth = 640,
    [ValidateRange(90, 1080)]
    [int]$PixelStreamingCameraHeight = 360,
    [ValidateRange(1, 120)]
    [int]$PixelStreamingCameraFps = 90,
    [string]$CaptureNetworkHost = '127.0.0.1',
    [ValidateRange(0, 65535)]
    [int]$CaptureNetworkPort = 0,
    [string]$AutoCommand = '',
    [string]$DerivedDataCachePath = '',
    [switch]$Foreground
)

. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$editor = Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor.exe'
# The installed graph normally relies on Zen and marks its filesystem node
# DeleteOnly. Our project config makes Local writable as a persistent fallback.
# Keep Zen enabled so existing cached shaders/assets do not need recompilation.
if (!$DerivedDataCachePath) { $DerivedDataCachePath = Join-Path $ProjectRoot 'Saved\DerivedDataCache' }
$DerivedDataCachePath = [IO.Path]::GetFullPath($DerivedDataCachePath)
New-Item -ItemType Directory -Path $DerivedDataCachePath -Force | Out-Null
$probe = Join-Path $DerivedDataCachePath ('.write-probe-' + [Guid]::NewGuid().ToString('N'))
try {
    [IO.File]::WriteAllText($probe, 'writable')
} catch {
    throw "UE derived-data cache is not writable: $DerivedDataCachePath. $($_.Exception.Message)"
} finally {
    if (Test-Path -LiteralPath $probe) { Remove-Item -LiteralPath $probe -Force }
}
$arguments = @(
    $ProjectFile, '-game', '-windowed', "-ResX=$Width", "-ResY=$Height",
    "-BskListen=$ListenAddress", "-BskPort=$Port", '-log', '-DDC=InstalledDerivedDataBackendGraph'
)
$normalizedCaptureProducts = @()
foreach ($item in $CaptureProducts) {
    foreach ($product in ($item -split ',')) {
        if ($product.Trim()) { $normalizedCaptureProducts += $product.Trim().ToLowerInvariant() }
    }
}
$unsupportedCaptureProducts = @($normalizedCaptureProducts | Where-Object { $_ -notin @('rgb') })
if ($unsupportedCaptureProducts.Count -gt 0) {
    throw "Unsupported capture products: $($unsupportedCaptureProducts -join ', ')"
}
# Preview startup remains unchanged; authoritative RGB capture requires a fresh DLL.
if ($normalizedCaptureProducts.Count -gt 0 -or $CaptureNetworkPort -gt 0 -or $CaptureDirectory) {
    . (Join-Path $PSScriptRoot 'runtime_build.ps1')
    Assert-BskCaptureRuntimeBuild $ProjectRoot
    if ($CaptureRate -gt 0 -and $CaptureRate -notin @(1, 2, 5, 10, 30)) {
        throw 'LeRobot capture rate must be 1, 2, 5, 10 or 30 Hz.'
    }
}
if ($ReplayPath) {
    $resolvedReplay = [IO.Path]::GetFullPath($ReplayPath)
    $arguments += @("-BskReplay=$resolvedReplay", "-BskReplayRate=$ReplayRate")
}
if ($ScreenshotPath) {
    $arguments += "-BskScreenshot=$([IO.Path]::GetFullPath($ScreenshotPath))"
}
if ($CaptureDirectory) {
    $arguments += "-BskCaptureDir=$([IO.Path]::GetFullPath($CaptureDirectory))"
}
if ($normalizedCaptureProducts.Count -gt 0) {
    # Unreal's FParse::Value treats commas as token delimiters. Use '+' on the
    # command line; the runtime normalizes it back into a product list.
    $arguments += "-BskCaptureProducts=$($normalizedCaptureProducts -join '+')"
}
if ($CaptureRate -gt 0.0) {
    $arguments += "-BskCaptureRate=$CaptureRate"
}
if ($PreviewRate -gt 0.0) {
    $arguments += "-BskPreviewRate=$PreviewRate"
}
if ($PixelStreamingURL) {
    $arguments += @(
        # Pixel Streaming must keep rendering even when no local window is focused/minimized.
        # ForceRes keeps the requested back-buffer size in off-screen mode.
        '-RenderOffscreen',
        '-ForceRes',
        # Stay on the native GPU-copy/fence path. UE 5.6 MediaCapture previously
        # exhausted its output buffer pool during long-running camera streams.
        '-PixelStreamingUseMediaCapture=false',
        # Do not inflate FPS by re-sending old frames on a separate timer.
        '-PixelStreamingDecoupleFramerate=false',
        ('-ExecCmds="t.MaxFPS ' + $PixelStreamingFps + ',r.VSync 0"'),
        "-PixelStreamingConnectionURL=$PixelStreamingURL",
        "-PixelStreamingID=$PixelStreamingId",
        "-PixelStreamingWebRTCFps=$PixelStreamingFps",
        '-PixelStreamingEncoderCodec=H264',
        # Limit compression quality loss without overriding WebRTC's adaptive bitrate.
        "-PixelStreamingEncoderMinQuality=$EncoderMinQuality",
        '-PixelStreamingEncoderLatencyMode=UltraLowLatency',
        '-PixelStreamingWebRTCDisableTransmitAudio=true',
        '-PixelStreamingWebRTCDisableReceiveAudio=true'
    )
    if ($PixelStreamingCameraIds.Count -gt 0) {
        $arguments += @(
            "-BskPixelStreamingURL=$PixelStreamingURL",
            "-BskPixelStreamingBaseId=$PixelStreamingId",
            "-BskPixelStreamingCameras=$($PixelStreamingCameraIds -join '+')",
            "-BskPixelStreamingCameraWidth=$PixelStreamingCameraWidth",
            "-BskPixelStreamingCameraHeight=$PixelStreamingCameraHeight",
            "-BskPixelStreamingCameraFps=$PixelStreamingCameraFps"
        )
    }
}
if ($CaptureNetworkPort -gt 0) {
    $arguments += @("-BskCaptureHost=$CaptureNetworkHost", "-BskCapturePort=$CaptureNetworkPort")
}
if ($AutoCommand) {
    $arguments += "-BskAutoCommand=$AutoCommand"
}
if ($Foreground) {
    $previousCachePath = [Environment]::GetEnvironmentVariable('UE-LocalDataCachePath', 'Process')
    try {
        [Environment]::SetEnvironmentVariable('UE-LocalDataCachePath', $DerivedDataCachePath, 'Process')
        & $editor @arguments
        $result = $LASTEXITCODE
    } finally {
        [Environment]::SetEnvironmentVariable('UE-LocalDataCachePath', $previousCachePath, 'Process')
    }
    exit $result
}
$saved = Join-Path $ProjectRoot 'Saved'
New-Item -ItemType Directory -Path $saved -Force | Out-Null
$process = Start-Process -FilePath $editor -ArgumentList $arguments -PassThru -WindowStyle Hidden `
    -Environment @{'UE-LocalDataCachePath' = $DerivedDataCachePath}
Set-Content -LiteralPath (Join-Path $saved 'BskRenderer.pid') -Value $process.Id -Encoding ascii
if ($ReplayPath) {
    Write-Output "BSK Unreal Renderer started (PID $($process.Id)), replaying $resolvedReplay at ${ReplayRate}x"
} else {
    Write-Output "BSK Unreal Renderer started (PID $($process.Id)), listening on $ListenAddress`:$Port"
}
