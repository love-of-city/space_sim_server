# Dataset capture must never silently use a pre-change DLL after a -NoLink check.
function Assert-BskCaptureRuntimeBuild([string]$ProjectRoot) {
    $pluginRoot = Join-Path $ProjectRoot 'Plugins/BskUnrealRuntime'
    $binaryPath = Join-Path $pluginRoot 'Binaries/Win64/UnrealEditor-BskUnrealRuntime.dll'
    if (!(Test-Path -LiteralPath $binaryPath -PathType Leaf)) {
        throw "Capture runtime DLL is missing: $binaryPath. Run scripts/build.ps1 (full build, without -NoLink) before starting data collection."
    }
    $sourceRoot = Join-Path $pluginRoot 'Source'
    if (!(Test-Path -LiteralPath $sourceRoot -PathType Container)) {
        throw "Cannot verify capture runtime sources: $sourceRoot"
    }
    $binary = Get-Item -LiteralPath $binaryPath
    $newer = Get-ChildItem -LiteralPath $sourceRoot -Recurse -File |
        Where-Object { $_.Extension -in @('.cpp', '.h', '.cs') -and $_.LastWriteTimeUtc -gt $binary.LastWriteTimeUtc } |
        Sort-Object LastWriteTimeUtc -Descending | Select-Object -First 1
    if ($newer) {
        throw "Capture runtime DLL is stale: $($newer.FullName) is newer than $binaryPath. Run scripts/build.ps1 (full build, without -NoLink), then restart the scene. A compile-only check does not update the running plugin."
    }
}
