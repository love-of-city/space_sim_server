# Authoritative capture requires matching sources, shader, engine and modules.
function Assert-BskCaptureRuntimeBuild([string]$ProjectRoot, [string]$UnrealRoot = '') {
    $engine = Resolve-UnrealRoot $UnrealRoot
    $python = Resolve-BskPython -RequiredModules @()
    $identityTool = Join-Path $ProjectRoot '../../scripts/write_ue_build_manifest.py'
    & $python $identityTool --project $ProjectRoot --engine $engine --platform Win64 --verify
    if ($LASTEXITCODE -ne 0) {
        throw 'Capture runtime identity differs from the full build. Run scripts/build.ps1, then restart the scene.'
    }
}
