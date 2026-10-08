# The offline worker must not depend on an old terminal inheriting new user variables.
# This selection is separate from the authoritative Basilisk/backend interpreter.
function Resolve-SpaceSimPosturePython {
    param([string]$RepositoryRoot, [switch]$ValidateRuntime)

    $candidate = $env:SPACE_SIM_POSTURE_PYTHON
    $configPath = Join-Path $RepositoryRoot '.space-sim-posture-python'
    if (!$candidate -and (Test-Path -LiteralPath $configPath -PathType Leaf)) {
        $candidate = [IO.File]::ReadAllText($configPath, [Text.Encoding]::UTF8).Trim()
        if (!$candidate) {
            throw "Offline posture Python configuration is empty: $configPath. No fallback was attempted."
        }
    }
    # Preserve the existing backend-interpreter fallback for installations that
    # have not selected a separate worker environment.
    if (!$candidate) { return }

    $candidate = [Environment]::ExpandEnvironmentVariables($candidate.Trim().Trim('"'))
    $candidate = [IO.Path]::GetFullPath($candidate, [IO.Path]::GetFullPath($RepositoryRoot))
    if (!(Test-Path -LiteralPath $candidate -PathType Leaf)) {
        throw "Offline posture Python does not exist: $candidate. No fallback was attempted."
    }
    if ($ValidateRuntime) {
        # Import Python MuJoCo only in this independent subprocess, never in
        # the backend or in the native Basilisk simulation process.
        try {
            $output = & $candidate -c 'import numpy, mujoco; assert mujoco.__version__ == "3.7.0", "expected MuJoCo 3.7.0, got " + mujoco.__version__' 2>&1
            if ($LASTEXITCODE -ne 0) { throw ($output -join [Environment]::NewLine) }
        } catch {
            throw "Offline posture Python preflight failed: $candidate. Requires numpy and mujoco==3.7.0; no fallback was attempted. $($_.Exception.Message)"
        }
    }
    return $candidate
}
