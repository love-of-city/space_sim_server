# Keep this resolver identical in both independently cloned repositories.
function Resolve-SpaceSimPython {
    param([string]$RepositoryRoot, [string]$RequestedPython = '', [string[]]$RequiredModules = @())
    $candidate = if ($RequestedPython) { $RequestedPython } else { $env:SPACE_SIM_PYTHON }
    if (!$candidate -and $env:VIRTUAL_ENV) {
        $candidate = [IO.Path]::Combine($env:VIRTUAL_ENV, 'Scripts', 'python.exe')
    }
    if (!$candidate -and $env:CONDA_PREFIX) {
        $candidate = [IO.Path]::Combine($env:CONDA_PREFIX, 'python.exe')
    }
    if (!$candidate) {
        foreach ($root in @($RepositoryRoot, (Split-Path -Parent $RepositoryRoot))) {
            foreach ($name in @('.venv', 'venv')) {
                $local = [IO.Path]::Combine($root, $name, 'Scripts', 'python.exe')
                if (Test-Path -LiteralPath $local -PathType Leaf) { $candidate = $local; break }
            }
            if ($candidate) { break }
        }
    }
    if (!$candidate) {
        $command = Get-Command python -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($command) { $candidate = $command.Source }
    }
    if (!$candidate -or !(Test-Path -LiteralPath $candidate -PathType Leaf)) {
        throw "Selected Python does not exist: '$candidate'. Activate an environment or set SPACE_SIM_PYTHON to python.exe; no fallback was attempted."
    }
    $candidate = (Resolve-Path -LiteralPath $candidate).Path
    & $candidate -c 'import importlib,sys; [importlib.import_module(m) for m in sys.argv[1:]]' @RequiredModules
    if ($LASTEXITCODE -ne 0) {
        throw "Python preflight failed: $candidate. Required modules: $($RequiredModules -join ', '). Install dependencies in this environment; no fallback was attempted."
    }
    return $candidate
}
