# Build (or verify) the native acceleration DLLs the running simulation needs.
#
# The local dynamics backend and the native posture backend both load a
# source-hash-named DLL from run/native_acceleration. A missing or stale DLL is a
# hard error at load time (simulation/native_acceleration.py), never a silent
# fallback, so a fresh checkout must be able to produce them here.
#
# MuJoCo 3.7.0 headers are vendored under third_party/mujoco/include, so no
# offline Python environment is required to build.
function Get-SpaceSimNativeComponents {
    $components = [System.Collections.Generic.List[string]]::new()
    # SPACE_SIM_NATIVE_COMPONENTS overrides the set (comma separated), e.g. for a
    # deployment that only ever runs the basilisk backend.
    $requested = $env:SPACE_SIM_NATIVE_COMPONENTS
    if ($requested) {
        foreach ($name in $requested.Split(',')) {
            $trimmed = $name.Trim()
            if ($trimmed) { $components.Add($trimmed) }
        }
        return $components
    }
    $components.Add('posture')
    $dynamics = $env:SPACE_SIM_DYNAMICS_BACKEND
    if (!$dynamics -or $dynamics.Trim().ToLowerInvariant() -ne 'basilisk') {
        $components.Add('local_mujoco_stepper')
    }
    return $components
}

function Test-SpaceSimComponentBuilt {
    param([string]$RepositoryRoot, [string]$Component)

    $source = Join-Path $RepositoryRoot "native/$Component.cpp"
    if (!(Test-Path -LiteralPath $source -PathType Leaf)) {
        throw "Native component source is missing: $source"
    }
    # Mirror simulation/native_acceleration.py exactly: the manifest records both
    # the source digest it was built from and where the DLL landed
    # (run/native_acceleration/<component>_<hash>/<component>.dll). Checking a
    # guessed path here would call a current build "stale" on every platform start.
    $manifest = Join-Path $RepositoryRoot "run/native_acceleration/$Component.json"
    if (!(Test-Path -LiteralPath $manifest -PathType Leaf)) { return $false }
    try {
        $document = Get-Content -LiteralPath $manifest -Raw -Encoding UTF8 | ConvertFrom-Json
    } catch {
        return $false
    }
    if ($document.abi -ne 1) { return $false }
    $actual = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($document.source_sha256 -ne $actual) { return $false }
    if (!$document.library) { return $false }
    $library = Join-Path $RepositoryRoot $document.library
    if (!(Test-Path -LiteralPath $library -PathType Leaf)) { return $false }
    $libraryDigest = (Get-FileHash -LiteralPath $library -Algorithm SHA256).Hash.ToLowerInvariant()
    return ($libraryDigest -eq $document.library_sha256)
}

function Ensure-SpaceSimNativeAcceleration {
    param([string]$RepositoryRoot, [string]$Python, [switch]$Rebuild)

    $components = Get-SpaceSimNativeComponents
    $missing = @()
    foreach ($component in $components) {
        if ($Rebuild -or !(Test-SpaceSimComponentBuilt -RepositoryRoot $RepositoryRoot -Component $component)) {
            $missing += $component
        }
    }
    if ($missing.Count -eq 0) { return }

    $builder = Join-Path $RepositoryRoot 'tools/build_native_acceleration.py'
    Write-Host ("构建原生加速组件：{0}" -f ($missing -join ', '))
    foreach ($component in $missing) {
        & $Python $builder --component $component
        if ($LASTEXITCODE -ne 0) {
            throw ("原生组件 $component 构建失败。需要 Visual Studio 的 MSVC x64 生成工具；" +
                   "构建失败时不会回退到其他实现。")
        }
    }
    foreach ($component in $missing) {
        if (!(Test-SpaceSimComponentBuilt -RepositoryRoot $RepositoryRoot -Component $component)) {
            throw "原生组件 $component 构建后仍不可用：run/native_acceleration"
        }
    }
}
