param()

$script:ProjectRoot = Split-Path -Parent $PSScriptRoot
$script:ProjectFile = Join-Path $script:ProjectRoot 'BskUnrealRenderer.uproject'
$script:WorkspaceRoot = (Resolve-Path (Join-Path $script:ProjectRoot '..\..')).Path

function Resolve-UnrealRoot {
    param([string]$RequestedRoot = '')
    $candidates = @()
    if ($RequestedRoot) { $candidates += $RequestedRoot }
    if ($env:UE56_ROOT) { $candidates += $env:UE56_ROOT }
    $candidates += 'E:\UE5.6\UE_5.6'
    $candidates += 'C:\Program Files\Epic Games\UE_5.6'
    foreach ($candidate in $candidates) {
        $editor = Join-Path $candidate 'Engine\Binaries\Win64\UnrealEditor.exe'
        if (Test-Path -LiteralPath $editor) { return (Resolve-Path $candidate).Path }
    }
    throw 'Unreal Engine 5.6 was not found. Pass -UnrealRoot or set UE56_ROOT.'
}

function Resolve-BasiliskRoot {
    param([string]$RequestedRoot = '')
    $candidates = @()
    if ($RequestedRoot) { $candidates += $RequestedRoot }
    if ($env:BSK_SOURCE_ROOT) { $candidates += $env:BSK_SOURCE_ROOT }
    $candidates += (Join-Path $script:WorkspaceRoot 'basilisk')
    $candidates += (Join-Path (Split-Path -Parent $script:WorkspaceRoot) 'basilisk')
    foreach ($candidate in $candidates) {
        $scenario = Join-Path $candidate 'examples\mujoco\scenarioMJSceneVizard.py'
        if (Test-Path -LiteralPath $scenario) { return (Resolve-Path $candidate).Path }
    }
    throw 'Basilisk source examples were not found. Pass -BasiliskRoot or set BSK_SOURCE_ROOT.'
}

function Set-BskPythonPath {
    $paths = @(
        (Join-Path $script:WorkspaceRoot 'Adapters'),
        $script:WorkspaceRoot,
        (Join-Path $script:ProjectRoot 'python')
    )
    if ($env:PYTHONPATH) { $paths += $env:PYTHONPATH }
    $env:PYTHONPATH = $paths -join [IO.Path]::PathSeparator
}

. (Join-Path $PSScriptRoot 'python_runtime.ps1')
function Resolve-BskPython {
    param([string]$RequestedPython = '', [string[]]$RequiredModules = @('numpy'))
    $selected = Resolve-SpaceSimPython -RepositoryRoot $script:WorkspaceRoot -RequestedPython $RequestedPython -RequiredModules $RequiredModules
    $env:SPACE_SIM_PYTHON = $selected
    return $selected
}
