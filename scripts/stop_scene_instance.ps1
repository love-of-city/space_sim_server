param(
    [switch]$Quiet,
    [switch]$PreserveState,
    [string]$AdapterRoot = ''
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$statePath = Join-Path $projectRoot 'run\scene_runtime.json'

. (Join-Path $PSScriptRoot 'scene_process_helpers.ps1')

$state = $null
if (Test-Path -LiteralPath $statePath) {
    try { $state = Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json } catch { }
}
if ($state) {
    if (!$PreserveState) {
        # Mark an intentional stop before terminating children, so supervision
        # cannot mistake the requested shutdown for a renderer crash.
        $state.phase = 'stopped'
        $state.updated_at_ns = [string][DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() + '000000'
        $state.error = ''
        $temporary = "$statePath.stop.$PID"
        $state | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $temporary -Encoding utf8
        Move-Item -LiteralPath $temporary -Destination $statePath -Force
    }

    if ($state.PSObject.Properties.Name -contains 'simulation_pid') {
        $start = if ($state.PSObject.Properties.Name -contains 'simulation_start') { [long]$state.simulation_start } else { 0 }
        Stop-RecordedProcessTree ([int]$state.simulation_pid) $start
    }
    if ($state.PSObject.Properties.Name -contains 'renderer_pid') {
        $start = if ($state.PSObject.Properties.Name -contains 'renderer_start') { [long]$state.renderer_start } else { 0 }
        Stop-RecordedProcessTree ([int]$state.renderer_pid) $start
    }
    if (!$AdapterRoot -and $state.PSObject.Properties.Name -contains 'adapter_root') {
        $AdapterRoot = [string]$state.adapter_root
    }
    if ($state.PSObject.Properties.Name -contains 'launcher_pid') {
        $start = if ($state.PSObject.Properties.Name -contains 'launcher_start') { [long]$state.launcher_start } else { 0 }
        Stop-RecordedProcessTree ([int]$state.launcher_pid) $start
    }
}

if ($AdapterRoot) {
    $pidFile = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer\Saved\BskRenderer.pid'
    if (Test-Path -LiteralPath $pidFile) {
        try {
            $rendererPid = [int](Get-Content -Raw -LiteralPath $pidFile)
            Stop-RecordedProcessTree $rendererPid 0
        } catch { }
        Remove-Item -LiteralPath $pidFile -Force -ErrorAction SilentlyContinue
    }
}
if (!$Quiet) { Write-Output 'The active UE and Basilisk/MJScene scene instance has been stopped.' }
