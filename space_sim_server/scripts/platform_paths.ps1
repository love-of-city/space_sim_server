# Side-effect-free discovery for a normal checkout or an extra enclosing folder.
function Resolve-PlatformAdapterRoot([string]$RequestedRoot, [string]$ProjectRoot) {
    if ($RequestedRoot) {
        $root = [IO.Path]::GetFullPath($RequestedRoot, $ProjectRoot)
        if (!(Test-Path -LiteralPath (Join-Path $root 'Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject') -PathType Leaf)) {
            throw "The configured adapter is not a UE adapter checkout: $root"
        }
        return $root
    }
    # Keep the dataset worktree paired with its UE worktree, never silently use
    # the original main renderer binary for the new capture protocol.
    if ((Split-Path -Leaf $ProjectRoot) -eq 'space_sim_server_lerobot_v3') {
        $datasetAdapter = Join-Path (Split-Path -Parent $ProjectRoot) 'space_sim_UE_adapter_lerobot_v3'
        if (Test-Path -LiteralPath (Join-Path $datasetAdapter 'Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject')) {
            return $datasetAdapter
        }
        throw 'Paired LeRobot UE worktree is missing; set AdapterRoot explicitly.'
    }
    $workspace = Split-Path -Parent $ProjectRoot
    $parent = Split-Path -Parent $workspace
    $candidates = @()
    foreach ($base in @($workspace, $parent) | Where-Object { $_ } | Select-Object -Unique) {
        $sibling = Join-Path $base 'space_sim_UE_adapter'
        $candidates += $sibling
        $candidates += Join-Path $sibling 'space_sim_UE_Adapter'
    }
    foreach ($candidate in $candidates | Select-Object -Unique) {
        if (Test-Path -LiteralPath (Join-Path $candidate 'Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject') -PathType Leaf) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw 'UE adapter checkout was not found beside the server checkout or its enclosing folder. Place space_sim_UE_adapter there, or set adapter_root in the deployment configuration.'
}
