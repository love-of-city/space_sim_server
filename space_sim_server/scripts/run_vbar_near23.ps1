param(
    [Parameter(Mandatory=$true)][string]$BasiliskRoot,
    [Parameter(Mandatory=$true)][string]$ReferenceServer,
    [Parameter(Mandatory=$true)][string]$ReferenceScene,
    [Parameter(Mandatory=$true)][string]$Output,
    [string]$Python = 'python',
    [double]$InitialDepthM = 110,
    [double]$DurationS = 2600,
    [int]$Seed = 7
)
$ErrorActionPreference = 'Stop'
$serverRoot = Split-Path $PSScriptRoot -Parent
Push-Location $serverRoot
try {
    # This selected integration trial explicitly permits healthy GNSS beyond
    # the legacy 12 m boundary. It does not edit the default web mission.
    & $Python tools/diagnostics/validate_gnss_proximity_dynamics.py `
        --basilisk-root $BasiliskRoot --source-server $ReferenceServer `
        --source-scene $ReferenceScene --scene-model model/SARM/platform/sarm_dual_capture26.xml `
        --camera-config config/vision/capture26_near23_front_mount_20261029.json `
        --mode vbar_original --depth-m $InitialDepthM --duration-s $DurationS --seed $Seed `
        --gnss-bridge-diagnostic --kinematic-origin-prediction --image-channel wide `
        --image-position-sigma-m .006 --with-grasp --output $Output
    if ($LASTEXITCODE -ne 0) { throw "Native integration trial failed (exit $LASTEXITCODE); inspect $Output" }
    & $Python tools/diagnostics/score_vbar_near23.py $Output
    if ($LASTEXITCODE -ne 0) { throw "Physical node/grasp acceptance not passed; inspect $Output/acceptance.json" }
} finally {
    Pop-Location
}
