$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'scene_render_profiles.ps1')
$profile=Get-SceneRenderProfile 'sarm-dual-capture26'
if($profile.camera_ids.Count -ne 3){throw 'Dual scene needs three cameras'}
$ids=@(Resolve-SceneCameraIds 'sarm-dual-capture26' @('teleop/camera/spacecraft_overview'))
if($ids.Count -ne 3 -or $ids -notcontains 'teleop/camera/capture_wrist_cam'){throw 'Partial launch arguments lost capture camera'}
if($profile.asset_catalog -ne 'sarm_dual_capture26.catalog.json'){throw 'Wrong scene assets'}
if($profile.automatic_capture_default -ne $false){throw 'Unexpected automatic capture'}
if(@(Resolve-SceneCameraIds 'unknown' @('one')).Count -ne 1){throw 'Legacy camera arguments changed'}
'PASS: scene profiles and omitted/partial camera arguments'
