function Get-SceneRenderProfiles {
    Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '../config/scene_render_profiles.json') | ConvertFrom-Json
}
function Get-SceneRenderProfile([string]$TemplateId) {
    $profiles=Get-SceneRenderProfiles
    $property=$profiles.PSObject.Properties[$TemplateId]
    if($property){$property.Value}
}
function Resolve-SceneCameraIds([string]$TemplateId, [string[]]$RequestedIds) {
    $profile=Get-SceneRenderProfile $TemplateId
    $ids=@($RequestedIds | ForEach-Object {$_ -split ','} | ForEach-Object {$_.Trim()} | Where-Object {$_})
    if($profile){$ids+=@($profile.camera_ids)}
    $ids | Select-Object -Unique
}
