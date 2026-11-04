param([string]$UnrealRoot = '', [switch]$Verify)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$python = Resolve-BskPython -RequiredModules @()
& $python (Join-Path $PSScriptRoot 'check_external_environment_assets.py') --project $ProjectRoot
if ($LASTEXITCODE -ne 0) { throw 'Restore the external Earth asset package before preparing or launching the environment.' }
$map = Join-Path $ProjectRoot 'Content/BSK/Maps/BskEnvironment.umap'
if (Test-Path -LiteralPath $map -PathType Leaf) {
    $stream = [IO.File]::OpenRead($map)
    try {
        $header = New-Object byte[] 200
        $count = $stream.Read($header, 0, $header.Length)
        $text = [Text.Encoding]::ASCII.GetString($header, 0, $count)
        if ($text.StartsWith('version https://git-lfs.github.com/spec/v1')) {
            throw "Environment map is a Git LFS pointer: $map. Run git lfs pull."
        }
        if ($count -lt 128) { throw "Environment map is empty or truncated: $map. Refusing to overwrite it." }
        $magic = [BitConverter]::ToString($header, 0, 4).Replace('-', '')
        if ($magic -notin @('C1832A9E', '9E2A83C1')) {
            throw "Environment map is not a UE package: $map. Refusing to overwrite it."
        }
    } finally { $stream.Dispose() }
    if (!$Verify) {
        Write-Output "Preserving saved environment map: $map"
        return
    }
}
$ue = Resolve-UnrealRoot $UnrealRoot
$editor = Join-Path $ue 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
$generator = Join-Path $PSScriptRoot 'create_environment_level.py'
& $editor $ProjectFile -run=pythonscript "-script=$generator" -unattended -nop4 -nosplash -NullRHI
if ($LASTEXITCODE -ne 0) { throw 'Environment level creation failed. Build the Editor target first and inspect the UE log.' }
if (!(Test-Path -LiteralPath $map -PathType Leaf)) { throw "UE did not create the environment map: $map" }
if ($Verify) { Write-Output "Verified editable environment map: $map" }
else { Write-Output "Created editable environment map: $map" }
