param(
    [switch]$Render,
    [switch]$Interactive,
    [switch]$FreePreview,
    [string]$Python = ''
)
$ErrorActionPreference = 'Stop'
if (!$Render -and !$Interactive) {
    $image = Join-Path $PSScriptRoot 'preview\overview.png'
    if (!(Test-Path -LiteralPath $image)) { throw 'Preview image is missing. Run render_preview.ps1 first.' }
    Invoke-Item -LiteralPath $image
    return
}
$repoCandidates = @(
    [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..')),
    [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\space_sim_server'))
)
$repo = $repoCandidates | Where-Object { Test-Path -LiteralPath (Join-Path $_ 'tools\render_step_preview.py') } | Select-Object -First 1
if (!$repo) { throw 'Cannot find tools\render_step_preview.py relative to this model.' }
$workspace = Split-Path -Parent $repo
if (!$Python) {
    $pythonCandidates = @(
        (Join-Path $workspace 'run\step-converter-venv\Scripts\python.exe'),
        (Join-Path $repo '.venv-step\Scripts\python.exe')
    )
    $Python = $pythonCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (!$Python) { throw 'Pass -Python with the dedicated converter Python path; see README.md.' }
$pythonArgs = @((Join-Path $repo 'tools\render_step_preview.py'), $PSScriptRoot, '--backend', 'cpu')
if ($Interactive) { $pythonArgs += '--interactive' }
if ($FreePreview) { $pythonArgs += '--free-preview' }
& $Python @pythonArgs
if ($LASTEXITCODE -ne 0) { throw "Model preview exited with code $LASTEXITCODE" }
