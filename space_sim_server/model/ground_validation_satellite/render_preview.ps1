param([string]$Python = '')
& (Join-Path $PSScriptRoot 'view_preview.ps1') -Render -Python $Python
