@echo off
cd /d "%~dp0"
"C:\tools\powershell-7.4.13\pwsh.exe" -NoProfile -File "%~dp0scripts\remote_visualization.ps1" -Action Start
pause
