@echo off
setlocal
set "PWSH="
for %%P in (pwsh.exe) do set "PWSH=%%~$PATH:P"
if not defined PWSH if exist "%ProgramFiles%\PowerShell\7\pwsh.exe" set "PWSH=%ProgramFiles%\PowerShell\7\pwsh.exe"
if not defined PWSH (
    echo PowerShell 7 was not found.
    pause
    exit /b 1
)
"%PWSH%" -NoLogo -NoProfile -STA -ExecutionPolicy RemoteSigned -File "%~dp0scripts\show_public_access.ps1" %*
exit /b %ERRORLEVEL%
