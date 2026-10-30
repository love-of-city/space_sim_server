@echo off
setlocal
set "PWSH="
for %%P in (pwsh.exe) do set "PWSH=%%~$PATH:P"
if not defined PWSH if exist "%ProgramFiles%\PowerShell\7\pwsh.exe" set "PWSH=%ProgramFiles%\PowerShell\7\pwsh.exe"
if not defined PWSH (
    echo PowerShell 7 was not found. Install it before running this launcher.
    pause
    exit /b 1
)
"%PWSH%" -NoLogo -NoProfile -ExecutionPolicy RemoteSigned -File "%~dp0scripts\deployment_launcher.ps1" -Action Stop %*
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" (
    echo.
    echo Stop failed. Read the error above. Existing services may still be running.
    pause
)
exit /b %RESULT%
