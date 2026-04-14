@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "PS_SCRIPT=%SCRIPT_DIR%run_full_workflow.ps1"

if not exist "%PS_SCRIPT%" (
    echo Could not find "%PS_SCRIPT%"
    exit /b 1
)

powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%PS_SCRIPT%" %*
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%"=="0" (
    echo.
    echo Workflow failed with exit code %EXIT_CODE%.
)

exit /b %EXIT_CODE%
