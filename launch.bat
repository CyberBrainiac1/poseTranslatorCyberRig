@echo off
setlocal EnableExtensions
title Motion Translator Launcher

set "APP_DIR=%~dp0"
set "SCRIPT=%APP_DIR%translator.py"
set "TRIGGER=%APP_DIR%autostart.enable"
set "VENV_ACTIVATE=%APP_DIR%venv\Scripts\activate.bat"
set "VENV_PYTHONW=%APP_DIR%venv\Scripts\pythonw.exe"
set "PYTHONW_EXE="

if exist "%VENV_ACTIVATE%" (
    call "%VENV_ACTIVATE%"
)

if exist "%VENV_PYTHONW%" (
    set "PYTHONW_EXE=%VENV_PYTHONW%"
)

if not defined PYTHONW_EXE (
    for %%P in (pythonw.exe) do (
        if not "%%~$PATH:P"=="" set "PYTHONW_EXE=%%~$PATH:P"
    )
)

if not defined PYTHONW_EXE (
    for /f "usebackq delims=" %%P in (`py -c "import pathlib, sys; p = pathlib.Path(sys.executable).with_name('pythonw.exe'); print(p if p.exists() else '')" 2^>nul`) do (
        if not "%%P"=="" set "PYTHONW_EXE=%%P"
    )
)

if not defined PYTHONW_EXE (
    echo ERROR: Python with pythonw.exe is not installed or is not available on PATH.
    echo.
    echo Install Python 3.9 through 3.12 from https://www.python.org/downloads/windows/
    echo During install, enable "Add python.exe to PATH", then run this launcher again.
    echo.
    pause
    exit /b 1
)

if not exist "%SCRIPT%" (
    echo ERROR: translator.py was not found in:
    echo %APP_DIR%
    echo.
    pause
    exit /b 1
)

pushd "%APP_DIR%" >nul
start "" /D "%APP_DIR%" "%PYTHONW_EXE%" "%SCRIPT%"
ping -n 3 127.0.0.1 >nul
> "%TRIGGER%" echo enable
popd >nul

endlocal
exit /b 0
