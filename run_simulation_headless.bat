@echo off
cd /d "%~dp0"
title Headless Simulation (LQR)
echo ===================================================
echo Running Headless Simulation (LQR Controller)
echo ===================================================
set "PATH=%~dp0env;%~dp0env\Scripts;%~dp0env\Library\bin;%PATH%"
set "PYTHONPATH=%~dp0src;%PYTHONPATH%"

if not exist "%~dp0env\python.exe" (
    echo [ERROR] Python environment not found in %~dp0env
    pause
    exit /b 1
)

"%~dp0env\python.exe" scripts\run_simulation.py --controller lqr --headless --config-dir config

echo.
echo Simulation finished. Telemetry data saved to results\data\manual\
pause
