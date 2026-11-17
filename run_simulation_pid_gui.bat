@echo off
cd /d "%~dp0"
title PyBullet 3D Simulation (PID)
echo ===================================================
echo Starting Self-Balancing Robot 3D PyBullet Simulation
echo Controller: PID
echo ===================================================
set "PATH=%~dp0env;%~dp0env\Scripts;%~dp0env\Library\bin;%PATH%"
set "PYTHONPATH=%~dp0src;%PYTHONPATH%"

if not exist "%~dp0env\python.exe" (
    echo [ERROR] Python environment not found in %~dp0env
    pause
    exit /b 1
)

"%~dp0env\python.exe" scripts\run_simulation.py --controller pid --gui --config-dir config

echo.
echo Simulation session ended.
pause
