@echo off
cd /d "%~dp0"
title Robot Test Suite
echo ===================================================
echo Running Self-Balancing Robot Test Suite
echo ===================================================
set "PATH=%~dp0env;%~dp0env\Scripts;%~dp0env\Library\bin;%PATH%"
set "PYTHONPATH=%~dp0src;%PYTHONPATH%"

if not exist "%~dp0env\python.exe" (
    echo [ERROR] Python environment not found in %~dp0env
    pause
    exit /b 1
)

"%~dp0env\python.exe" -m pytest -v

echo.
echo Test suite finished.
pause
