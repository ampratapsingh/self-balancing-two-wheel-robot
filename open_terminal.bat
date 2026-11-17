@echo off
cd /d "%~dp0"
title Robot Development Terminal
echo ===================================================
echo Self-Balancing Robot Terminal
echo Environment: Ready with Python, PyBullet, SciPy, NumPy
echo ===================================================
set "PATH=%~dp0env;%~dp0env\Scripts;%~dp0env\Library\bin;%PATH%"
set "PYTHONPATH=%~dp0src;%PYTHONPATH%"

cmd /k
