@echo off
REM ================================================================
REM  SentinelAI - one-click start (Windows)
REM  First run: creates .venv and installs packages (needs internet, ~5-10 min).
REM  Then starts the app and opens the dashboard at http://127.0.0.1:5000
REM  Extra options are passed to run_app.py, e.g.:
REM     start_sentinel.bat --source http://PHONE_IP:8080/video
REM     start_sentinel.bat --robot-sim
REM ================================================================
setlocal
cd /d "%~dp0"
title SentinelAI

if exist ".venv\Scripts\python.exe" goto run

echo [setup] Looking for Python 3.12 / 3.11 ...
set "PYCMD="
py -3.12 --version >nul 2>&1 && set "PYCMD=py -3.12"
if not defined PYCMD py -3.11 --version >nul 2>&1 && set "PYCMD=py -3.11"
if not defined PYCMD py -3.13 --version >nul 2>&1 && set "PYCMD=py -3.13"
if not defined PYCMD (
  echo.
  echo [error] Python 3.11, 3.12 or 3.13 was not found.
  echo         Install Python 3.12 from https://www.python.org/downloads/
  echo         ^(tick "Add python.exe to PATH" and keep "py launcher" selected^), then run this file again.
  echo.
  pause
  exit /b 1
)
echo [setup] Using %PYCMD%
%PYCMD% -m venv .venv
if errorlevel 1 (
  echo [error] Could not create the virtual environment.
  pause
  exit /b 1
)
echo [setup] Installing packages - this takes a few minutes the first time ...
".venv\Scripts\python.exe" -m pip install --upgrade pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 (
  echo.
  echo [error] Package installation failed. Check the internet connection and see docs\TROUBLESHOOTING.md
  rmdir /s /q .venv
  pause
  exit /b 1
)
echo [setup] Done.

:run
echo.
echo Starting SentinelAI ...  ^(close this window or press Ctrl+C to stop^)
echo.
".venv\Scripts\python.exe" run_app.py %*
echo.
echo SentinelAI stopped. Messages above show why if it was an error.
pause
