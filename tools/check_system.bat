@echo off
REM SentinelAI laptop self-check: packages, cameras 0-3, model speed, settings, robot reachability.
REM Double-click this file. Result: reports\system_check.json (no images, no passwords).
cd /d "%~dp0\.."
if not exist ".venv\Scripts\python.exe" (
  echo The .venv folder is missing. Run start_sentinel.bat once first, then run this check again.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" tools\check_system.py
echo.
pause
