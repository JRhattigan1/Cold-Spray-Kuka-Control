@echo off
rem Start the cold spray window, or run the tests:  run   /   run test
set PY=%USERPROFILE%\.venvs\coldspray\Scripts\python.exe
cd /d "%~dp0"
if /i "%~1"=="test" (
    "%PY%" -m pytest %2 %3 %4 %5
) else (
    "%PY%" -m coldspray.gui
)
