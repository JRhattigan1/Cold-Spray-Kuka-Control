@echo off
rem Start the cold spray window, or run the tests:  run   /   run test   /   run test serial
rem The tests run on 6 workers (pytest-xdist, one test file per worker): ~10 min instead of ~30.
set PY=%USERPROFILE%\.venvs\coldspray\Scripts\python.exe
cd /d "%~dp0"
if /i "%~1"=="test" (
    if /i "%~2"=="serial" (
        "%PY%" -m pytest %3 %4 %5 %6
    ) else (
        "%PY%" -m pytest -n 6 --dist loadfile %2 %3 %4 %5
    )
) else (
    "%PY%" -m coldspray.gui
)
