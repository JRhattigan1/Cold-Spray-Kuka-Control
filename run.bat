@echo off
rem Start the cold spray window, or run the tests:  run   /   run test   /   run test serial
rem The tests run on 4 workers (pytest-xdist, one test file per worker). Not 6+: each worker holds ~2.5 GB and a GPU context; two parallel runs at once crashed the PC (video memory bugcheck 0x10E, 2026-10-29).
set PY=%USERPROFILE%\.venvs\coldspray\Scripts\python.exe
cd /d "%~dp0"
if /i "%~1"=="test" (
    if /i "%~2"=="serial" (
        "%PY%" -m pytest %3 %4 %5 %6
    ) else (
        "%PY%" -m pytest -n 4 --dist loadfile %2 %3 %4 %5
    )
) else (
    "%PY%" -m coldspray.gui
)
