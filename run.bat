@echo off
rem Start the cold spray window, or run the tests:
rem   run                  the app
rem   run test             the areas your changes touch + the guard set (about a minute)
rem   run test surface     one area (surface, layers, krl, solver, collision, kinematics, part, cell, sim, gui;
rem                        several with commas; "list" shows them) - see tests/conftest.py
rem   run test all         everything, 4 workers (~12 min): before merging or pushing
rem   run test serial      everything, one at a time (~30 min)
rem Never more than one "all" run at once: two parallel runs crashed the PC (video memory bugcheck 0x10E, 2026-10-29).
set PY=%USERPROFILE%\.venvs\coldspray\Scripts\python.exe
cd /d "%~dp0"
if /i "%~1"=="test" (
    if "%~2"=="" (
        "%PY%" -m pytest -q --changed
    ) else if /i "%~2"=="all" (
        "%PY%" -m pytest -n 4 --dist loadfile %3 %4 %5
    ) else if /i "%~2"=="serial" (
        "%PY%" -m pytest %3 %4 %5 %6
    ) else (
        "%PY%" -m pytest -q --area %2 %3 %4 %5
    )
) else (
    "%PY%" -m coldspray.gui
)
