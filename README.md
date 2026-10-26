# Cold Spray Kuka Control

Toolpath generation, 3D simulation and KRL program export for cold spray
additive manufacturing on a KUKA KR 50 R2500 (KR C4) with linear track and
positioner.

## Setup (once)
```
py -3.12 -m venv C:\Users\qrb21117\.venvs\coldspray
C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pip install -e .[sim,dev]
```
The virtual environment lives outside OneDrive on purpose.

## Run
```
C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m coldspray.demo
```
Writes `output/plate_raster.src` and opens the viewer (drag the slider, press `p` to play).

```
C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pytest
```
Runs the tests.

## Layout
| Path | What |
|---|---|
| `src/coldspray/config/` | Robot, tool, cell and process parameters (YAML) |
| `src/coldspray/kinematics/` | KR 50 R2500 forward / inverse kinematics |
| `src/coldspray/toolpath/` | Toolpath format and path generators |
| `src/coldspray/post/krl.py` | Toolpath → KRL `.src` |
| `src/coldspray/sim/viewer.py` | 3D viewer |

## Before running on the real robot
- `tool.yaml` and `cell.yaml` hold **placeholder** values: replace with the taught TOOL_DATA / BASE_DATA.
- Verify the kinematics: compare pendant X/Y/Z/A/B/C with our FK at several joint positions.
- First run in T1, spray off, well clear of the part.
