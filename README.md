# Cold Spray Kuka Control

Toolpath generation, 3D simulation and KRL program export for cold spray
additive manufacturing on a KUKA KR 50 R2500 (KR C4), hanging from a KIEHNE
gantry (E1), with a GTV KDT3000 tilting turntable and an Impact Gun 6/11 EvoCSII.

## Setup (once)
```
py -3.12 -m venv C:\Users\qrb21117\.venvs\coldspray
C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pip install -e .[sim,cad,dev]
```
The virtual environment lives outside OneDrive on purpose.

CAD files and manuals go in `assets/cad/` and `docs/manuals/`. Both are kept
out of git (licensed material). Convert the cell model once (takes ~8 min):
```
C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m coldspray.cad.step_cache "assets\cad\Cold Spray Cell.stp" assets\cad\cache\cell
```

## Run
**Main window** - open a STEP part or assembly and set it up on the faceplate
(drag handles, X/Y/Z/A/B/C boxes or a SpaceMouse; "Pick base face" + "Snap face
down" stands it on a flat face). With several bodies, pick the **build body**
(sliced and sprayed, e.g. a repair patch); the others (component, fixtures) are
collision-checked only. The substrate plate is optional. Layers build up from
the bottom of the build body. "Auto position" picks table tilt, gun roll and
gantry E1 to keep the gun level (max `gun_max_down_deg` below horizontal).
Choose a strategy (Layers, or Revolve: coat a round part while the turntable
spins), set process parameters, generate, check reach and collisions in 3D,
save the KRL program:
```
.\run          (or double-click run.bat)
.\run test     (the tests)
```
which runs `C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m coldspray.gui`.
Other entry points: `python -m coldspray.demo_part [part.step]` (command line),
`python -m coldspray.demo` (flat plate raster), `python -m pytest` (tests).

## Layout
| Path | What |
|---|---|
| `src/coldspray/config/` | Robot, tool, cell and process parameters (YAML) |
| `src/coldspray/kinematics/` | KR 50 R2500 forward / inverse kinematics |
| `src/coldspray/cad/` | STEP import, mesh cache, layer slicing |
| `src/coldspray/planner/` | Contour + raster spray passes per layer |
| `src/coldspray/toolpath/` | Toolpath format |
| `src/coldspray/post/krl.py` | Toolpath → KRL `.src` |
| `src/coldspray/sim/` | 3D scene (cell, robot, gun, turntable, path), collision checking |
| `src/coldspray/gui.py` | Main window |

## Before running on the real robot
- Gun TCP is computed from CAD: check `clock_deg` / nozzle length in `tool.yaml`, or paste the taught TOOL_DATA.
- BASE_DATA must be taught at the turntable faceplate centre (`cell.yaml`).
- Gantry: confirm E1 = 0 matches the CAD carriage position, and the travel limits.
- Turntable: confirm +E2 tilts towards the tailstock and which way +E5 turns, the
  axis limits and max speeds, and whether it is mathematically coupled to BASE 1
  (`cell.yaml` → `turntable.coupled`).
- Verify kinematics: compare pendant X/Y/Z/A/B/C with our FK at several joint positions.
- First run in T1, spray off, well clear of the part.
