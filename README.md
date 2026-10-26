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
**Main window**: five steps across the top. Click any step to go there; the
"Next" button at the bottom of each panel moves on. A tick marks a finished
step; an amber dot marks one that is out of date because an earlier step changed.
1. **Part** - open a STEP part or assembly. Give each body a role: **Build**
   (sliced and sprayed, e.g. a repair patch), **Fixture** (the component,
   clamps: collision-checked only) or **Ignore**. Hide / isolate bodies to reach
   hidden faces, then pick the **start face** (on the build body: layers are
   parallel to it and grow away from it) and optionally a **base face** for the
   part to stand on.
2. **Position** - place it on the faceplate (drag handles, X/Y/Z/A/B/C boxes or
   a SpaceMouse), optional substrate plate. **Keep on plate** (on when a base
   face is set) stands it on its base face and locks Z/B/C; untick it to place
   the part anywhere in 3D (fixture, riser). "Snap base face down" / "Drop onto
   plate" move it once. Table tilt E2, rotation E5, gantry E1, gun roll and
   nozzle; "Auto position" (also run on entering this step) picks tilt, gun
   roll and E1 to keep the gun level (max `gun_max_down_deg` below horizontal).
3. **Toolpath** - strategy (Layers, or Revolve: coat a round part while the
   turntable spins) and process parameters, then Generate. "Save as default"
   writes the process values to `config/process.yaml`.
4. **Simulate** - reach, gun angle and collision results (the collision check
   runs in the background after Generate) and playback.
5. **Export** - save the KRL program.

Start it with:
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
| `src/coldspray/gui/` | Main window: `app.py` shell and step bar, `steps/` one module per step |

## Before running on the real robot
- Gun TCP is computed from CAD: check `clock_deg` / nozzle length in `tool.yaml`, or paste the taught TOOL_DATA.
- BASE_DATA must be taught at the turntable faceplate centre (`cell.yaml`).
- Gantry: confirm E1 = 0 matches the CAD carriage position, and the travel limits.
- Turntable: confirm +E2 tilts towards the tailstock and which way +E5 turns, the
  axis limits and max speeds, and whether it is mathematically coupled to BASE 1
  (`cell.yaml` → `turntable.coupled`).
- Verify kinematics: compare pendant X/Y/Z/A/B/C with our FK at several joint positions.
- First run in T1, spray off, well clear of the part.
