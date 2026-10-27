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
**Simple | Advanced** in the header (next to the theme button) is remembered
between runs. In Simple mode the Toolpath step hides the advanced process
fields (the collapsible "Advanced" section); any advanced value that differs
from `process.yaml` stays visible with a "changed" chip, and the section chip
shows "N changed". Advanced mode shows every field. Other panels are not yet
tied to the switch (see `plans/`).
1. **Part** - open a STEP part or assembly. Give each body a role: **Build**
   (sliced and sprayed, e.g. a repair patch), **Fixture** (the component,
   clamps: collision-checked only) or **Ignore**. Hide / isolate bodies to reach
   hidden faces, then pick the **start face** (on the build body: layers are
   parallel to it and grow away from it) and optionally a **base face** for the
   part to stand on.
   **Coat faces** (for the Surface strategy, any body): tick "Pick coat faces",
   then click toggles a face, Shift+click adds, Ctrl+click removes. "Grow to
   tangent faces" adds neighbours whose normals are within 10° across the
   shared edge. "Clear" empties the set. Coat faces show orange.
2. **Position** - place it on the faceplate (drag handles, X/Y/Z/A/B/C boxes or
   a SpaceMouse), optional substrate plate. **Keep on plate** (on when a base
   face is set) stands it on its base face and locks Z/B/C; untick it to place
   the part anywhere in 3D (fixture, riser). "Snap base face down" / "Drop onto
   plate" move it once. Table tilt E2, rotation E5, gantry E1, gun roll and
   nozzle; "Auto position" (also run on entering this step) picks tilt, gun
   roll and E1 to keep the gun level (max `gun_max_down_deg` below horizontal).
   **Datums** (cell view toolbar): XY / XZ / YZ planes through the faceplate
   centre, each with a checkbox and an offset (mm). They tilt and turn with the
   table. Constraints (touch, distance, centre, ...) between part faces and
   these planes: solver done (`cad/constraints.py`), UI in progress, see `plans/`.
   **Warm-up / purge point**: Enabled, X/Y/Z (WORLD mm), spray direction
   preset (choose "Custom" to type a dir X/Y/Z vector; it is normalised to unit
   length), dwell, at start / at end / every N layers. A ghost gun shows the
   point; the chip says whether it is in reach at the current E1. The program
   goes PTP to the point, switches the spray **on**, dwells, then travels to
   the part **with the spray still on**, and returns there at the end. The
   spray stays on from the first purge to the last; it is switched off only
   at the end. "Save to cell.yaml" writes the `purge:` block into
   `src/coldspray/config/cell.yaml` (other lines and comments are kept; no
   confirm dialog, the status bar shows where it was saved).
3. **Toolpath** - strategy, process parameters, then Generate. "Save as
   default" writes the process values to `config/process.yaml`. Strategies:
   - **Layers**: contour passes then infill. `infill_pattern`: zigzag, oneway
     (all strokes one direction, lifted return), concentric, spiral,
     crosshatch (0/90° on alternate layers), islands (checkerboard tiles).
     `contour_tilt_deg` > 0 leans the contour passes outward (into the hole
     for holes); infill stays vertical. Path lines are coloured by type:
     contour magenta, infill orange, turn yellow, jump grey.
   - **Revolve**: coat a round part while the turntable spins.
   - **Surface**: raster over the coat faces from step 1, gun normal to the
     surface at standoff, curved faces allowed. `surface_coats` coats,
     alternating direction; points tilted more than `surface_max_tilt_deg`
     from the mean normal are skipped. No coat faces: go back to step 1.
     `infill_pattern` other than zigzag: a region that faces one way (an end
     face, a plate top, a curved panel) is filled like a layer - contour
     passes, then the pattern - and the strokes are laid on the faces. A
     region that wraps round (e.g. a whole cylinder) is zigzagged; the
     summary says how many.
   `turn_mode` / `turn_plane` (turning points off the part, for an overspray
   mask) are in the form but do nothing yet: in progress, see `plans/`.
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
| `src/coldspray/cad/constraints.py` | Placement constraint solver (touch, align, distance, centre, ...) |
| `src/coldspray/planner/` | Contour + raster spray passes per layer |
| `src/coldspray/planner/patterns.py` | Infill patterns (`PATTERNS`), contour loops, stroke ordering |
| `src/coldspray/planner/contour_tilt.py` | Outward-tilted contour poses |
| `src/coldspray/planner/surface.py` | Surface strategy: conformal raster over coat faces |
| `src/coldspray/planner/purge.py` | Adds the warm-up / purge visits to a toolpath |
| `src/coldspray/toolpath/` | Toolpath format |
| `src/coldspray/post/krl.py` | Toolpath → KRL `.src` |
| `src/coldspray/sim/` | 3D scene (cell, robot, gun, turntable, path), collision checking |
| `src/coldspray/sim/datums.py` | XY / XZ / YZ datum planes on the faceplate |
| `src/coldspray/sim/markers.py` | Ghost gun marker for the purge point |
| `src/coldspray/gui/` | Main window: `app.py` shell and step bar, `steps/` one module per step |

## Before running on the real robot
- Gun TCP is computed from CAD: check `clock_deg` / nozzle length in `tool.yaml`, or paste the taught TOOL_DATA.
- BASE_DATA must be taught at the turntable faceplate centre (`cell.yaml`).
- Gantry: confirm E1 = 0 matches the CAD carriage position, and the travel limits.
- Turntable: confirm +E2 tilts towards the tailstock and which way +E5 turns, the
  axis limits and max speeds, and whether it is mathematically coupled to BASE 1
  (`cell.yaml` → `turntable.coupled`).
- Verify kinematics: compare pendant X/Y/Z/A/B/C with our FK at several joint positions.
- Purge point: CHECK `spray_dir` on the cell (the default is a guess) so the
  purge blows into the booth, not at people or kit.
- Purge transit is sprayed: the gun travels purge point → part → purge point
  with the spray ON. Check the collision results for the purge moves and that
  the path through free space does not cross anything.
- Contour tilt and Surface mode change the gun orientation along a pass: run
  them in T1 first and watch for wrist flips / singularities.
- First run in T1, spray off, well clear of the part.
