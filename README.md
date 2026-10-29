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
between runs. In Simple mode the Toolpath and Position steps hide advanced
fields; any advanced value that differs from its default stays visible with a
"changed" chip. Advanced mode shows every field. (The Part step is not yet tied
to the switch.) Long jobs (Generate, Auto position, the collision check, Solve
problems) run in the background with a progress bar and Cancel.
The sidebar and the 3D view share the window: drag the gap between them to
widen the sidebar (its width is remembered). The window starts at 1440x900, or
less on a small screen, and can't go below 1100x680. Each step's "Next" button
stays at the bottom of its panel while the panel scrolls.

1. **Part** - open a STEP part or assembly, or pick a built-in test part from
   the drop-down (square, block with hole, round boss, stepped block, taper,
   U-block, tube, curved panel, repair, groove repair). Give each body a role:
   **Build** (sliced and sprayed, e.g. a repair patch), **Fixture** (the
   component, clamps: collision-checked only) or **Ignore**. Hide / isolate
   bodies to reach hidden faces, then pick the **start face** (on the build
   body: layers are parallel to it and grow away from it) and optionally a
   **base face** for the part to stand on.
   **Coat faces** (for the Surface strategy, any body): tick "Pick coat faces",
   then click toggles a face, Shift+click adds, Ctrl+click removes. "Grow to
   tangent faces" adds neighbours whose normals are within 10° across the
   shared edge. "Clear" empties the set. Coat faces show orange.
   **KRL base (optional)**: "Pick KRL base", then click a flat face near the
   corner to use as the origin, beside the edge to use as X: +Z points out of
   the face, +X along that edge, the origin is the corner. Red/green/blue arrows
   show it. The program is then written relative to this frame and selects
   `BASE_DATA[part_base_number]` (`cell.yaml`, default 2), which you teach on the
   real part (3-point: origin, along X, into the XY plane). Only the KRL changes;
   the simulation is the same. "Clear" goes back to the faceplate BASE.
2. **Position**
   - **Mounted**: *On the turntable* (default) or *Fixed in the cell* - a part
     held off the table (fixture, stand) anywhere the robot can reach. Fixed:
     the table is held at 0 (an obstacle only), no substrate plate, no faceplate
     limits, placement boxes reach ±6 m (still measured from the faceplate
     centre with the table at 0), Auto position searches E1 only, Revolve is
     not allowed.
   - **Placement**: drag handles, X/Y/Z/A/B/C boxes or a SpaceMouse; optional
     substrate plate. **Keep on plate** (on when a base face is set) stands the
     part on its base face and locks Z/B/C; untick it to place it anywhere in 3D
     (fixture, riser). "Snap base face down" / "Drop onto plate" move it once.
   - **Constraints** (Advanced): touch, align, distance, parallel,
     perpendicular, angle, centre, centre-between, fix - between part faces and
     the faceplate, substrate, datum planes or table axis. Each row has a
     **Pick** button: click it, then click the face in the 3D view (flat or
     cylindrical faces; Esc or Pick again cancels). The selected row's faces
     show purple. The header chip shows the degrees of freedom left.
   - **Datums** (cell view toolbar): XY / XZ / YZ planes through the faceplate
     centre, each with a checkbox and an offset (mm). They tilt and turn with
     the table.
   - **Table and gantry**: table tilt E2, rotation E5, gantry E1, nozzle, and
     **Gun angle** - the gun's turn about its spray axis from the usual set-up.
     **Auto position** (button only; it no longer runs on entering the step)
     picks tilt and E1 so the gun stays level (at most `gun_max_down_deg` below
     horizontal) and the whole path is reachable.
   - **Warm-up / purge point**: Enabled, X/Y/Z (WORLD mm), spray direction
     preset (choose "Custom" to type a direction; it is normalised), dwell, at
     start / at end / every N layers. **Suggest purge point** fills these in
     front of the filtration unit, spraying into it, stepped back towards the
     robot until it is reachable; every value stays editable. A ghost gun shows
     the point; the chip says whether it is in reach at the current E1. The
     program goes PTP to the point, switches the spray **on**, dwells, then
     travels to the part **with the spray still on**, and returns there at the
     end. "Save to cell.yaml" writes the `purge:` block (other lines and
     comments are kept).
3. **Toolpath** - Mode (Layers / Revolve / Surface; hover for the long
   description), process parameters, then Generate. "Save as
   default" writes the process values to `config/process.yaml`.
   - **Layers**: contour passes, then infill. `infill_pattern`: zigzag, oneway
     (all strokes one way, lifted return), concentric, spiral, islands
     (checkerboard tiles); old configs with crosshatch load as zigzag. `raster_start_deg` sets
     the first raster direction (from BASE X); `raster_angle_step_deg` turns it
     each layer. `contour_tilt_deg` > 0 leans the contour passes outward (into
     the hole for holes). Long gaps are lifted jumps of `jump_height_mm`, and the
     move to the next layer lifts the same (0: the gun stays at spray height and
     the spray stays on between layers). The part is not a collision body in
     Layers (it isn't built yet); in Revolve and Surface it is.
     `turn_mode` puts the raster turns on the part (`inside`), `overrun_mm` past
     its edge (`overrun`), or past a datum plane (`beyond_plane` + `turn_plane`),
     e.g. off a masked area. Path colours: contour magenta, infill orange, turn
     yellow, jump grey.
   - **Revolve**: coat a round part while the turntable spins.
   - **Surface**: coat the faces picked in step 1, gun along the surface normal
     at standoff, curved faces allowed. Picked faces that don't touch are
     separate regions, joined by a jump; a region that wraps round is split per
     CAD face (own raster, gun off between). Step-over is measured along the
     surface, so passes stay evenly spaced over blends. With **zigzag** the faces are cut by
     parallel planes (strokes along the longest side, turned by
     `raster_start_deg`; rings round a whole cylinder). With **any other
     pattern** a region that faces one way (an end face, a plate top, a curved
     panel) is filled like a layer - contour passes, then the pattern - and laid
     on the faces; a region that wraps round is zigzagged and the summary says
     so. `surface_coats` coats; points tilted more than `surface_max_tilt_deg`
     from the mean normal are skipped.
   - **Gun hold, all strategies**: the gun never winds round (the cables and
     powder line out of the back are not in the CAD). The first pose has the
     wrist up, or towards the robot when spraying down; after that the gun keeps
     its attitude and only tilts as the spray direction changes - it does not
     turn with the travel direction on loops or zig-zags. Then the Gun angle is
     added.
4. **Simulate** - playback with reach, gun angle and collision results (the
   collision check runs in the background after Generate; "Check collisions"
   turns it off). Collisions cover the robot, gun, table, gantry carriage and
   cell; the HOME → first and last → HOME legs are checked as the KRL moves
   them. A gun more than `gun_max_down_deg` (70°) below horizontal is a
   warning, as is A4 / A6 winding further than `cell.cable_windup_deg` from
   home (270°, a placeholder). **Markers** (view toolbar) shows or hides the
   collision, unreachable and steep-gun points. The **flip** row flags a joint
   jump of more than 20° inside one LIN or CIRC (a configuration flip the real
   arm can't make). **Solve problems** tries to fix what the check found (the
   status bar shows "Solver: trying …" for each): arm configuration, gun angle
   (±30°) and lean (±10°), re-routing approach / jump / purge moves, E1 per
   layer (±1000 mm; a change of E1 is written as a PTP), one gantry E1 for the
   whole program, and moving the purge point (XYZ, angle kept). Suggested E1 /
   purge values are filled in with **Apply suggested set-up**. What it can't
   fix is listed and Save stays disabled. The **Before the robot** list
   collects what still has to be checked on the cell. Cycle time and powder
   use (`powder_feed_g_min`, a placeholder) are estimated.
   **Thickness** colours the part by predicted coating thickness (grey = none).
   **Build-up** shows material going down during playback: in Layers, finished
   layers solid (copper) and the layer being sprayed track by track in two
   alternating shades (path lines hidden);
   in other modes the thickness map fills in as the gun passes.
5. **Export** - shows the BASE / TOOL numbers (the part's KRL base when one is
   picked), spray output and turntable mode, the cycle time and powder
   estimate, then saves the KRL `.src`. Options: **Pre-flight check** (HALT
   before motion if `$ACT_TOOL` / `$ACT_BASE` differ or are untaught),
   **Split long programs** (main + `_Pnn` subprograms) and, in Advanced,
   **Program switches spray** (untick to spray by hand: the `$OUT` lines become
   comments, timing unchanged). Open items from the Before the robot list are
   confirmed once before saving.

**Jobs**: the Job button (Ctrl+S / Ctrl+O, recent jobs) saves and opens a
`*.csjob` file: part, placement, constraints, KRL base, process settings, E1,
purge and export options. Opening a job does not regenerate (press Generate).
**Undo / redo** of Position edits: Ctrl+Z, Ctrl+Shift+Z / Ctrl+Y, or ↶ ↷ in
the step bar (purge edits are not covered).

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
| `src/coldspray/kinematics/` | KR 50 R2500 forward / inverse kinematics (batched OPW IK for whole paths) |
| `src/coldspray/cad/` | STEP import, mesh cache, layer slicing |
| `src/coldspray/cad/part.py` | Part model: placement, start / base / coat faces, KRL base, fixed in the cell |
| `src/coldspray/cad/constraints.py` | Placement constraint solver (touch, align, distance, centre, ...) |
| `src/coldspray/pipeline.py` | Part loading (STEP or built-in), `plan_part` for each strategy, placement checks |
| `src/coldspray/planner/layers.py` | Layers strategy: contours + infill per layer, jumps |
| `src/coldspray/planner/patterns.py` | Infill patterns (`PATTERNS`), contour loops, stroke ordering |
| `src/coldspray/planner/turns.py` | Raster turns off the part (`turn_mode`) |
| `src/coldspray/planner/contour_tilt.py` | Outward-tilted contour poses |
| `src/coldspray/planner/revolve.py` | Revolve strategy |
| `src/coldspray/planner/surface.py` | Surface strategy: plane-cut zigzag, or patterns on flattened regions |
| `src/coldspray/planner/gun.py` | Gun hold: no winding, wrist up / towards the robot, gun angle |
| `src/coldspray/planner/purge.py` | Warm-up / purge visits, purge point suggestion |
| `src/coldspray/planner/position.py` | Auto position (table tilt, E1) |
| `src/coldspray/planner/solver.py`, `solve_*.py` | Solve problems: arm, gun angle, re-route, E1 per layer, gantry, purge levers |
| `src/coldspray/toolpath/` | Toolpath format, sampling for simulation |
| `src/coldspray/post/krl.py` | Toolpath → KRL `.src` (faceplate BASE or the part's KRL base) |
| `src/coldspray/post/joints.py` | PTP joints for the program, computed as Simulate does |
| `src/coldspray/post/krl_split.py`, `krl_preflight.py` | Split a long program into subprograms; TOOL/BASE check at the start (Export options) |
| `src/coldspray/post/krl_status.py` | Status/Turn bits (unverified, not used by Export yet) |
| `src/coldspray/job_io.py` | Job files (`*.csjob`, versioned YAML) |
| `src/coldspray/sim/` | 3D scene (cell, robot, gun, turntable, path), collision checking |
| `src/coldspray/sim/deposit.py` | Predicted coating thickness, Build-up playback, cycle time and powder estimates |
| `src/coldspray/sim/datums.py` | XY / XZ / YZ datum planes on the faceplate |
| `src/coldspray/sim/markers.py` | Ghost gun marker for the purge point |
| `src/coldspray/gui/` | Main window: `app.py` shell and step bar, `steps/` one module per step |
| `src/coldspray/gui/plan_jobs.py` | Planning on a worker thread, plan cache |
| `src/coldspray/gui/checks.py` | Simulate and "Before the robot" check lists |
| `src/coldspray/gui/job_menu.py`, `undo.py` | Job button and recent jobs; Position undo/redo |
| `tools/` | `bench_solve.py`, `bench_plan.py` (timings), `ui_screenshots.py` |
| `.github/workflows/tests.yml` | CI: KRL hashes, imports, planner/post unit tests |
| `plans/` | `NEXT_SESSION.md` (start here), `ROADMAP.md`, `HAND_CHECK.md`, agent reports |

## Before running on the real robot
- Gun TCP is computed from CAD: check `clock_deg` / nozzle length in `tool.yaml`, or paste the taught TOOL_DATA.
- BASE_DATA[1] must be taught at the turntable faceplate centre (`cell.yaml`).
  With a KRL base picked on the part, teach BASE_DATA[`part_base_number`] on the
  real part instead and compare it with the values in the program header
  (taught with the table at 0 when the table is not coupled).
- Part fixed in the cell: the program drives the table to E2 = 0, E5 = 0; check
  the table is clear and that is where you expect it.
- Gantry: confirm E1 = 0 matches the CAD carriage position, and the travel limits.
  Programs from Solve problems may change E1 between layers (written as PTP,
  on gun-off moves only). The gantry carriage is in the collision check.
  Simulation flags HOME at E1 2730 (nozzle into the rotary base) and E1 5500
  (arm crosses the booth wall): check both on the cell.
- Turntable: confirm +E2 tilts towards the tailstock and which way +E5 turns, the
  axis limits and max speeds, and whether it is mathematically coupled to BASE 1
  (`cell.yaml` → `turntable.coupled`).
- Verify kinematics: compare pendant X/Y/Z/A/B/C with our FK at several joint positions.
- Purge point: CHECK `spray_dir` on the cell (the default is a guess) so the
  purge blows into the booth or filtration unit, not at people or kit.
- Purge transit is sprayed: the gun travels purge point → part → purge point
  with the spray ON. Check the collision results for the purge moves and that
  the path through free space does not cross anything.
- Cables and powder line: the gun hold avoids winding the gun round, and
  Simulate warns past `cell.cable_windup_deg`, but that limit is a guess: set
  it from the cable dressing.
- Contour tilt and Surface mode change the gun orientation along a pass: run
  them in T1 first and watch for wrist flips / singularities.
- First run in T1, spray off, well clear of the part.
