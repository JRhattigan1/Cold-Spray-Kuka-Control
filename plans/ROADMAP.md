# Roadmap: cold-spray feature plan (status after session 1)

Source of truth: tracker in `plans/2026-10-27-feature-plan.md` §5 and the reports in `plans/reports/`. This page states only what those reports say. Session 2 (A3, G1, E1b, F2b, A2b, T1, R1-R3, DOC1/2) is running; its results are not reflected here yet.

## Feature by feature

| Feature | Works now (on main) | Left | Risks |
|---|---|---|---|
| **A Constraints + datum planes** | A1: pure solver `cad/constraints.py`, all 9 types, conflict status, DOF from Jacobian rank, < 50 ms, agrees with `constrain()`. A2: XY/XZ/YZ datum planes in the cell view (`sim/datums.py`), toggle + offset menu, follow tilt/rotation, `pick_name` gives `"datum:XZ"`. | A3 constraint UI (running). A2: 3D labels, size from faceplate (fixed 600 mm now; A2b running), real cell-view pick flow (A3). | Cell-view face picking is new. Status is only ok/conflicting (over-constraint is an info flag). |
| **B Infill patterns** | B1: `planner/patterns.py` with zigzag, oneway, concentric, spiral, crosshatch, islands; ≥ 95 % coverage tests; zigzag identical to old. B2: `plan_layers` uses `PATTERNS[proc.infill_pattern]`; path coloured by `Move.tag`. | Per-pattern move-count snapshots and reach/collision per pattern (T1 running). Contour seam rotation (changes KRL hashes, needs its own WP). GUI check of tag colours. | Seam rotation will change the regression hashes on purpose. |
| **C Contour tilt** | C1: `planner/contour_tilt.py`, outward lean incl. holes, smoothed normals, 20° ± 0.5° on test_block, reach 100 %; tilt 0 is byte-identical. | Optional cap on orientation change per move (45° step at a 90° corner). | Wrist flips / singularities: watch in Simulate. |
| **D Coat faces + Surface mode** | D1: `Part.coat_faces`, pick/Shift/Ctrl, grow-to-tangent, highlight. D2: `planner/surface.py` conformal raster, coats, tilt limit, overrun turns. D3: `MODES["surface"]`, routing in `plan_part`, clear error for empty selection. | End-to-end run on test_repair with reach, collisions, KRL (T1 running). Auto-position in surface mode not exercised. Invalidate the toolpath when coat faces change. | Surface turns are straight gun-on LINs with no lift; unbridged pieces joined without lift; could pass close to a convex surface. |
| **E Simple / Advanced UI** | E1: mode registry `widgets.advanced()`, `ModeToggle` in the header, persisted with QSettings. WP0: generic process form (combo/check/spin). | Register advanced process-form rows (E1b running). E2: sweep Part/Position/Simulate/Export, shortcuts, tooltips, README, screenshots. | Simple mode must never hide a changed value without a chip. |
| **F Purge point** | F1: `planner/purge.py` `add_purge` (start, end, every N layers), WORLD-fixed at any tilt, dwell in KRL, spray stays on between purge and part; hooked into all three `plan_part` returns. F2: purge Section in Position step, ghost gun marker, reach chip. | Save to cell.yaml (F2b running). Drag marker, "Move gun here" IK preview, custom direction / `approach_mm` widget. `plan_part` test with a real PurgeConfig. | Purge transit has the spray on in free space; `auto_position` now also tries to reach the purge point. Spray direction default is a CHECK. |
| **G Turns off the part** | Surface mode already has overrun turns (D2). `turn_mode`/`turn_plane` config fields exist (WP0). | G1 overrun / beyond_plane for layer rasters (running, `planner/turns.py`); `turn_plane` picker in the form. | Greedy ordering must not jump across the part. |

## Recommended order for the remaining work
1. **Merge and review session 2** (R1-R3 findings, then A3, G1, E1b, F2b, A2b, T1, DOC1, DOC2). Rerun the suite on main; KRL hashes must stay unchanged unless a WP says why.
2. **Clean-up:** remove the `getattr(proc/cell, ..., default)` fallbacks (B2, C1, D3, F1, F2, A1) now the config fields are on main.
3. **Resume what is still partial** after session 2: F2 drag / IK preview / custom direction, D2 lift on turns, D1 coat-change invalidation, B2 seam rotation (own WP, update hashes), C1 orientation cap if the wrist is jerky.
4. **Human GUI check** (`.\run`): patterns + contour tilt on test_block, datum planes, coat faces + Surface mode, purge marker, constraints on test_tube, turns past the YZ plane, Simple/Advanced toggle.
5. **E2 final UI sweep**, alone, after everything above has merged.
6. **Robot-cell verification** (README "Before running on the real robot", plan §6):
   - Gun TCP from CAD: check `clock_deg` / nozzle length in `tool.yaml`, or paste the taught TOOL_DATA.
   - BASE_DATA taught at the turntable faceplate centre (`cell.yaml`).
   - Gantry: E1 = 0 matches the CAD carriage position; travel limits.
   - Turntable: +E2 tilt direction, +E5 turn direction, axis limits and max speeds, and whether it is coupled to BASE 1 (`turntable.coupled`).
   - Kinematics: compare pendant X/Y/Z/A/B/C with our FK at several joint positions.
   - **Purge `spray_dir` default: CHECK on the cell** (marked in `cell.yaml`), and check collision results for the purge moves (spray on in free space).
   - Every new motion first in T1, spray off, well clear of the part.
