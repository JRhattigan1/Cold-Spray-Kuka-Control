# Next session

**Start here.** This replaces the old session hand-offs and run-sheet; git history has them. The feature plan is `plans/2026-10-27-feature-plan.md`. Its §5 tracker, §8 run log and `plans/ROADMAP.md` are **out of date**, so trust this file. Each agent's detail is in `plans/reports/<ID>-1.md`.

## Session 4 (2026-10-27): branch `surface-patterns`, not merged
Two features, committed together on `surface-patterns` (from `main` at `3324d09`).

**1. Surface mode uses `infill_pattern` (`planner/surface.py`).**
- **How it works:**
  - A region that faces one way is flattened along its mean normal, filled like a layer and laid back on the faces by ray cast. Examples: the tube end, plate tops, a curved panel.
  - The fill is `contour_passes` loops, then the pattern.
  - Strokes more than 1.5 × step-over apart are joined by a lifted jump.
- **Zigzag is unchanged.** It still uses the plane-cut raster, and the KRL regression hashes still pass.
- **Regions that wrap round fall back to zigzag**, for example a whole cylinder, or the tube end plus its sides as one connected region. `path.zigzag_regions` counts them, and the summary line says so.
- **Slivers:** where a concentric inset closes up (the centre line of a ring), the slivers are dropped. Loops under step-over² in area and open strokes under one step-over go.
- **Coats:** pattern coats use the coat number as the layer index, so crosshatch crosses and seams rotate.
- **Overrun:** only `oneway` and `crosshatch` infill get it.
- **Tests:** these pass: `tests/test_surface*.py` (4 new tests) and `test_krl_regression.py`.
- **Open questions:**
  - Should a wrapping region be split per CAD face, so the end face gets the pattern while the sides zigzag?
  - Contour passes are not applied in zigzag Surface mode.
  - Pattern step-over is measured in the flattened plane, so it opens up on steep parts of a curved face.
  - The zigzag still sprays across holes (pieces of a cut are bridged). The user's tube-end comment ("covering the whole area") may have been about this. Ask them.

**2. Optional KRL base on the part (user request).**
- **Part step:** new "KRL base (optional)" section, with one click on a flat face near a corner.
  - +Z is the face's outward normal.
  - +X runs along the straight edge nearest the click.
  - The origin is that edge's end nearest the click.
  - Code: `cad/part.py:krl_base_frame`, stored as `Part.krl_base` and `krl_base_face`.
  - Arrows are drawn by `PartView.show_frame`.
- **KRL output:** `to_krl(..., part_base)` writes LIN points relative to `placement @ krl_base` and selects `BASE_DATA[cell.part_base_number]` (new, default 2, in `cell.yaml`).
  - The header gives the expected BASE values: in the cell's WORLD with the table at 0 when not coupled, relative to the faceplate when coupled.
  - PTP joint values and the simulation are unchanged.
- **Export:** the Export step and the "Before the robot" check list follow it (`gui/checks.py`, `part_base=`).
- **Tests:** `tests/test_krl_base.py` passes, as do `test_krl.py` and the regression test.
- **Not done:**
  - The GUI has not been checked by hand.
  - `_part_key` ignores the base. That's intended: only the KRL changes.
  - No README section yet.
  - The user may later want other ways to define it (3 points, typed values). They chose "face and edge for now".
  - Picking happens in part-CAD with `vtkCellPicker.GetPickPosition()`. Check the click point is right when the section view is on.

**Next:**
1. **Done:** the full suite on `surface-patterns` (`1f9ec45`) passed 590 plus 3 xfails in 12 minutes.
   - That commit fixed a launch crash: the KRL base tooltip had raw newlines inside a string.
   - It also added `tests/test_imports.py`, which imports every module except `__main__`, because that one starts the app.
2. Check by hand in the app (`.\run`):
   - Surface mode with each pattern on the tube end;
   - pick a KRL base on `test_block` and save the KRL, then check the header and the `BAS(#BASE, 2)` line.
3. Merge with `--no-ff` and run the full suite.

## State (2026-10-27, end of session 3)
- **`main`:** every feature package is merged: constraints, patterns, contour tilt, Surface mode, purge, Simple/Advanced, turning points off the part, and the solver. Nothing is pushed.
- **Tests:** the last full suite passed 474 plus 3 xfails, before the session-3 work below. A fast run after S5 passed 495. Since then only the affected tests have run: jump height, Surface regions, test parts. **Run the full suite first.**
- **Not merged:**
  - `wp/P8`: Surface turn clearance. It works, but planning a tube with every side face selected went from 6.5 s to 31 s. **It also clashes with `a8cfe48`:** Surface turns now lift by jump height, at 0 not at all, as the user asked. Rework it so it only raises turns that are actually too close, and leaves jump height 0 alone, or drop it and let the solver's reroute lever handle colliding turns.
  - `wp/E2p`: Simple/Advanced for the Part step. Its new test `tests/test_part_mode.py` **hangs** on the first test. Stubbing `_refresh_steps` didn't fix it.
- **Obsolete branches:** `wp/WP0-wip` and `wp/M2-fix` (report only).
- **Housekeeping:** about 45 old worktrees are in `C:\Users\qrb21117\worktrees\`. Remove them with `git worktree remove`; stuck `.git\worktrees\<name>` folders clear with PowerShell `Remove-Item -Recurse -Force`.

## Done in session 3 (beyond the feature plan)
- **Gun angle:** the gun is held with the wrist above it, plus a user "Gun angle" (`planner/gun.py`). Auto position searches only tilt and E1.
- **Solver:**
  - The framework is `planner/solver.py`, with four levers:
    - arm configuration (`solve_arm`);
    - gun angle ±30° and lean ±10° (`solve_angle`);
    - re-routing approach, jump and purge moves (`solve_route`);
    - E1 per layer, within ±1000 mm (`solve_e1`).
  - The "Solve problems" button is in Simulate.
  - What can't be fixed is reported, and Save stays disabled.
- **E1 per move:** written to KRL as a PTP whenever it changes. The collision checker follows it, and raises an error if it goes beyond its cropped cell.
- **Markers menu:** hides collision, unreachable and steep-gun points.
- **Built-in test parts:** square, block with hole, round boss, stepped block, taper, U-block, tube, curved panel, repair and groove repair, all in a drop-down.
- **Jump height:** moves between layers now lift by jump height, not approach height. At 0 the gun moves between all paths at spray height. KRL hashes were updated.
- **Surface regions:** picked faces with a gap between them are separate regions, joined by a jump.
- **Speed:** a path check takes about 2.7 s on the test square.

## Waiting on the user (decide before building)
1. **Spray between layers:** it still switches off, with a settle wait each layer. Keep it on when jump height is 0?
2. **Crosshatch:** with the default 90° raster step it sprays two layers in a row in the same direction (0, 90, 90, 0). Change it?
3. **Position Simple mode:** E1, tilt and roll count as changed whenever they aren't 0, so they nearly always show. Compare E1 against `gantry.position_mm` instead?
4. **Purge:** with purge enabled the spray is on from the first purge visit to the last, including between layers. Confirm. The purge `spray_dir` in `cell.yaml` is still marked CHECK.
5. **Usage:** use `.claude/agents/worker.md` (Sonnet) for agent runs? Also, should `CLAUDE_CODE_SUBAGENT_MODEL=sonnet` go in `~/.claude/settings.json`?

## To do, in order
1. Run the full suite on `main`, then the human check in the app (`.\run`):
   - gun angle;
   - Markers;
   - test parts;
   - jump height 0;
   - Surface mode with separate faces;
   - Solve problems on `test_repair` in Surface mode.
2. **Collisions found by P7:** A2/A3 against the "Filtration unit" along the whole path, plus the faceplate and fixture. Re-check them in Simulate after Auto position. The cause may be the cell model or crop rather than the path. See `P7-1.md`.
3. **Fix `wp/E2p`'s hanging test, then merge.**
4. **Speed up `wp/P8`, then merge.** The solver's own slow tests also need work: `test_solve_e1` takes minutes. Mark them `slow` or speed up `path.sample`, which is half the check time.
5. **Solver gaps:**
   - no cancel inside a single lever;
   - `export.py` confirm list lacks the solver rows;
   - the collision checker can't follow E1 more than 1000 mm.
6. **Constraints panel:**
   - picking faces in 3D;
   - highlight the selected row's face;
   - save constraints with the Part step choices.
7. **Docs:**
   - README for everything in session 3;
   - refresh plan §5 and §8 and `ROADMAP.md`, or delete them.
8. **Robot cell checks (README "Before running on the real robot"):**
   - purge spray direction;
   - collision results for purge transits, which run with the spray on;
   - T1 runs for wrist flips with contour tilt and Surface mode;
   - how E1 is coupled on the controller.

## Agent protocol (paste into every agent prompt)
```
Repo C:\Users\qrb21117\OneDrive - University of Strathclyde\Documents\Cold-Spray-Kuka-Control. Timebox 10-20 min; stop coding 3 min before.
git worktree add C:\Users\qrb21117\worktrees\cs-<ID> -b wp/<ID> main
cmd /c mklink /J C:\Users\qrb21117\worktrees\cs-<ID>\assets "<repo>\assets"
Test: $env:PYTHONPATH="C:\Users\qrb21117\worktrees\cs-<ID>\src"; C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pytest -q -p no:cacheprovider <files>
GUI tests offscreen; stub MainWindow._refresh_steps (cell assets never load off screen); cap long runs (Start-Process + WaitForExit).
Test first, minimal diff, edit only owned files. Commit on wp/<ID>, trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". No push, no main.
Report plans/reports/<ID>-1.md: status, done, not done, test line. Return: sha + test line + 1 line.
```
**Models:**
- Sonnet (the `worker` agent) for contained, single-module tasks.
- Opus for work across threading, the interface and result flow. S5's Sonnet draft had 7 bugs.

**Merging:**
- `--no-ff`;
- keep both sides on conflict;
- run the touched tests plus `tests/test_krl_regression.py`;
- run the full suite afterwards.
