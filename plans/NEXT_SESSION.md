# Next session

**Start here.** This replaces the old session hand-offs and run-sheet; git history has them. The feature plan is `plans/2026-10-27-feature-plan.md`. Its §5 tracker, §8 run log and `plans/ROADMAP.md` are **out of date**, so trust this file. Each agent's detail is in `plans/reports/<ID>-1.md`.

## State (2026-10-27, end of session 3)
- **`main`:** every feature package is merged: constraints, patterns, contour tilt, Surface mode, purge, Simple/Advanced, turning points off the part, and the solver. Nothing is pushed.
- **Tests:** the last full suite passed 474 plus 3 xfails, before the session-3 work below. A fast run after S5 passed 495. Since then only the affected tests have run: jump height, Surface regions, test parts. **Run the full suite first.**
- **Not merged:**
  - `wp/P8`: Surface turn clearance. It works, but planning a tube with every side face selected went from 6.5 s to 31 s. It needs a coarse pre-check before merging.
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
