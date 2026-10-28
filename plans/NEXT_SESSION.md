# Next session

**Start here.** Where each feature stands is in [ROADMAP.md](ROADMAP.md), and what the app does is in [README.md](../README.md). Each agent's detail is in `plans/reports/<ID>-1.md` on its branch (`git show wp/<ID>:plans/reports/<ID>-1.md`; the session 5 worktrees are removed). Older hand-offs are in git history.

## Session 9 hand-over (2026-10-28) — READ FIRST
User asked: fix known bugs, add day-to-day features, housekeeping (robot T1 not possible yet). Session ended mid-run.
- **Done on main:** `246e8c3` CI workflow `.github/workflows/tests.yml` (windows-latest, KRL hashes + imports + planner/post units; 365 passed locally in 2 min). **Not pushed yet**: push main after merging.
- **Branches deleted:** remote `wp/E2p`, `wp/O5`, `wp/P8`; local `wp/S8I`, `wp/FM`, `surface-patterns`.
- **Cleanup left for the user (blocked by the permission check):** `git branch -D wp/KM-K1` (obsolete, unmerged); worktrees `cs-S8B`, `cs-U20` hold only throwaway untracked files (assets junctions already removed): `git worktree remove --force` both, then `git branch -d wp/S8B wp/U20`; pause OneDrive and `git worktree prune` (stale `cs-S8*` records). Remote `surface-patterns` still exists (merged; delete if wanted).
- **Agents launched (wave 1+2), each on `wp/<ID>` in `C:\Users\qrb21117\worktrees\cs-<ID>`, report `plans/reports/<ID>-1.md`.** Check each branch: report present? tests? then review + merge `--no-ff` (protocol below).
  | ID | task | main files |
  |---|---|---|
  | S9S | Surface step-over by distance along the surface (to-do 5) + repair patch far-side reach by lowering lift between faces (to-do 2; un-xfail) | planner/surface.py |
  | S9J | Job save/open (`job_io.py`, Ctrl+S/O, recent jobs) | job_io.py, gui/app.py |
  | S9H | HOME legs in solver; HOME→first checked as the real LIN/PTP; PTP drawn along joint path; finding 8 | planner/solver.py, sim/collision.py, viewer path drawing |
  | S9T | Predicted thickness map (`sim/deposit.py`), Simulate toggle; cycle time + powder (`powder_feed_g_min`) in Simulate and Export | sim/deposit.py, viewer colouring, steps/simulate.py, export |
  Overlap: S9H and S9T both touch `sim/viewer.py` (different functions). If an agent died mid-run, its worktree has uncommitted work: commit it as WIP and finish inline.
- **Wave 3, not started:** (a) undo for part placement and constraint edits; (b) spray on/off from KRL `$OUT` (option, off by default; default KRL hashes must not change); (c) A4/A6 cable wind-up check in Simulate; (d) filtration-unit collisions (P7). (a) touches gui/app.py: start after S9J is merged.
- **After merging:** full suite; rewrite `ROADMAP.md` (still describes session 5); update this file; push main.

## State (2026-10-28, session 7)
- **`main` @ `7efb1f8`**, all session 6/7 work merged; nothing pushed. **Full suite** (before S7O/S7S/startup fix): 703 passed, 1 failed (fixed: `d3795b7`), 3 xfailed, 18 min. After the last merges, touched tests: 141 passed, 1 xfailed.
- Merged this session (`git log --first-parent main`):
  | merge | what |
  |---|---|
  | S6P2 | purge sprays +Y (`CellConfig.back_dir`) |
  | S7Q | gun on between layers at jump 0; crosshatch removed (old configs load as zigzag); `gun_max_down_deg` 70 (warning); solver logs "trying <lever>"; purge → start rises over the part |
  | S7K (+S6C) | gantry carriage in collision; E1 agrees Simulate/KRL (`Toolpath.sample(initial=)`, `cell.axis_defaults`); moved-E1 crash; NaN PTP seed warning; PTP + HOME legs in joint space and collision-checked; **no part collisions in Layers** (`collision.part_for_mode`) |
  | S7M (+S6M, F3, F1, F2fit, U20, S6E, S6A, S6B, S6F) | panels ≤ 398 px with the real stylesheet; mojibake; purge actions dropdown; crisp step-bar; SpaceMouse name in tooltip |
  | S7A (K1) | CIRC arcs + merged LINs behind `ProcessConfig.arcs`, **off by default** (default KRL unchanged) |
  | S7G | solver levers "gantry" (one whole-program E1, Apply button) and "purge" (point moved ±200 mm XYZ, angle kept); purge clearance on every trip |
  | S7O | Layers overrun measured from the part outline, not the inset infill |
  | S7S | wrapping Surface pick split per CAD face (own raster, gun off between); oneway overrun from the face edge |
- Branches left: `wp/FM`, `wp/KM-K1` (obsolete, delete), `wp/U20` + worktree `cs-U20` (merged; holds 12 untracked `shots_before` PNGs), `surface-patterns` (merged). Remote: `origin/wp/E2p`, `O5`, `P8` still exist.

## Run 8: Solve problems speed (2026-10-28, merged `15c0b74`)
- 10 agents (S8B bench, S8T golden test, S8F solver caches, S8C collision, S8R route/e1, S8G gantry/purge, S8A arm, S8N angle, S8I profile only, S8M merger).
- `tools/bench_solve.py` (`--only`, `--json`, `--compare`, `--profile`); `tests/test_solver_golden.py` guards solver output.
- Solve only, collision on: block_layers 29.0 → 24.8 s; repair_surface 115.5 → 42.9 s; total 144.6 → 67.7 s. Output IDENTICAL to main on both.
- Still slow: block_layers **reroute** (19 s; IK over the whole range per candidate; sample/IK only the changed span) and repair_surface **gantry** (30 s; FCL narrow phase; split the cell mesh into chunks so the broadphase culls). IK `ik_path` per-sample loop (S8I). tube_surface not benchmarked (about 25k issues).
- `.git\worktrees\cs-S8*` records locked by OneDrive: pause OneDrive, `git worktree prune`. `wp/S8B`, `wp/S8I` (report only) and worktree `cs-S8B` (2 baseline JSONs) are left.

## To do, in order
1. **By-hand check:** [HAND_CHECK.md](HAND_CHECK.md).
2. **Decide (user):** repair patch top + 4 sides now sprays the sides; 10/482 moves on the far side are out of reach (lifted turns, lift between faces). `test_surface_e2e::test_repair_patch_top_and_sides` is xfail. Options: accept; lower lift between faces (jump height); old tilt limit for split faces.
3. **Arcs in T1** (test_block, arcs on): orientation along CIRC with `#PATH`, C_DIS blending, `$VEL.CP`, `$CIRC_TYPE` inside split `_Pnn`. CIRC on a turning table is a 2-LIN chord (short arcs only).
4. **Gaps left by S7K:** HOME legs not in solver `find_issues`; HOME → first checked as PTP even when move 0 is a LIN; drawn path shows PTP as a straight line; `solve_arm` uses Cartesian samples; no GUI test for the Layers part gate.
5. **Surface step-over on curved transitions (user, 2026-10-28):** where a flat face runs into a curved blend, the passes bunch up on the curve and stay evenly spaced on the flat ([screenshot](images/surface_stepover_curve.png)); problem markers cluster there too. Step-over seems set in the flattened / plane-cut frame, not measured along the surface. Fix: space passes by distance along the surface (geodesic, or re-space cuts by arc length across the curve) so the track overlap is constant; add a test on a flat-plus-fillet pick.
6. **Gantry lever speed:** up to ~32 whole-path checks; measure on WearPlate2 with collision on.
7. **Position Simple mode:** compare E1 with `gantry.position_mm` for "changed" (user: maybe).
8. **Review finding 8:** `Toolpath.sample` axis value (partly covered by `initial=`; re-check).
9. Slicer `polygonize` option; launch time; Export split/preflight on the controller; S/T bits vs `$POS_ACT`; cable wind-up A4/A6; holes in Surface; saved constraints; solver gaps; P7 filtration-unit collisions; reach IK on a worker; README robot checks in T1.

## User decisions (2026-10-28)
- Spray stays on between layers at jump 0; crosshatch removed; purge spray on first→last visit (spray is manual for now).
- Gun angle limit 70° as a warning. Back of the cell (filtration) is +Y.
- Layers: the part is not a collision body (not built yet); Revolve/Surface: it is.
- Purge → start avoids the part. Solver suggests gantry E1 and moves purge XYZ, not its angle.
- Overrun is measured from the body's faces, not the toolpath.
- Wrapping Surface regions split per CAD face.
- **Fewer agents** (user, session 7): do work inline; use an agent only for a big separate piece.

## Lessons for running agents
- Keep agents few (user, session 7); the earlier crashes came from on-screen app windows, so app, screenshot and GUI-test runs stay offscreen.
- App and screenshot runs are offscreen and one at a time.
- Agents' clocks drift: use `date`, not their own sense of time.
- A manager per team (KM, FM) worked: one plan, disjoint files, one merger.
- A hook that crosses branches (e.g. K2's `krl.py` hook on K1's file) needs one named owner.
- No agent may edit the main checkout. One did, and it blocked a merge. Check `git status` on main before merging.
- **Cleanup:** each worktree's `assets` is a junction to the real `assets\` (licensed CAD, not in git). Remove the junction with `cmd /c rmdir <wt>\assets` before `git worktree remove` (no `--force`). Never `Remove-Item -Recurse` or `rm -rf` a worktree folder.
- Per-run files (brief, check-in log, INT and CLEANUP reports, screenshots) live in `C:\Users\qrb21117\worktrees\run5\`.

## Agent protocol (paste into every agent prompt)
```
Repo C:\Users\qrb21117\OneDrive - University of Strathclyde\Documents\Cold-Spray-Kuka-Control. Timebox 10-20 min; stop coding 3 min before.
git worktree add C:\Users\qrb21117\worktrees\cs-<ID> -b wp/<ID> main
Assets junction from PowerShell (Git Bash mklink mangles the path): cmd /c mklink /J C:\Users\qrb21117\worktrees\cs-<ID>\assets "<repo>\assets"
Test: $env:PYTHONPATH="C:\Users\qrb21117\worktrees\cs-<ID>\src"; C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pytest -q -p no:cacheprovider <files>
GUI tests offscreen; stub MainWindow._refresh_steps (cell assets never load off screen); plan_jobs.wait(win) after generate/_auto_position; cap long runs.
Benchmark on small built-in parts (test_tube, test_block, test_repair), not WearPlate2 (too slow).
Use absolute worktree paths for every edit (an agent once wrote into the main checkout by mistake).
Test first, minimal diff, edit only owned files. Commit on wp/<ID>, trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". No push, no main.
Report plans/reports/<ID>-1.md: status, done, not done, test line. Return: sha + test line + 1 line.
```
**Models:**
- The `worker` agent type (`.claude/agents/worker.md`, Sonnet) loaded in session 5. Use it for contained single-module tasks; otherwise `general-purpose` with `model: sonnet`.
- Use `model: opus` for work across threading, the interface and result flow, or tricky maths (IK, arcs).

**Merging:**
- `--no-ff`;
- review the diff;
- run the touched tests plus `tests/test_krl_regression.py` and `tests/test_imports.py`;
- run the full suite afterwards.

**Stuck agent:** stop it, check its worktree, and commit its work in progress with a partial report, as was done for O5.
