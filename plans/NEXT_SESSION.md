# Next session

**Start here.** Where each feature stands is in [ROADMAP.md](ROADMAP.md), and what the app does is in [README.md](../README.md). Each agent's detail is in `plans/reports/<ID>-1.md` on its branch (`git show wp/<ID>:plans/reports/<ID>-1.md`; the session 5 worktrees are removed). Older hand-offs are in git history.

## Session 9 (2026-10-28): all merged
User asked: fix known bugs, add day-to-day features, housekeeping (robot T1 not possible yet). Everything is merged to main; see ROADMAP.md (rewritten).
| merge / commit | what |
|---|---|
| `246e8c3` | CI: `.github/workflows/tests.yml` (windows-latest; KRL hashes, imports, planner/post units) |
| S9S | Surface step-over measured along the surface (`arc_map`); repair patch reach fixed (turns keep the next stroke's roll; lift unchanged); xfail removed |
| S9J (+`a2962f7`, `eca9182`) | Job files `*.csjob` (`job_io.py`, `gui/job_menu.py`), Ctrl+S / Ctrl+O, recent jobs; saves export options; opening resets undo |
| `f1c114b` | Export: **Pre-flight and Split were not wired into Save** (fixed); new "Program switches spray" option (off = `$OUT` lines as comments, timing unchanged; split still tracks them) |
| `426cb89` | A4/A6 cable wind-up warning (`cell.cable_windup_deg`, 270° placeholder: CHECK) |
| S9H | HOME legs in solver `find_issues`; HOME→first checked as the KRL moves it (LIN or PTP); PTP drawn along the joint path; finding 8: no change needed |
| S9U | Undo/redo of Position edits (`gui/undo.py`), Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y, ↶ ↷ in the step bar |
| S9T | Thickness map (`sim/deposit.py`), Simulate toggle; cycle time + powder (`powder_feed_g_min`, 30 g/min placeholder) in Simulate and Export |
| S9F | Filtration-unit hits explained: cell model correct; greedy IK picks **elbow-down** (A2 +10°) so the elbow enters the unit; the arm lever fixes it (0 hits after Solve). `_follow_e1` 1e-6 tolerance fix (solver crash at exactly 1000 mm E1 shift) |

**S9F findings to act on (product decisions):**
- **HOME collides at E1 2730** (cell.yaml `gantry.position_mm`): nozzle through the faceplate into the rotary base; at E1 2000 the gun touches the faceplate. Check on the real cell; the HOME legs now flag it.
- **E1 5500** (upper limit): arm and gun cross the Booth wall (root X -3654, wall ≈ -3894). Check the stroke limit or wall placement.
- Example purge point (-884.56, 400, 2000) is 37 mm from the filtration unit face: use Suggest purge point (150 mm standoff).
- `ik_path` ranks IK by max joint change from home → elbow-down. Weighting A2/A3 (or summing) would start paths elbow-up without Solve, but changes KRL hashes and the golden: user decision.
- Repair Surface: 657 greedy hits (wrist/gun vs tilted faceplate): needs collision-aware auto-position tilt/roll. Gantry lever scores at 5 mm sampling and disagrees with the 2 mm check.

**Open from session 9:** thickness map runs on the GUI thread (~4 s on test_tube); undo does not cover purge edits; HOME legs not drawn; no lever fixes HOME-leg problems directly.

**Cleanup left for the user (blocked by the permission check):** `git branch -D wp/KM-K1`; `git worktree remove --force` on `C:/Users/qrb21117/worktrees/cs-S8B` and `cs-U20` (throwaway untracked files only), then `git branch -d wp/S8B wp/U20`; delete merged `wp/S9*` branches and worktrees (remove each `assets` junction with `cmd /c rmdir` first); pause OneDrive and `git worktree prune`. Remote `surface-patterns` still exists (merged).

## To do, in order
1. **By-hand check:** [HAND_CHECK.md](HAND_CHECK.md) (add: Job save/open, undo, thickness toggle, estimates, Export spray-by-hand and split).
2. **Decide (user):** S9F findings above (HOME at E1 2730, E1 5500 wall, purge example, elbow-up IK weighting).
3. **Arcs in T1** (test_block, arcs on): orientation along CIRC with `#PATH`, C_DIS blending, `$VEL.CP`, `$CIRC_TYPE` inside split `_Pnn`. CIRC on a turning table is a 2-LIN chord (short arcs only).
4. **Calibrate:** thickness model and `powder_feed_g_min` against a sprayed coupon; `cable_windup_deg` from the cable dressing.
5. **Collision-aware auto-position** (repair Surface hits); gantry lever sampling vs the 2 mm check; gantry lever speed on WearPlate2.
6. Thickness map on a worker thread; HOME legs drawn; undo for purge edits.
7. **Position Simple mode:** compare E1 with `gantry.position_mm` for "changed" (user: maybe).
8. Slicer `polygonize` option; launch time; S/T bits vs `$POS_ACT`; holes in Surface; reach IK on a worker; `solve_arm` uses Cartesian samples; README robot checks in T1.

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
