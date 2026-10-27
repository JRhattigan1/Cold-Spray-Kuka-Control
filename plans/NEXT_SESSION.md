# Next session

**Start here.** Where each feature stands is in [ROADMAP.md](ROADMAP.md), and what the app does is in [README.md](../README.md). Each agent's detail is in `plans/reports/<ID>-1.md`. Older hand-offs are in git history.

## State (2026-10-27, end of session 4)
- **`main`:** sessions 1–3.
- **`surface-patterns`:** checked out. It is `main` plus everything from session 4 and the optimisation run. **Not merged into `main`.**
  - Full suite: **619 passed, 3 xfailed** (5 min) at `0088be9` + docs.
- **Nothing is pushed.**
- **What `surface-patterns` adds** (the README describes each in detail):
  - Surface mode with any infill pattern, on regions that face one way.
  - KRL base picked on the part: `BASE_DATA[part_base_number]`, default 2.
  - Auto position only from its button; planning errors show a message.
  - Gun hold with minimal twist: no winding, for the cables and powder line.
  - `raster_start_deg`.
  - "Mounted: Fixed in the cell": the table is held at 0.
  - Speed-ups, each verified identical:
    - gun hold 12.8 → 0.8 s;
    - sampling 17.5 → 0.4 s;
    - IK 24 → 3.3 s;
    - collision FK 3.25 → 1.65 s.
  - Generate and Auto position run on a worker thread, with a plan cache (`gui/plan_jobs.py`; tests call `plan_jobs.wait(win)`).
  - Suggest purge point (filtration unit).
  - Constraint faces picked in the 3D view.
  - `tests/test_imports.py`, which imports every module (it caught two launch crashes).
- **Not merged:**
  - **`wp/O5`:** arcs (CIRC) and merged LINs. Partial, stopped by the user. See `reports/O5-1.md`.
    - Works: simplify, CIRC sampling, KRL `CIRC` with `$CIRC_TYPE = #PATH`.
    - test_tube Surface goes from 40,893 to 1,581 moves.
    - Left:
      - check the programs, then update the regression hashes;
      - fix the `test_separate_faces…` assumption;
      - simplify tests;
      - solver levers must keep `aux` consistent;
      - full suite;
      - T1.
  - **`wp/E2p`:** Simple/Advanced for the Part step. Its `tests/test_part_mode.py` hangs on the first test.
  - **`wp/P8`:** Surface turn clearance. It is slow (6.5 → 31 s) and clashes with "turns lift by jump height, 0 = straight across". Rework it to raise only turns that are too close, or drop it and let the solver re-route.
- **Obsolete branches:** `wp/WP0-wip`, `wp/M2-fix`, and every merged `wp/*`.

## Liquid Glass UI overhaul: merged on `surface-patterns`, NEEDS FIXING FIRST
- **What was done:**
  - Code consolidation `91ecd77`, full suite 619 passed: one OPW forward/inverse implementation (batch), `Robot.link_frames_batch`, `cell.base_worlds`, no config getattr fallbacks, `Toolpath.zigzag_regions`.
  - 13 UI agents merged, U1–U13 except U2 (step bar), which was still running at the end. Its branch is `wp/U2`.
  - `tools/ui_screenshots.py OUTDIR [--theme dark]` renders every step to PNG. Use it to check the look.
- **Seen in the light screenshot of the Toolpath step:**
  - Spin boxes show no values: text colour or glass fill in the U1 QSS or the U11 card.
  - The Generate / Next buttons are orange with no text (U3 `primary_button` paint).
  - The sidebar contents are cut off on the right; the panel is too narrow for the new margins (U7 sidebar card).
  - The strategy segments are clipped ("ayers (build up…"); they need short labels Layers / Revolve / Surface (U11).
  - The sidebar has a yellowish tint (U7 alpha fill on the panel colour).
  - Fixed: `StatTiles` lacked `clear()`, which crashed on loading a part (`cafed19`).
- **Dark mode is wanted too (user).**
  - The 3D gradient from U6 is hard-coded light; it should come from `style.view_background(mode)`.
  - U4 toolbars and U13 markers read colours once and don't follow theme changes.
  - Dark screenshots were taken but not reviewed.
- **Not run since the UI merges:** the full suite. Run it, plus a by-hand check in the app, before anything else.
- **Housekeeping:** about 60 worktrees and the merged `wp/*` branches, the UI ones included. A dry run of their removal was started but nothing was deleted.

## Check by hand in the app (`.\run`) before merging `surface-patterns`
1. **Surface on the tube end:** each pattern. Concentric should give clean rings.
2. **KRL base:** pick one on `test_block`, save the KRL, check the header and the `BAS(#BASE, 2)` line.
3. **Fixed in the cell:** move a part off the table, Generate, Simulate.
4. **Gun hold:** a concentric Layers job with the table flat. The gun must not turn with the loops.
5. **Constraints:** Pick, then click a face (only stubbed in tests), Esc to cancel, purple highlight.
6. **Suggest purge point**, then reach and collisions in Simulate.
7. **Generate and Auto position stay responsive:** Cancel works, and Generate after Auto position is instant (cache).
8. **WearPlate2** (`Downloads/WearPlate2.stp`): Surface on the top, the angled face and the blends.

## To do, in order
1. **Check the list above, then merge `surface-patterns` into `main`** (`--no-ff`) and run the full suite.
2. **Finish `wp/O5`** (user request: arcs instead of many small LINs). Rebase it on the merged branch first.
3. **Speed, left over from `OPTIMISATION_PLAN.md`:**
   - reach IK in `Scene.set_path` on a worker;
   - Layers planning for tall parts (WearPlate2 Layers: 429 s, 214,557 moves; merged LINs will help).
4. **Simulate check for cable wind-up:** A4/A6 travel against a limit in `cell.yaml`.
5. **Surface:**
   - split a wrapping region per CAD face, so the end face gets the pattern and the sides zigzag;
   - the zigzag sprays across holes.
6. **Constraints:** save them with the Part step choices; pick references in 3D.
7. **`wp/E2p` and `wp/P8`:** fix and merge, or drop.
8. **Solver gaps:**
   - cancel inside a lever;
   - solver rows in the export confirm;
   - collision following E1 beyond 1000 mm.
9. **Collisions found by P7:** A2/A3 against the "Filtration unit" along the whole path. Re-check after Auto position; it may be the cell model or the crop (`reports/P7-1.md`).
10. **Housekeeping:** about 50 worktrees in `C:\Users\qrb21117\worktrees\` and the merged `wp/*` branches. Remove them with `git worktree remove` and `git branch -d`. Stuck `.git\worktrees\<name>` folders clear with PowerShell `Remove-Item -Recurse -Force`.
11. **Robot cell checks:** README "Before running on the real robot", in T1 first, including CIRC orientation once O5 is merged.

## Waiting on the user
1. **Spray between layers:** it still switches off, with a settle wait each layer. Keep it on when jump height is 0?
2. **Crosshatch:** with the default 90° step it repeats a direction (0, 90, 90, 0). Change it?
3. **Position Simple mode:** E1, tilt and roll count as "changed" whenever they aren't 0. Compare E1 with `gantry.position_mm` instead?
4. **Purge:** confirm the spray stays on from the first purge visit to the last, including between layers. `spray_dir` in `cell.yaml` is still marked CHECK.
5. **Surface tube end:** did "covering the whole area" mean the zigzag spraying across the bore?
6. **Wrapping regions:** split them per CAD face, as in to-do 5?

## Agent protocol (paste into every agent prompt)
```
Repo C:\Users\qrb21117\OneDrive - University of Strathclyde\Documents\Cold-Spray-Kuka-Control. Timebox 10-20 min; stop coding 3 min before.
git worktree add C:\Users\qrb21117\worktrees\cs-<ID> -b wp/<ID> <base branch: surface-patterns until it is merged, then main>
Assets junction from PowerShell (Git Bash mklink mangles the path): cmd /c mklink /J C:\Users\qrb21117\worktrees\cs-<ID>\assets "<repo>\assets"
Test: $env:PYTHONPATH="C:\Users\qrb21117\worktrees\cs-<ID>\src"; C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pytest -q -p no:cacheprovider <files>
GUI tests offscreen; stub MainWindow._refresh_steps (cell assets never load off screen); plan_jobs.wait(win) after generate/_auto_position; cap long runs.
Benchmark on small built-in parts (test_tube, test_block, test_repair), not WearPlate2 (too slow).
Use absolute worktree paths for every edit (an agent once wrote into the main checkout by mistake).
Test first, minimal diff, edit only owned files. Commit on wp/<ID>, trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". No push, no main.
Report plans/reports/<ID>-1.md: status, done, not done, test line. Return: sha + test line + 1 line.
```
**Models:**
- The `worker` agent type (`.claude/agents/worker.md`) wasn't loaded this session. Use `general-purpose` with `model: sonnet` for contained single-module tasks.
- Use `model: opus` for work across threading, the interface and result flow, or tricky maths (IK, arcs).

**Merging:**
- `--no-ff`;
- review the diff;
- run the touched tests plus `tests/test_krl_regression.py` and `tests/test_imports.py`;
- run the full suite afterwards.

**Stuck agent:** stop it, check its worktree, and commit its work in progress with a partial report, as was done for O5.
