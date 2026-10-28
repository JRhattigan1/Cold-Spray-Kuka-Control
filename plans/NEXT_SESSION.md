# Next session

**Start here.** Where each feature stands is in [ROADMAP.md](ROADMAP.md), and what the app does is in [README.md](../README.md). Each agent's detail is in `plans/reports/<ID>-1.md`. Older hand-offs are in git history.

## State (2026-10-28, end of session 5)
- **`main`:** sessions 1–4 (`surface-patterns` merged at `a987069`, work in progress). Nothing from session 5 is merged yet; INT merges, D0 (these docs) is merged by the user.
- **Nothing is pushed.** Session 5 worktrees: `C:\Users\qrb21117\worktrees\cs-<ID>`; brief and check-ins in `worktrees\run5\`.
- **KRL safety (review findings 1–8 in `run5\BRIEF.md`):**
  - **K1** `wp/K1` `e24f730`: O5 arcs rebased on `main`. Arc fit checks the chord sagitta, 2 m radius cap, `to_krl(joints=)`, CIRC with an E1 change is a `KRLError`, a turning uncoupled table keeps LINs, `solve_angle` tilts `aux`. 21 passed (krl/regression/gun/sample_speed). **HELD:** the regression hashes were re-baselined without a reviewed diff, and CIRC `#PATH` orientation and `C_DIS` between CIRCs are unverified on the real robot.
  - **K2** `wp/K2` `4d93c11`: `post/joints.py` `program_joints` gives PTP joints as Simulate does (finding 1); E1 only on gun-off moves (5); plan key includes the robot side (6). Solver 7 + solve_ui 5 passed.
  - **K3** `wp/K3` `48c6402`: "flip" check row, joint jump > `JUMP_DEG` inside a LIN/CIRC (2); collision check follows the configured gantry when axes has no E1 (4); `allowed_text()` for the log (7b). 23 passed.
  - **K4** `wp/K4` `18309a8`: `post/krl_split.py` (subprograms), `post/krl_preflight.py` (TOOL/BASE guard), `post/krl_status.py` (S/T). All off; `krl.py` untouched. 7 passed.
  - **KM** `wp/KM` `a669e44`: K4, K2, K3 merged; `plans/KM-options.patch` wires `options=` into `krl.py` (preflight off by default). **`wp/KM-K1`** `469d15c`: K1 merged plus the K2 joints hook and `options=` on K1's `krl.py`. **HELD** with K1.
- **Speed:**
  - **OP1** `wp/OP1` `6461806`: cell npz meshes memoised, gun mesh loads in parallel, lazy trimesh in `cad/part.py` and `planner/revolve.py`. Timings are noise (machine overloaded). Still top-level: trimesh in `cad/slicer.py`, `planner/surface.py` (+ cKDTree).
  - **OP2** `wp/OP2` `87810c4`: test_tube Layers 85.4 → 19.1 s (vectorised `_order`, constant down-frame); all 7 cases identical to the `tools/bench_plan.py` baseline. 96 passed. Next: `raster_segments`.
- **UI (scaling, positions, dark mode):**
  - **U20** `wp/U20` `437fae5` (code `976e56e`): opaque field and sidebar fills, ink text. **Tests not run.** Primary button text still off-screen; inputs white-on-white in dark (U22 ask).
  - **U21** `wp/U21` `3ad1bd9`: sidebar/3D splitter, sidebar 460 (min 400, sized to panels), window min 1100x680, start 1440x900 capped to the screen, pinned Next footer, sidebar width in QSettings, short mode labels. `tests/test_gui_layout.py` 8 tests.
  - **U22** `wp/U22` `f0869e0`: dark 3D gradient from `style.view_background`, markers/toolbars/view bars follow the theme, `part_view.py` too. 79 + 3 passed.
  - **U23** `wp/U23` `a383efd`: `fit_panel` on the step panels, 2x2 stat tiles, timeline legend fits, `allowed_text` logged. Asks: check-row gaps (`checks.py:214`), dark review.
  - **U24** `wp/U24` `372fea7`: `tools/ui_screenshots.py --size WxH --scale S`, offscreen by default. Shots in `run5\shots\U24\`. Offscreen glyphs are boxes and the 3D view is blank: judge layout only.
  - **FM** (sidebar-fit team): **in progress**, nothing committed (`wp/FM` = `main`).
- **Older branches still not merged:** `wp/O5` (superseded by `wp/K1`), `wp/E2p` (test hangs), `wp/P8` (slow; rework or drop).

## Check by hand in the app (`.\run`)
1. **Surface on the tube end:** each pattern. Concentric should give clean rings.
2. **KRL base:** pick one on `test_block`, save the KRL, check the header and the `BAS(#BASE, 2)` line.
3. **Fixed in the cell:** move a part off the table, Generate, Simulate.
4. **Gun hold:** a concentric Layers job with the table flat. The gun must not turn with the loops.
5. **Constraints:** Pick, then click a face (only stubbed in tests), Esc to cancel, purple highlight.
6. **Suggest purge point**, then reach and collisions in Simulate.
7. **Generate and Auto position stay responsive:** Cancel works, and Generate after Auto position is instant (cache).
8. **WearPlate2** (`Downloads/WearPlate2.stp`): Surface on the top, the angled face and the blends.
9. **Session 5 UI:** resize the window, drag the splitter, light and dark, 100 % and 150 % scaling.

## To do, in order
1. **Machine load:** at most about 6 agents at once; app runs and `ui_screenshots.py` jobs one at a time (session 5 pytest runs were killed and timings were noise).
2. **Merge session 5:** K2, K3, K4 (via `wp/KM`), OP1, OP2, U21–U24, then U20 once its tests run. Full suite after.
3. **KRL hook state:** `krl.py` on `main` still seeds PTP joints with `robot.ik`. The K2 `program_joints` hook and `options=` exist only on `wp/KM-K1` (with K1) and as `plans/KM-options.patch`. Apply the hook without K1 if K1 stays held; PTP lines in the regression files will change (review the diff).
4. **K1 arcs (held):** review the KRL diff behind the new hashes, then T1 on a fillet: CIRC `#PATH` orientation and `C_DIS` between CIRCs.
5. **Review finding 3, open:** PTP moves are interpolated in Cartesian space in `Toolpath.sample`, and HOME → first and last → HOME are not simulated. Sample PTPs in joint space (`ptp_joint_path`, never written) and check those legs.
6. **Review finding 7a, open:** the gantry carriage is not in the collision environment.
7. **Review finding 8, open:** `Toolpath.sample` uses the last move's axis value where KRL uses the configured value.
8. **S/T bits** (`post/krl_status.py`): S bit0 (overhead) and bit1 (elbow) are UNVERIFIED defaults. Compare with `$POS_ACT` on the arm before writing S/T on the first LIN after a PTP.
9. **"90Â°" mojibake** in the 3D HUD labels (`sim/viewer.py:594`, `:625`): the source has `Â°`; write `°`.
10. **UI left:** primary button text overflow (U20), dark inputs white-on-white, check-row gaps (`checks.py:214`), faint step-bar done labels, a dark screenshot review.
11. **Speed:** lazy trimesh/cKDTree in `cad/slicer.py` and `planner/surface.py`; `raster_segments` vectorised; reach IK in `Scene.set_path` on a worker.
12. **Simulate check for cable wind-up:** A4/A6 travel against a limit in `cell.yaml`.
13. **Surface:** split a wrapping region per CAD face; the zigzag sprays across holes.
14. **Constraints:** save them with the Part step choices; pick references in 3D.
15. **`wp/E2p` and `wp/P8`:** fix and merge, or drop.
16. **Solver gaps:** cancel inside a lever; solver rows in the export confirm; collision following E1 beyond 1000 mm.
17. **Collisions found by P7:** A2/A3 against the "Filtration unit" (`reports/P7-1.md`).
18. **Housekeeping:** about 75 worktrees in `C:\Users\qrb21117\worktrees\` and the merged `wp/*` branches. Remove them with `git worktree remove` and `git branch -d`. Stuck `.git\worktrees\<name>` folders clear with PowerShell `Remove-Item -Recurse -Force`.
19. **Robot cell checks:** README "Before running on the real robot", in T1 first.

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
- The `worker` agent type (`.claude/agents/worker.md`) wasn't loaded this session. Use `general-purpose` with `model: sonnet` for contained single-module tasks.
- Use `model: opus` for work across threading, the interface and result flow, or tricky maths (IK, arcs).

**Merging:**
- `--no-ff`;
- review the diff;
- run the touched tests plus `tests/test_krl_regression.py` and `tests/test_imports.py`;
- run the full suite afterwards.

**Stuck agent:** stop it, check its worktree, and commit its work in progress with a partial report, as was done for O5.
