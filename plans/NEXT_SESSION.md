# Next session

**Start here.** Where each feature stands is in [ROADMAP.md](ROADMAP.md), and what the app does is in [README.md](../README.md). Each agent's detail is in `plans/reports/<ID>-1.md` on its branch (`git show wp/<ID>:plans/reports/<ID>-1.md`; the session 5 worktrees are removed). Older hand-offs are in git history.

## State (2026-10-28, session 6 mini run)
Branches are unmerged unless stated; none is pushed. Per-run files are in `C:\Users\qrb21117\worktrees\run6\`.
| agent | branch | sha | result |
|---|---|---|---|
| S6M | `wp/S6M` | `76bf656` | Fit merge (F3, F1, F2fit, U20): merges clean; 87 passed, 2 failed (capsule test, order-dependent fit test). |
| S6T | (none) | n/a | Full suite on `main` @ `3120475`, not slow, `-x`: 598 passed, 0 failed, 3 skipped. Unfinished: still in `test_surface_e2e.py::test_tube_outer_cylinder` (~91 %) at cutoff. |
| S6A | `wp/S6A` | `e581a9e` | Mojibake fixed, `Toolpath.sample` defaults; 13 passed. |
| S6B | `wp/S6B` | `e3d06c5` | Purge-actions dropdown in Position; 10 passed, 2 skipped. |
| S6C | `wp/S6C` | `d7d7d18` | Gantry carriage in the collision env (finding 7a), moves with E1, tool never allowed; 26 passed, 3 skipped. |
| S6E | `wp/S6E` | `30f808d` | `widgets.Segmented` fallback, from `wp/F3`; 3 passed. |
| S6F | `wp/S6F` | `6305733` | Crisp step-bar labels; 3 passed. |
| S6R | read-only | n/a | Review of `a987069..3120475` (src), no tests: `run6\REVIEW.md`. Top findings are to-do 18-23. |

## State of session 5 (2026-10-28)
- **`main` @ `4087bf2`.** Sessions 1–4, plus these session 5 merges (`git log --first-parent main`):
  | merge | branch | what |
  |---|---|---|
  | `0fac8ba` | OP1 | faster cell asset load, lazy trimesh |
  | `46bbb39` | KM | K2 PTP joints hook in `krl.py`, K3 flip check + stale-root fix, K4 split/preflight/status; no K1 |
  | `bc4f554` | OP2 | Layers planning 4.5x faster, lazy trimesh/cKDTree imports |
  | `977cf1f` | OP3 | batched `Toolpath.transformed`/`rolled` |
  | `86461dd` | OP2b | vectorised `raster_segments`, surface hot spot |
  | `2ab321a` | U24 | `ui_screenshots.py --size/--scale`, offscreen by default |
  | `ebf6cfa` | U23 | timeline legend, panels fit the sidebar, 2x2 stat tiles |
  | `eae7c30` | U22 | dark 3D background, toolbars and markers follow the theme |
  | `4087bf2` | U21 | splitter, window sizing, pinned Next, saved sidebar width |
- **Tests:** each merge ran its touched tests + `test_krl_regression.py` + `test_imports.py`, all passed. Final: 76 passed. **The full suite was not run this session.** KRL output is byte-identical (hashes hold).
- **Main checkout is clean.** An agent had written F3's `style.py`/`widgets.py` changes into it by mistake (the INT tests ran with them in place). They matched `wp/F3` line for line, so they were discarded; they live on `wp/F3`.
- **Cleanup done** (`C:\Users\qrb21117\worktrees\run5\CLEANUP.md`):
  - 77 worktrees and 79 merged branches removed;
  - `assets\` checked intact (13 files);
  - left: worktree `cs-U20` (untracked `shots_before/`) and about 78 stale `.git\worktrees\cs-*` folders that OneDrive locked; pause OneDrive and run `git worktree prune`.
- **Nothing is pushed.**

## Kept branches
| branch | sha | what | to do |
|---|---|---|---|
| `wp/K1` | `4599a60` | Arcs (CIRC) and merged LINs, O5 finished: sagitta arc fit, 2 m radius cap, CIRC E1 error, LINs on a turning uncoupled table. Tube Surface 40,893 → 1,629 moves. **Held.** | Rebase on `main`; CIRC in the other move walkers; review the hash diff; full suite; T1 check of CIRC `#PATH` and `C_DIS`. |
| `wp/KM-K1` | `469d15c` | The K1 variant of KM: joints hook + `options=` on K1's `krl.py`. | Use as the reference when K1 is merged, or drop. |
| `wp/F3` | `7e8a80f` | `widgets.fit_combo`, `TileGrid`, style padding; U20 test fixed. | Merge first. |
| `wp/F1` | `a90bfd5` | WIP: Part/Position fit the 420 px sidebar (Part 346, Position 376 px). Purge-actions dropdown not done. | Merge second; switch its local `fit_combo` to F3's. |
| `wp/F2fit` | `e990480` | Toolpath Mode as a combo, StatTiles 2 columns, Playback in 2 rows, Export split `.src` + preflight checkbox. | Merge third; expect a conflict in `steps/export.py`. |
| `wp/FM` | `eda5589` | Sidebar-fit audit and merge plan (`FM-1.md`), base = U20/U21/U23. | Follow its steps; re-audit every panel ≤ 400 px; add `fit_combo(self.nozzle)` in `app.py` (~350). |
| `wp/U20` | `437fae5` | Opaque field and sidebar fills, ink text. Not merged: its tests never ran (pytest hung under load). | Merge with or after F3, then run its tests. |
| `wp/K2`, `wp/OP2b` | — | Merged; only a later report commit is left. | Delete with `-D` once read. |

## Results of session 5
- **Speed** (warm, all identical to the `tools/bench_plan.py` baseline):
  - tube Layers 85.4 → 14.8 s;
  - tube Surface 11.7 → 1.7 s;
  - block Layers 1.2 → 0.11 s.
- **KRL safety** (review findings in `C:\Users\qrb21117\worktrees\run5\BRIEF.md`, if kept):
  - 1: PTP joints come from `post/joints.py` `program_joints`, the same IK as Simulate (`robot.ik` only as a fallback);
  - 2: "flip" check row: a joint jump > `JUMP_DEG` inside one LIN/CIRC;
  - 4: the collision check follows the configured gantry when axes has no E1 (no stale root);
  - 5: E1 changes go only on gun-off moves;
  - 6: the plan key includes the robot side.
- **K4 modules** (all off by default): `post/krl_split.py` (subprograms), `post/krl_preflight.py` (TOOL/BASE guard), `post/krl_status.py` (S/T; S bits 0 and 1 UNVERIFIED).
- **UI:** sidebar/3D splitter; sidebar 460 px (min 400), width saved; window min 1100x680, start 1440x900 capped to the screen; Next pinned below each panel; Mode shown as Layers / Revolve / Surface; panels shrink to the sidebar; dark mode for the 3D view, toolbars and markers.

## To do, in order
1. **Merge the fit branches:** done in `wp/S6M` (`76bf656`); fix its 2 failing tests (capsule, order-dependent fit), then merge to main.
2. **Full suite:** S6T ran 598 passed on `main`, unfinished at `test_tube_outer_cylinder`; rerun after the merges.
3. **By-hand check in the app (`.\run`):** resize, splitter, light and dark, 100 % and 150 %; Surface patterns on the tube end; KRL base; Fixed in the cell; gun hold; constraint picking; Suggest purge point; Cancel and the plan cache; WearPlate2.
4. **K1 arcs:** see Kept branches.
5. **Review finding 3:** PTP moves are interpolated in Cartesian space in `Toolpath.sample`, and HOME → first and last → HOME are not simulated. Sample PTPs in joint space and check those legs.
6. **Review finding 7a:** done in `wp/S6C` (`d7d7d18`); review and merge.
7. **Review finding 8:** `Toolpath.sample` uses the last move's axis value where KRL uses the configured value.
8. **Faint step-bar done labels:** done in `wp/S6F` (`6305733`); check on screen, merge.
9. **"90Â°" mojibake:** done in `wp/S6A` (`e581a9e`); merge.
10. **`widgets.Segmented`:** done in `wp/S6E` (`30f808d`, on F3); merge.
11. **Slicer `polygonize` option:** faster `slice_mesh`, but it changes the output. Decide, then re-baseline.
12. **Launch time:** re-measure on a quiet machine (78 s cold under load); the `pyvistaqt` import is the biggest part.
13. **Purge-actions dropdown:** done in `wp/S6B` (`e3d06c5`); merge.
14. **Export split/preflight end to end:** save a long program, load all parts on the controller, check the HALT guard.
15. **S/T bits:** verify against `$POS_ACT` before writing S/T on LINs.
16. **Older list:** cable wind-up check (A4/A6); Surface per-face split and holes; saved constraints; solver gaps; P7 collisions with the Filtration unit; reach IK on a worker.
17. **Robot cell checks:** README "Before running on the real robot", in T1 first.
18. **S6R 1:** E1 is written on the layer's first gun-off move, so moves before it use the LAST E1 in `Toolpath.sample` (back-fill) but the cell E1 in the KRL (`solve_e1.py:27`, `krl.py:82`). Simulate and the program disagree. Also `fix_e1` scores pre-carrier gun-on moves it cannot fix, and a layer with `carriers[k] is None` leaks E1 into the next.
19. **S6R 2:** `krl.py:157` PTP uses Simulate's configuration, but the joint-space sweep (e.g. the E1-change PTP) is never collision-checked.
20. **S6R 3:** `krl.py:160` NaN fallback seeds from the last PTP, not the last LIN, and silently picks `sols[0]`. Seed from the last valid `pj`; log or raise.
21. **S6R 4:** `collision.py:231` `check()` follows the live `cell.gantry.position_mm`; a moved E1 raises an uncaught `ValueError` in `solver.py:97,130`, `solve_arm.py:76`, `viewer.py:918`. Also an `np.allclose` per check in the hot loop.
22. **S6R 5:** `plan_jobs.py:101` side-free is judged on the final path, not at `level_gun` time, so a plan can be reused for the wrong robot side; `PlanKey.cell` is live (racy).
23. **S6R 6-8:** sidebar min width is fixed at build, so Advanced mode can clip with no scroll bar; Next footer lookup is fragile; OP3 `zigzag_regions` now carried (output change; batched gemm not verified bit-identical).

## Waiting on the user
1. **Obsolete unmerged branches:** `wp/WP0-wip`, `wp/M2-fix`, `wp/E2p` (Part Simple/Advanced, test hangs), `wp/P8` (slow turn clearance), `wp/O5` (superseded by `wp/K1`). Delete them?
2. **Spray between layers:** it still switches off, with a settle wait each layer. Keep it on when jump height is 0?
3. **Crosshatch:** with the default 90° step it repeats a direction (0, 90, 90, 0). Change it?
4. **Position Simple mode:** compare E1 with `gantry.position_mm` for "changed"?
5. **Purge:** spray on from the first purge visit to the last? `spray_dir` is still CHECK.
   - **Suggest purge point along ±Y** (user: spray along Y towards the filtration unit at the back of the cell).
     `wp/S6P` `b2d072c` makes the spray exactly (0, ±1, 0); 11 tests pass. **Not merged.**
   - The problem: the whole "Filtration unit" mesh is huge (X −1308..1301, Y 437..1540, Z to 2830), and the robot root at Y 1069 sits inside its Y span. S6P picks −Y and puts the point inside the unit's box.
   - **User decision (2026-10-28): the back of the cell is +Y.** `wp/S6P2`, branched from `wp/S6P`, was started on the end-of-session-6 deadline. It fixes the spray to (0, +1, 0) as a config setting, aims at the unit's mesh rather than its bbox, and restores the strict test. Check whether it committed (`git log wp/S6P2`; report in `plans/reports/S6P-1.md`), then finish and merge it.
6. **Wrapping Surface regions:** split them per CAD face?

## Lessons for running agents
- Agent count is not the limit (user, 2026-10-28): the crashes came from on-screen app windows. Run app, screenshot and GUI-test runs offscreen; run as many agents as the work needs.
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
