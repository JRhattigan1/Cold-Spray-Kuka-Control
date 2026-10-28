# Next session

**Start here.** Where each feature stands is in [ROADMAP.md](ROADMAP.md), and what the app does is in [README.md](../README.md). Each agent's detail is in `plans/reports/<ID>-1.md` on its branch (`git show wp/<ID>:plans/reports/<ID>-1.md`; the session 5 worktrees are removed). Older hand-offs are in git history.

## State (2026-10-28, end of session 5)
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
- **The main checkout has uncommitted edits** to `gui/style.py` and `gui/widgets.py` (button padding 14 → 10, `fit_combo`, `TileGrid`), made by an agent that wrote there by mistake. They block the U20 merge, and the INT tests ran with them in place. Compare them with `wp/F3`, then commit or discard them.
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
| `wp/U20` | `437fae5` | Opaque field and sidebar fills, ink text. Not merged (blocked by the main-checkout edits). | Merge with or after F3. |
| `wp/D0` | — | These docs. | Merge. |

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
1. **Merge the fit branches:** sort out the main-checkout edits, then F3 → F1 → F2fit (FM-1 steps), then U20.
2. **Full suite** (not run this session).
3. **By-hand check in the app (`.\run`):** resize, splitter, light and dark, 100 % and 150 %; Surface patterns on the tube end; KRL base; Fixed in the cell; gun hold; constraint picking; Suggest purge point; Cancel and the plan cache; WearPlate2.
4. **K1 arcs:** see Kept branches.
5. **Review finding 3:** PTP moves are interpolated in Cartesian space in `Toolpath.sample`, and HOME → first and last → HOME are not simulated. Sample PTPs in joint space and check those legs.
6. **Review finding 7a:** the gantry carriage is not in the collision environment.
7. **Review finding 8:** `Toolpath.sample` uses the last move's axis value where KRL uses the configured value.
8. **Faint step-bar done labels** (maybe nested QGraphicsEffects; check on screen).
9. **"90Â°" mojibake** in the 3D HUD labels (`sim/viewer.py` ~594, ~625): write `°`.
10. **`widgets.Segmented`** (combo shown when the capsule row doesn't fit): planned by FM, not written.
11. **Slicer `polygonize` option:** faster `slice_mesh`, but it changes the output. Decide, then re-baseline.
12. **Launch time:** re-measure on a quiet machine (78 s cold under load); the `pyvistaqt` import is the biggest part.
13. **Purge-actions dropdown** in Position (F1 left the buttons stacked).
14. **Export split/preflight end to end:** save a long program, load all parts on the controller, check the HALT guard.
15. **S/T bits:** verify against `$POS_ACT` before writing S/T on LINs.
16. **Older list:** cable wind-up check (A4/A6); Surface per-face split and holes; saved constraints; solver gaps; P7 collisions with the Filtration unit; reach IK on a worker.
17. **Robot cell checks:** README "Before running on the real robot", in T1 first.

## Waiting on the user
1. **Obsolete unmerged branches:** `wp/WP0-wip`, `wp/M2-fix`, `wp/E2p` (Part Simple/Advanced, test hangs), `wp/P8` (slow turn clearance), `wp/O5` (superseded by `wp/K1`). Delete them?
2. **Spray between layers:** it still switches off, with a settle wait each layer. Keep it on when jump height is 0?
3. **Crosshatch:** with the default 90° step it repeats a direction (0, 90, 90, 0). Change it?
4. **Position Simple mode:** compare E1 with `gantry.position_mm` for "changed"?
5. **Purge:** spray on from the first purge visit to the last? `spray_dir` is still CHECK.
6. **Wrapping Surface regions:** split them per CAD face?

## Lessons for running agents
- At most about 6 agents at once. With more, pytest runs were killed and timings were noise.
- App and screenshot runs are offscreen and one at a time.
- Agents' clocks drift: use `date`, not their own sense of time.
- A manager per team (KM, FM) worked: one plan, disjoint files, one merger.
- A hook that crosses branches (e.g. K2's `krl.py` hook on K1's file) needs one named owner.
- No agent may edit the main checkout. One did, and it blocked a merge.

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
