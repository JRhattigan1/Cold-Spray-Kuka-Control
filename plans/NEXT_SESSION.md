# Next session

**Start here.** Where each feature stands is in [ROADMAP.md](ROADMAP.md), and what the app does is in [README.md](../README.md). Each agent's detail is in `plans/reports/<ID>-1.md` on main. Older hand-offs (session 7 state, run 8 solver speed-up) and the finished feature and optimisation plans are in git history.

## State (2026-10-29)
- `main` @ `641faad` (session 11 merged); not pushed (origin/main is at session 9).

## Session 13 (2026-10-29, no agents): startup, rotor part, Revolve regions + helix
- Startup 24 s -> 16 s: OCP/trimesh imported on a thread (`gui/__main__.py`); cell meshes carry numpy point normals so PyVista skips `compute_normals` (`sim/cell_model.py`).
- `test_rotor` built-in (500 long, 60 dia journals, drum + 4 discs); picks Revolve.
- Revolve honours picked coat faces: `revolve_bands` -> one region per axial band, gun off between.
- `turntable.rotation.endless` (default false): Revolve plans one helix per band. Open: real axis mode on the controller and how KRL must write an endless E5 (now a growing angle, e.g. 28800°). No dedicated tests yet for bands / helix.
- `endless: true` is committed in `cell.yaml` for testing the helix: test_tube Revolve tests (incl. KRL regression) expect rings, so set it back to false before `run test all`.
- Cell view: double-click empty background = zoom to fit the cell (`Scene._double_click_fit`); not on the part preview panel.
- Note: `process.yaml` is rewritten by the GUI's "Save as default"; values other than the committed ones (e.g. `jump_height_mm: 0`) fail the KRL regression and layer tests.

## Session 12 (2026-10-29): Surface as one blended raster (in progress)
- User: a multi-face Surface pick must be coated as one smoothly blended raster, with a **consistent step-over on the surface** (reverses the S7S per-face split).
- Round 1 `wp/S12U` (not merged): passes = geodesic-distance level sets; `surface_split_faces` option (default off); `test_wearplate` built-in. Reviewer `wp/S12UR`: FIX FIRST: repair top+4 sides 18.9 % of gaps outside ±10 % (max 2.99 mm at 3-face corners), wearplate 9.5 %; gun jumps 90° / 56°; planning 35 s (box) vs 1.3 s on main, tube 207 s / 193k moves. The builder loosened its own spacing test (rejected). Independent checker: `wp/S12UR` `tests/test_S12UR_accept.py`.
- Round 2 `wp/S12V` (from S12U, not merged) + reviewer `wp/S12VR`: FIX FIRST. Only cap + square walls handled (`wall_split`, `_ring_strokes`): top rastered alone, sides as rings, gun off between. Repair: 4.5 % of gaps outside ±10 % (min 0.94 mm at the cap/wall seam, max 2.42), axis step 14.7° infill; 26.7 s, 5652 moves. Wearplate unchanged (9.9 %, max 3.02, 55.8° turns). Tube not measured (>180 s). Not done: one-shot geodesic, smoothing, 3-face corner seams in general. `test_surface_unfold.py:89` is still round 1's loosened "99 % within ±10 %": tighten to every gap. Golden/KRL/imports not run on S12V.
- **Decision needed (user):** at sharp edges (box top→sides) one pass can't stay at standoff and turn smoothly. (a) blend everywhere (quick smooth turn, spacing near the edge suffers) or (b) blend fillets/gentle edges, split only at sharp edges with passes lined up so step-over carries across (first pass step/2 from the edge, gun off only there). Recommended (b).

## Session 11 (2026-10-29): 6 builders + 6 reviewers, 18 min, all merged
| merge | what |
|---|---|
| S11B | KRL base picker: Face + edge (default, KRL unchanged) / 3 planes / Vertex (hover preview, KD-tree snap); spin about Z, ±90°, Flip Z; faceless bases save/open (job stores the final turned base) |
| S11T | **Cell** button → dialog: Tool tab (TOOL_DATA per nozzle, pendant paste, CAD TCP + difference), Base tab (BASE_DATA stored only). At merge: KRL now selects the active nozzle's tool number (`KRLOptions.tool_number`) |
| S11H | HOME tab in the Cell dialog: jog A1–A6 + E1, live robot, collisions/limits, Save `home_deg` to cell.yaml; robot goes back on leaving the tab |
| S11P | Purge settings in Position undo; a drag is one undo step (dragging already existed) |
| S11S | Re-route lever checks only the changed span: block_layers reroute 29 → 10 s, output identical |
| S11G | Cell meshes split into 600 mm FCL chunks: gantry lever 53 → 41 s (noisy; re-time), output identical |
Reviewer reports: `wp/<ID>R` branches (`plans/reports/<ID>R-1.md`, acceptance tests not merged).

**Open from session 11 (reviewers):**
- Radial nozzle `tool_number: 2` in tool.yaml is a guess: CHECK the pendant. 3D view's TCP only refreshes after restart. Taught bases not shown in the KRL header.
- `save_tools` drops a trailing comment on a `tool_number:` line; `parse_pendant` reads `X abc` as 0 (should reject); a typo in a Cell table cell raises an uncaught ValueError.
- HOME: Save allowed with joints out of limits or colliding (warning only); E1 edited but not saved; multi-line `home_deg` not matched.
- Gantry lever: likely dominated by building a checker per candidate E1: profile (`bench_solve --profile`), cache checkers/cropped meshes per E1.
- Reroute: add a test comparing span vs full-path results at E1 changes / path end.
- Session 11 branches/worktrees (`wp/S11*`, `cs-S11*`) to clean up with the rest.

## Session 10 (2026-10-28): Build-up toggle (committed `e2ec9ae`)
- Simulate **Build-up** checkbox (under Thickness). Layers: finished layers solid (copper, part cut at the layer being sprayed), that layer's contour/infill tracks appear as the gun passes (step-over wide, one layer high, two alternating shades); path lines hidden, target part faded. Other modes: thickness map follows playback (final scale). Thickness map: uncoated now grey.
- Files: `sim/deposit.py` (`thickness(keep_steps=)`, `Deposit.thickness_at`, `spray_samples(times=)`), `sim/viewer.py` (`set_build_up`, `_layer_tracks`, `set_thickness(times=)`; `BUILT`/`TRACKS`/`UNCOATED`), `gui/steps/simulate.py`, `tests/test_deposit.py` (2 new tests + GUI toggle), HAND_CHECK.
- Limits: strips are flat; turns not drawn as strips; overrun lost once a layer is finished (solid = part shape); deposit steps kept in memory (grows with path length); thickness/build-up still on the GUI thread.

## Session 9 (2026-10-28): all merged
User asked: fix known bugs, add day-to-day features, housekeeping (robot T1 not possible yet).
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

**Cleanup left for the user (blocked by the permission check):**
- Worktrees still present: `C:/Users/qrb21117/worktrees/cs-S8B`, `cs-S9F`, `cs-S9H`, `cs-S9J`, `cs-S9S`, `cs-S9T`, `cs-S9U`, `cs-U20`. For each: `cmd /c rmdir <wt>\assets` (if the junction exists), then `git worktree remove <wt>` (`--force` only for S8B and U20, which hold throwaway untracked files).
- Then `git branch -d wp/S8B wp/S9F wp/S9H wp/S9J wp/S9S wp/S9T wp/S9U wp/U20` and `git branch -D wp/KM-K1` (obsolete).
- Pause OneDrive and `git worktree prune`. Remote `origin/surface-patterns` still exists (merged).

## To do, in order
1. **By-hand check:** [HAND_CHECK.md](HAND_CHECK.md).
2. **Decide (user):** S9F findings above (HOME at E1 2730, E1 5500 wall, purge example, elbow-up IK weighting).
3. **Arcs in T1** (test_block, arcs on): orientation along CIRC with `#PATH`, C_DIS blending, `$VEL.CP`, `$CIRC_TYPE` inside split `_Pnn`. CIRC on a turning table is a 2-LIN chord (short arcs only).
4. **Calibrate:** thickness model and `powder_feed_g_min` against a sprayed coupon; `cable_windup_deg` from the cable dressing.
5. **Collision-aware auto-position** (repair Surface hits); gantry lever sampling vs the 2 mm check; gantry lever speed on WearPlate2.
6. Thickness map on a worker thread; HOME legs drawn.
7. **Position Simple mode:** compare E1 with `gantry.position_mm` for "changed" (user: maybe).
8. Solver speed: reroute's remaining time is `find_issues` over the whole path after each accepted candidate; gantry: profile and cache per-E1 checkers (see session 11). Benchmark with `tools/bench_solve.py`.
9. Slicer `polygonize` option; launch time; S/T bits vs `$POS_ACT`; holes in Surface; reach IK on a worker; `solve_arm` uses Cartesian samples; README robot checks in T1.

## User decisions (2026-10-28)
- Spray stays on between layers at jump 0; crosshatch removed; purge spray on first→last visit (spray is manual for now).
- Gun angle limit 70° as a warning. Back of the cell (filtration) is +Y.
- Layers: the part is not a collision body (not built yet); Revolve/Surface: it is.
- Purge → start avoids the part. Solver suggests gantry E1 and moves purge XYZ, not its angle.
- Overrun is measured from the body's faces, not the toolpath.
- ~~Wrapping Surface regions split per CAD face.~~ **Reversed 2026-10-29:** the user wants a multi-face Surface pick coated as one smoothly blended raster (unfold and raster across), with a consistent step-over measured on the surface for even deposition; per-face split becomes an option (S12U).
- **Fewer agents** (user, session 7): do work inline; use an agent only for a big separate piece.

## Lessons for running agents
- App, screenshot and GUI-test runs stay offscreen and one at a time (the earlier crashes came from on-screen app windows).
- Agents' clocks drift: use `date`, not their own sense of time.
- A manager per team (KM, FM) worked: one plan, disjoint files, one merger.
- A hook that crosses branches (e.g. K2's `krl.py` hook on K1's file) needs one named owner.
- No agent may edit the main checkout. One did, and it blocked a merge. Check `git status` on main before merging.
- **Cleanup:** each worktree's `assets` is a junction to the real `assets\` (licensed CAD, not in git). Remove the junction with `cmd /c rmdir <wt>\assets` before `git worktree remove`. Never `Remove-Item -Recurse` or `rm -rf` a worktree folder.
- Per-run files (brief, check-in log, INT and CLEANUP reports, screenshots) live in `C:\Users\qrb21117\worktrees\run5\`.

## Agent protocol (paste into every agent prompt)
```
Repo C:\Users\qrb21117\OneDrive - University of Strathclyde\Documents\Cold-Spray-Kuka-Control. Timebox 10-20 min; stop coding 3 min before.
git worktree add C:\Users\qrb21117\worktrees\cs-<ID> -b wp/<ID> main
Assets junction from PowerShell (Git Bash mklink mangles the path): cmd /c mklink /J C:\Users\qrb21117\worktrees\cs-<ID>\assets "<repo>\assets"
Test: $env:PYTHONPATH="C:\Users\qrb21117\worktrees\cs-<ID>\src"; C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pytest -q -p no:cacheprovider --area <area>   (or <files>; areas in tests/conftest.py, ~1 min each, guard set included). NO -n, no full suite: the manager runs `run test all` once at merge. Two agents' -n 6 runs crashed the PC (video memory bugcheck 0x10E, 16 GB RAM)
GUI tests offscreen; stub MainWindow._refresh_steps (cell assets never load off screen); plan_jobs.wait(win) after generate/_auto_position; cap long runs.
Benchmark on small built-in parts (test_tube, test_block, test_repair), not WearPlate2 (too slow).
Use absolute worktree paths for every edit (an agent once wrote into the main checkout by mistake).
Test first, minimal diff, edit only owned files. Commit on wp/<ID>, trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". No push, no main.
Report plans/reports/<ID>-1.md: status, done, not done, test line. Return: sha + test line + 1 line.
```
**Models:**
- The `worker` agent type (`.claude/agents/worker.md`, Sonnet): contained single-module tasks; otherwise `general-purpose` with `model: sonnet`.
- Use `model: opus` for work across threading, the interface and result flow, or tricky maths (IK, arcs).

**Merging:**
- `--no-ff`;
- review the diff;
- run the touched tests plus `tests/test_krl_regression.py` and `tests/test_imports.py`;
- run the full suite afterwards.

**Stuck agent:** stop it, check its worktree, and commit its work in progress with a partial report, as was done for O5.
