# Session 3 run-sheet

Context: `plans/reports/session-2-summary.md`. Run each task as one agent, all in parallel, except where noted.

## Protocol (prepend to every agent prompt)
```
Repo C:\Users\qrb21117\OneDrive - University of Strathclyde\Documents\Cold-Spray-Kuka-Control. 10 min; stop coding at 7.
git worktree add C:\Users\qrb21117\worktrees\cs-<ID> -b wp/<ID> main
cmd /c mklink /J C:\Users\qrb21117\worktrees\cs-<ID>\assets "<repo>\assets"
Test: $env:PYTHONPATH="C:\Users\qrb21117\worktrees\cs-<ID>\src"; C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pytest -q -p no:cacheprovider <files>
Test first, minimal diff, edit only owned files. Commit on wp/<ID>, trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". No push, no main.
Report plans/reports/<ID>-1.md: status, done, not done, test line. Return: sha + test line + 1 line.
```

## Step 0 (coordinator, before agents)
1. ~~Run the full suite on `main`~~ Done: 446 passed after the 13 merges.
2. Merge `wp/G1` (ready, 90 tests passing).
3. `wp/R3` has 4 uncommitted files from a stopped agent. Review that diff in its worktree, keep or discard it, then give it to R3b.

## Tasks
| ID | Owns | Task |
|---|---|---|
| R3b | gui/steps/part.py, gui/steps/toolpath.py `_mode_changed`, tests/test_workflow.py, pyproject.toml | Branch from wp/R3 (merge main first). Apply the coat-face fix (R3-1.md #1, #2), remove the xfail, add a registered `slow` marker |
| P1 | cad/constraints.py, tests/test_constraints.py | Angle residual smooth at 0°/180° (right DOF count); optional distance side; make `test_fast` robust |
| P2 | planner/purge.py, planner/position.py, their tests | Every-N purge only at real layer ends (not Surface `approach`); auto_position names an unreachable purge point |
| P3 | planner/patterns.py, tests/test_patterns.py | Check crosshatch ≠ zigzag and spiral ≠ concentric by path, not count; fix if identical; update e2e move-count snapshots if they change |
| P4 | gui/constraints_panel.py, its test | Add `centre_between` and `fix` rows |
| P5 | gui/steps/simulate.py, gui/steps/export.py, new test | Simple/Advanced: register non-essential controls via `widgets.advanced` (register containers; see E1-1.md caveat) |
| P6 | gui/steps/position.py, tests/test_purge_ui.py | Remove F2 workarounds (`model_fields` guard, `hasattr`, `PURGE_DEFAULTS`); Simple = X/Y/A, Keep on plate, Auto position, purge Enabled |
| P7 | new tests/test_collision_e2e.py | Collision checks: pattern + purge on test_block, Surface on test_repair. Mark real hits as strict xfail; no product edits |
| P8 | planner/surface.py, tests/test_surface.py | Raise each turn until every sample is at least `standoff_mm` from the surface |
| P9 | gui/steps/toolpath.py process form | `turn_plane` combo (datum:XY/XZ/YZ + offset). **Run after R3b merges** (same file) |
| E2p | gui/steps/part.py | Simple/Advanced for the Part step. **Run after R3b merges** (same file) |

## Merge (coordinator, or one merge agent)
Order: R3b, P1, P2, P3, P8, P4, P6, P5, P7, then P9 and E2p. Use `--no-ff`. On conflict keep both sides, then run the touched tests plus `test_krl_regression.py`. Afterwards run the full suite.

## Then
- One docs agent: update plan §5 and §8, `plans/ROADMAP.md` and the README from the reports.
- Human check in the app (`.\run`), then the robot-cell checks in the README.
