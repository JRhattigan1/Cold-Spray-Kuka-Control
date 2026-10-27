# Session 1 hand-off (2026-10-27)

All `wp/*` branches are merged into local `main`; nothing is pushed. The tracker is §5 of `plans/2026-10-27-feature-plan.md`. Each WP's report is `plans/reports/<WP>-<n>.md`, and its "Not done" checklist plus "Exact next step" serve as the resume brief.

## Merge notes (resolved by the coordinator)
- **pipeline.py (F1 + D3):** `add_purge` is applied to all three returns (revolve, surface, layers).
- **layers.py (B1 + C1):** both imports kept (contour_tilt + patterns re-exports).
- **cad/part.py (WP0r + D1):** both `cylinders` and `coat_faces` fields kept.
- **Code still using `getattr(proc/cell, ..., default)`:** B2, C1, D3, F1, F2 and A1 were written before the config fields landed. The fields are on main now, so the `getattr` fallbacks (and B2's SimpleNamespace test fallback) can be simplified.
- **Stale OneDrive lock:** `.git/worktrees/cs-WP0` could not be deleted, which makes git print a harmless "Permission denied" on each commit. Delete it by hand with OneDrive paused. If git claims a merge is in progress when it isn't, `git merge --quit` clears a stale MERGE_HEAD.

## Next session: suggested order
1. **Human GUI check** (`.\run`):
   - test_block with each infill pattern and contour tilt 20°;
   - datum planes;
   - coat faces on test_tube, then Surface mode;
   - purge section and marker;
   - Simple/Advanced toggle.
2. **Parallel:** A3 (constraint UI), G1 (turning points off the part), the D3 end-to-end run, and E1 form wiring (`gui/steps/toolpath.py`). E1 and G1 both edit the toolpath form, so run them one after the other.
3. **Then:** the partial items in B2, F2 and A2, and finally E2 (the UI sweep and README) on its own.

## Worktrees
`C:\Users\qrb21117\worktrees\cs-*` can be removed (`git worktree remove`) now that all branches are merged.
