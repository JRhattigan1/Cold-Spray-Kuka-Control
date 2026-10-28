# KM-1: KRL integration plan (K1..K4). Status: sprint 1 design done, nothing merged yet

## Base
wp/O5 (9077170, WIP) adds CIRC + merged LINs: toolpath/simplify.py, Move.aux (via pose), model.py helpers
(circle, circ_poses, slerp_mats, move_length), Toolpath.sample follows arcs, krl.py writes
`CIRC aux, end [C_DIS]` with `$CIRC_TYPE=#PATH`, `$ORI_TYPE=#VAR`, arc-length speeds. 3 tests failing
(regression hashes x2, test_surface infill-gap assertion). K1 starts from wp/O5.

## Ownership (to avoid conflicts)
| Agent | Owns | Must not touch |
|---|---|---|
| K1 | post/krl.py, toolpath/simplify.py, toolpath/model.py, tests/test_krl*.py, tests/test_simplify.py, tests/test_surface.py | joints.py, checks.py |
| K2 | post/joints.py (new), planner/solve_e1.py, gui/plan_jobs.py (plan_key), tests/test_joints.py, test_solve_e1.py | krl.py (asks K1/KM) |
| K3 | gui/checks.py, sim/collision.py, tests/test_checks.py, test_collision*.py | krl.py, joints.py |
| K4 | post/<own module, e.g. krl_options.py> + its test | krl.py (asks K1/KM) |
`tests/test_krl_regression.py` hashes: only K1 edits them; KM re-hashes after each merge that changes output.

## Interfaces
- **K2 joints hook.** `post/joints.py: ptp_joints(path, robot, cell, home) -> dict[int, np.ndarray]`
  (move index -> 6 joints, deg) computed exactly as Simulate does: `path.sample(...)`, roots per E1
  (robroot), `robot.ik_path(targets, seed=home, hint=path.hint_for(s.move_index))`, take the sample at
  each PTP move's end. Raise/return NaN when unreachable. Ideally sim/viewer.py calls the same helper later
  (ask KM; not this run). krl.py change (K1, one line): `to_krl(..., joints=None)`; in the PTP branch
  use `joints[i]` when given, else `ptp_joints(...)`, keep robot.ik only as fallback with a comment.
  K2 also supplies `ptp_joint_path(q0, q1, n)` (linear joint interpolation) so HOME->first, last->HOME
  and PTP transits can be simulated/checked (finding 3) - consumed by K3.
- **K1 arcs.** CIRC must survive the PTP/E1 branch: a CIRC whose E1 differs from the previous move is an
  error (KRLError), not silently a PTP. CIRC only when the table does not turn during the move or the
  table is coupled (aux is computed at the END table position; a turning uncoupled table makes the arc
  wrong) - otherwise simplify keeps LINs. Solvers that change poses after simplify must call
  `toolpath.model.decirc(path)` (K1 provides) or re-run simplify; K1 documents which solvers do.
- **K3 flip check.** `checks.py` uses the same samples as Simulate (incl. arc samples) and flags joint
  jumps > solve_arm.JUMP_DEG inside any LIN/CIRC (finding 2). Collision: `CollisionChecker.check` resets
  the root to the placement when axes has no E1 (finding 4). K3 can import K2's `ptp_joint_path` after
  K2 merges; until then stub behind try/except ImportError.
- **K4 module.** Pure functions over (path, cell, proc) or over the KRL line list, all OFF by default so
  the regression hashes do not change; wired into krl.py by KM at merge via an `options=` argument.
  Candidates: soft-limit check of PTP joints, velocity/accel sanity, program splitting (size), header
  comments (sha, part, date), $ADVANCE, spray-state guard. K4 reports the list; KM picks.

## Integration order (into wp/KM)
1. K1 (arcs finished, tests green, hashes re-baselined with a reviewed diff: only LIN->CIRC/merge lines).
2. K2 (joints hook + solve_e1 + plan_key). Conflict expected only in krl.py PTP branch if K1 did not add
   `joints=`: KM adds it. Hashes change (PTP joint values) -> KM re-hashes, diff must touch PTP lines only.
3. K3 (checks/collision; no KRL output change; hashes must stay).
4. K4 (module + KM wiring; default off, hashes must stay).
Full suite after each merge; benchmark test_tube / test_block / test_repair (move counts, KRL diff).

## Decisions per K agent
- **K1:** finish O5 on wp/K1 from wp/O5: fix 3 tests, simplify tests (arc fit, 0.05 mm tolerance, CIRC
  syntax), add `joints=` param, KRLError for CIRC with E1 change, no CIRC on turning uncoupled table,
  lower arc radius cap (10 m is near-collinear: KUKA "circle not definable"; use <= 2 m or LIN), `decirc`.
- **K2:** `post/joints.py` as above (Simulate-identical), solve_e1 puts E1 changes only on gun-off moves
  (finding 5), plan_key keeps gantry.position_mm when `wrist` is set (level_gun uses robroot, finding 6).
- **K3:** flip check in checks.py for LIN and CIRC, collision root reset (finding 4); optional: PTP transit
  check with K2's joint path.
- **K4:** survey -> short list in report; implement 1-2 in its own module, default off, no krl.py edits.

## Real-robot safety risks (to verify in T1 before auto)
1. PTP configuration differs from the one simulated (finding 1) -> collision. Fixed by K2 hook.
2. Configuration flip inside LIN/CIRC -> robot stops or swings through a singularity (K3 flags).
3. Gun on during a PTP (E1 change on spray move, finding 5) -> uncontrolled spray path. K2 + K1 guard:
   krl.py should refuse a PTP while gun TRUE.
4. CIRC #PATH orientation model (circ_poses) unverified; C_DIS between CIRCs; near-180 deg arcs and huge
   radii -> controller errors or a different arc than simulated. T1 test on a fillet first.
5. Uncoupled turntable + CIRC -> arc in wrong frame.
6. Solvers moving poses without updating aux -> via point off the path (wild arc).
7. Stale E1 root in collision (finding 4) -> false "clear". HOME->first / last->HOME unsimulated (finding 3).
8. Gantry carriage not in the collision env; per-(link, object) allowed contacts can mask a whole-cell mesh (finding 7).
9. Toolpath.sample's axis fallback differs from KRL (finding 8) -> sim and program disagree on axis values.

## Not done
Nothing merged. Next: wait for K1 sprint 1 sha, merge K1 into wp/KM, run full suite, review KRL diff.

Test line: none (design sprint).

## Sprint 2-3 final state (08:41) - wp/KM = main + K4 + K2 + K3 + KM hook. K1 HELD (D1 decision b)
- Merged: wp/K4 18309a8 (krl_split, krl_preflight, krl_status), wp/K2 e6ba4cf (post/joints.py, solve_e1 carriers,
  PlanKey), wp/K3 48c6402 (flip row, collision root fix, allowed_text).
- KM on main's krl.py: `pj = program_joints(path, robot, cell)`; a PTP uses `pj[i]` when not NaN, else the old
  `robot.ik` seeded from the last PTP, KRLError if that is unreachable too (D1 a). `options: KRLOptions | None`
  (preflight=False default; `preflight_lines` after `BAS(#BASE, n)`). viewer.py: `self.move_kind` line (K3 ask; INT
  resolves with U22).
- **KRL diff:** test_block / test_repair / test_tube programs are byte-identical before/after the hook (the seeded ik
  already chose Simulate's branch on these parts), so the regression hashes are unchanged; no re-baseline.
- New tests/test_krl_joints.py: KRL PTP joints == program_joints rows (test_block); preflight off by default and
  placed right after BAS(#BASE).
- K1 variant kept on **wp/KM-K1** (wp/KM-before-reset + K1 e24f730 + hook on K1's `joints=` dict). Not for main
  until K1's hashes are re-baselined against a reviewed diff and its suite is green; #PATH/C_DIS needs T1.
  Test (c) (K2's E1 carriers never trigger K1's CIRC-with-E1 KRLError) is pending, since K1 is held.
- krl_split note: `_MOTION` matches PTP|LIN only, so with K1 a CIRC is never a cut point (safe, conservative);
  extend it to CIRC when K1 lands.
- Ask (GUI export owner, gui/steps/export.py): write `split_krl(text)` items as separate .src files when >1, and
  expose a "preflight" checkbox -> `to_krl(..., options=KRLOptions(preflight=True))`. I did not edit export.py.
- Not done: T1 verification of krl_status S bits 0/1 (unverified); the full suite (INT runs it).

Test line: test_krl_joints, test_krl, test_krl_regression, test_checks, test_collision*, test_imports, test_krl_split,
test_krl_preflight_status: 113 passed, 3 xfailed (76 s).
