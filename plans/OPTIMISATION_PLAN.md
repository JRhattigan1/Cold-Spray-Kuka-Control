# Optimisation plan (2026-10-27)

## Status (end of session 4)
All of this is on `surface-patterns`, and the full suite passes.

| Step | Status |
|---|---|
| 2. Per-move overhead | **Done.** Each change was verified identical to the old code: <ul><li>gun hold 12.8 → 0.8 s (WearPlate2);</li><li>sampling 17.5 → 0.4 s (test_tube);</li><li>IK with frames 24 → 3.3 s (WearPlate2);</li><li>collision check 3.25 → 1.65 s (test_block; the rest is FCL).</li></ul> |
| 1. Arcs and merged LINs | **Partial on `wp/O5`, not merged.** Simplify, CIRC sampling and KRL `CIRC` work; test_tube Surface goes from 40,893 to 1,581 moves. The checklist is in `reports/O5-1.md`. |
| 3. Keep the window responsive | **Mostly done.** <ul><li>Generate and Auto position run on a worker thread with Cancel.</li><li>Plan cache.</li><li>Not done: reach IK in `set_path` still runs on the GUI thread, and the collision checker isn't built in the background.</li></ul> |
| 4. Smarter checking | Not started. Only needed if it's still slow. |
| 5. Purge suggestion | **Done:** a "Suggest purge point" button. |

The measurements and the plan as written follow.

These numbers were measured on WearPlate2 (`Downloads/WearPlate2.stp`) in Surface mode, with the 17 faces round the top, the angled face and the blends selected. The run had the full test suite going at the same time, so the absolute times are high. The proportions are reliable. The profiling script is `profile_run.py` in the session scratchpad; it can be rebuilt from this table.

| Stage | Time | Where it goes |
|---|---|---|
| Load STEP + tessellate | 1.6 s (cached) – 7 s | fine |
| Surface plan (38,853 moves) | 38.7 s | `level_gun` ≈ 68%: `base_world()` per move with scipy rotations (13 s), and `frame(c=…)` per move (8 s). Pose building and `np.cross` on single vectors cost the rest. |
| `path.sample` (43,267 samples) | 39.9 s | a scipy `Slerp` built per move (16 s), plus `Rotation.from_matrix` per move |
| IK for every sample | 55.3 s | `base_in_robot()` per sample (13 s), and per-point OPW IK in Python (20 s) |
| Build collision checker | 17.1 s | FCL BVH builds in `trimesh` `add_object` (15 s) |
| Collision `check_path` (14,054 checks) | 40.3 s | `_place` per check (22 s, scipy forward kinematics), FCL queries (17 s) |
| Layers plan, test_block | 1.1 s | fine |
| Layers plan, WearPlate2 (about 800 layers, 214,557 moves) | **429 s** | Auto position used to run this on entering Position, once per tilt. Steps 1 and 2 apply: per-move overhead, and merging the straight infill LINs |

Generate, Auto position and Simulate's IK all run on the window's thread, so the app freezes for the whole time. Auto position also plans the part, and then Generate plans it again.

## 1. Fewer, better moves: arcs and merged lines (biggest win; also the user's KRL request)
- Today every 2 mm resample point becomes a `LIN`, so the blends come out as hundreds of tiny LINs.
- **Measured:** WearPlate2 would go from 38,853 moves to 463 `LIN` + 1,053 `CIRC` = **1,516 (4%)**, at a tolerance of 0.05 mm and 0.5° of gun change per merged run.
- **Do:** a `toolpath/simplify.py` pass after planning, for gun-on LIN runs:
  - merge collinear runs whose orientation stays within the limit into one LIN;
  - fit circles to the remaining runs (3 or more points within the tolerance) and turn each into one `Move(kind="CIRC", aux=pose)`.
- **KRL writer:** `CIRC {aux}, {end}` with `$CIRC_TYPE = #PATH`, so the gun orientation follows the arc on a fillet, and `$ORI_TYPE = #VAR`. Blending with `C_DIS` as now. The speed stays `$VEL.CP`.
- **Simulator:** `sample()` interpolates along the arc. IK and collision then follow automatically, with about 3× fewer samples, because sampling goes by length and not by move.
- **Check on the robot (T1):** the orientation behaviour of `CIRC` with `#PATH` on a blend, and the advance run with C_DIS between CIRCs.
- **Tests:**
  - a fillet on test parts becomes CIRC;
  - points at most 0.05 mm off the planned path;
  - KRL syntax;
  - the KRL regression hashes for flat Layers paths may change if LIN merging applies to them. Merge Layers too, but update the hashes knowingly.

## 2. Remove per-move scipy and Python overhead (no behaviour change)
- **`level_gun`:**
  - compute `base_world` once per distinct table-axes value (cache by tuple); a job almost always has one;
  - replace `frame(c=turn)` with a direct 2×2 rotation about tool X;
  - vectorise over all moves with numpy (`einsum` for the projections and the twist angle).
  - Expected: 24 s → under 1 s.
- **`spray_frame` / Surface pose building:** build all frames of a stroke in one numpy call instead of once per point.
- **`path.sample`:**
  - replace the per-move `Slerp` with one vectorised quaternion slerp over all moves;
  - skip interpolation when the start and end orientations are equal (most Layers moves).
  - Expected: 40 s → about 2 s.
- **`base_in_robot` per sample:** cache it per axes tuple, in IK and in the collision `_place`.
- **IK (`Robot.ik_path`):**
  - vectorise the OPW closed form over all poses (numpy arrays of 8 solutions each);
  - then choose the branch nearest the previous joints in a simple loop.
  - Expected: 55 s → about 3 s.
- **Forward kinematics in the collision `_place`:** precompute all link frames for all samples with vectorised FK before the loop, leaving only FCL queries (22 s → about 2 s).

## 3. Keep the window responsive
- **Run on the worker thread:** Generate (`plan_part`), Auto position and Simulate's IK, as the collision check already does, with `JobProgress` and Cancel.
- **Cache the plan:** key it on (part placement, Part choices, process values, mode, tilt, rotation, gun angle), so Generate after Auto position doesn't plan again.
- **Collision checker:**
  - build it in the background once the cell assets load (17 s), and keep it across Generates (already done unless E1 changes);
  - consider caching the FCL-ready meshes, or decimating the cell meshes further than now (the robot is already decimated by 0.85).

## 4. Smarter checking (after 1–3)
- **Collision, coarse to fine:** check every 10th sample, then every sample only near hits and the clearances found. FCL distance queries would give a margin to skip by.
- **Auto position:**
  - the coarse grid already uses a few sampled poses; reuse one plan across E1 values (it already does) and across tilts where the toolpath in BASE doesn't depend on tilt (Layers/Surface without gun levelling).
  - Plan once at tilt 0 and only re-level the gun per tilt (cheap after step 2).
- **Layers on tall parts (WearPlate2, about 800 layers):**
  - consecutive layers with the same region could reuse their strokes (offset in Z);
  - Auto position could check only a sample of layers.

## 5. Automatic purge point (user request)
- **Purpose:** a "Suggest purge point" button in the purge section. It fills in the point and spray direction, and every value stays editable, as now.
- **How:** use the "Filtration unit" group bounding box in the cell meshes (`sim/cell_model.py` OPTIONAL).
  - Aim along the horizontal line from the robot (at the current E1) towards the unit's centre.
  - Put the nozzle exit standoff + clearance in front of the unit's near face.
  - Point `spray_dir` at the unit.
- **Then:** run the existing purge reach check (roll and E1 search), and if it isn't reachable, step the point back towards the robot.
- **Tests:**
  - the suggestion is reachable in the default cell;
  - `spray_dir` points into the unit's bounding box;
  - the user edits survive.

## Suggested order
1. **Step 2 (overhead):** quick and safe. The test suite should catch any regressions, and the KRL hashes must not change.
2. **Step 1 (arcs and merged lines):** big gains in planning, simulation and KRL size, and the user wants it.
3. **Step 3 (threads and caching):** freezes gone.
4. **Step 5 (purge suggestion):** small and independent.
5. **Step 4 (smarter checking):** only if still slow.
