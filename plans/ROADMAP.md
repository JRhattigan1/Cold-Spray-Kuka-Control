# Roadmap: where each feature stands (2026-10-28, end of session 5)

This is the status by feature. The next steps and open decisions are in [NEXT_SESSION.md](NEXT_SESSION.md), and the per-agent detail is in `reports/`.

**Branches:**
- **`main`:** sessions 1–4 (`surface-patterns` merged, work in progress).
- **Session 5 `wp/*`:** KRL safety (K1–K4, KM), speed (OP1, OP2), UI scaling and dark mode (U20–U24, FM). Not merged yet; see NEXT_SESSION "State".
- **Nothing is pushed.**

| Feature | Works now | Left | Risks |
|---|---|---|---|
| **Constraints and datum planes** | <ul><li>Solver: all 9 types, conflict status, degrees of freedom left (`cad/constraints.py`).</li><li>XY/XZ/YZ datum planes in the cell view.</li><li>Constraints panel in Position, with part faces picked in the 3D view (*surface-patterns*).</li></ul> | <ul><li>Save constraints with the Part step choices.</li><li>Pick references (datum, faceplate) in 3D too.</li><li>Check by hand that real mouse picking works; the tests use a stub.</li></ul> | — |
| **Infill patterns** | <ul><li>zigzag, oneway, concentric, spiral, crosshatch and islands in Layers.</li><li>Path coloured by move type.</li><li>`raster_start_deg` sets the first direction (*surface-patterns*).</li></ul> | <ul><li>Crosshatch with the default 90° step repeats a direction (0, 90, 90, 0): user decision.</li><li>Rotating the contour seam (changes KRL hashes).</li></ul> | — |
| **Contour tilt** | Outward lean, holes included, smoothed normals. | Optional cap on the orientation change per move. | Wrist flips: T1 first. |
| **Coat faces and Surface mode** | <ul><li>Pick, grow and clear coat faces.</li><li>Plane-cut zigzag.</li><li>Separate regions joined by jumps.</li><li>Turns lift by jump height.</li><li>Any infill pattern on a region that faces one way, with fallback to zigzag (*surface-patterns*).</li></ul> | <ul><li>Split a wrapping region per CAD face so the end face gets the pattern.</li><li>The zigzag still sprays across holes.</li><li>Pattern step-over is measured in the flattened plane.</li><li>`wp/P8` (turn clearance) is slow and clashes with jump height 0: rework or drop.</li></ul> | Blends tilted past `surface_max_tilt_deg` are skipped (4,284 points on WearPlate2). |
| **Gun hold** | <ul><li>Gun angle.</li><li>Minimal-twist hold: no winding, wrist up or towards the robot (*surface-patterns*).</li></ul> | A Simulate check for A4/A6 travel against a cable wind-up limit. | Cables and powder line are not in the CAD. |
| **Simple / Advanced UI** | Header toggle; the Toolpath and Position steps follow it. | `wp/E2p` (Part step): its test hangs. Final UI sweep. | — |
| **Purge point** | <ul><li>Purge visits (start, end, every N layers), ghost marker, reach chip, save to `cell.yaml`.</li><li>Suggest purge point at the filtration unit (*surface-patterns*).</li></ul> | <ul><li>Drag the marker.</li><li>Confirm the spray is on between layers (user).</li></ul> | `spray_dir` default is a CHECK. The purge transit is sprayed. |
| **Turns off the part** | `turn_mode`: inside, overrun, or beyond a datum plane (`planner/turns.py`). | `turn_plane` picker in the form. | — |
| **Solver** | Arm, gun angle and lean, re-route, and E1-per-layer levers; the "Solve problems" button; unfixed items block Save. | <ul><li>Cancel inside a lever.</li><li>Solver rows in the export confirm.</li><li>Following E1 beyond 1000 mm in the collision check.</li></ul> | E1 changes are PTP moves on the robot; `wp/K2` puts them on gun-off moves only. |
| **KRL base on the part** | Face + edge pick; LIN points relative to it under `BASE_DATA[part_base_number]`; header values; check list (*surface-patterns*). | Other ways to define it (3 points, typed values) if wanted. | Must be taught on the real part. |
| **Parts fixed in the cell** | Mounted: Fixed in the cell. The table is held at 0, there are no table checks, and Auto position searches E1 only (*surface-patterns*). | Nothing known. | The table is driven to 0 by the program. |
| **Speed** (`OPTIMISATION_PLAN.md`) | <ul><li>Gun hold, sampling, IK and collision forward kinematics are vectorised, with identical output.</li><li>Generate and Auto position run on a worker thread with a plan cache.</li></ul> | <ul><li>**Arcs and merged LINs:** `wp/K1` (O5 finished), held for review and T1.</li><li>Layers planning 4.5x faster on test_tube: `wp/OP2`, not merged.</li><li>Reach IK still runs on the GUI thread.</li><li>Background collision checker.</li><li>Faster Layers planning for tall parts.</li></ul> | CIRC orientation on the robot (`#PATH`): T1 check. |
| **KRL safety** | PTP joints from `robot.ik` seeded from the last PTP. | <ul><li>`wp/K2`: PTP joints as Simulate (`program_joints`); hook not in `krl.py` yet.</li><li>`wp/K3`: flip check inside LIN/CIRC.</li><li>`wp/K4`: program split, TOOL/BASE pre-flight, S/T bits (off).</li><li>PTP sampled in joint space and HOME legs simulated (finding 3).</li><li>Gantry carriage in collision (7a).</li></ul> | S/T bits 0 and 1 unverified. CIRC `#PATH` and `C_DIS` unverified. |

## Order
1. **Merge session 5** (K2–K4 via `wp/KM`, OP1, OP2, U21–U24, U20 after its tests), then the full suite and the by-hand check list.
2. **Apply the K2 PTP joints hook to `krl.py`;** review the KRL diff.
3. **K1 arcs:** review the hash diff, then T1 on a fillet.
4. **Open user decisions:** listed in NEXT_SESSION.
5. **Robot cell checks:** the README section "Before running on the real robot", each in T1 first.
