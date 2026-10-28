# Roadmap: where each feature stands (2026-10-28, end of session 5)

This is the status by feature. The next steps and open decisions are in [NEXT_SESSION.md](NEXT_SESSION.md), and the per-agent detail is in `reports/`.

**Branches:**
- **`main` @ `4087bf2`:** sessions 1–5 (session 5: OP1–OP3, KM = K2+K3+K4, U21–U24). Full suite not run since.
- **Held:** `wp/K1` (arcs), `wp/KM-K1`; sidebar fit `wp/F3` → `wp/F1` → `wp/F2fit` (`wp/FM` plan), `wp/U20`. See NEXT_SESSION "Kept branches".
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
| **Solver** | Arm, gun angle and lean, re-route, and E1-per-layer levers; the "Solve problems" button; unfixed items block Save. | <ul><li>Cancel inside a lever.</li><li>Solver rows in the export confirm.</li><li>Following E1 beyond 1000 mm in the collision check.</li></ul> | E1 changes are PTP moves on the robot, on gun-off moves only. |
| **KRL base on the part** | Face + edge pick; LIN points relative to it under `BASE_DATA[part_base_number]`; header values; check list (*surface-patterns*). | Other ways to define it (3 points, typed values) if wanted. | Must be taught on the real part. |
| **Parts fixed in the cell** | Mounted: Fixed in the cell. The table is held at 0, there are no table checks, and Auto position searches E1 only (*surface-patterns*). | Nothing known. | The table is driven to 0 by the program. |
| **Speed** (`OPTIMISATION_PLAN.md`) | <ul><li>Gun hold, sampling, IK and collision forward kinematics are vectorised, with identical output.</li><li>Generate and Auto position run on a worker thread with a plan cache.</li><li>Session 5: tube Layers 85.4 → 14.8 s, tube Surface 11.7 → 1.7 s, identical output.</li></ul> | <ul><li>**Arcs and merged LINs:** `wp/K1` (O5 finished), held for review and T1.</li><li>Reach IK still runs on the GUI thread.</li><li>Background collision checker.</li><li>Faster Layers planning for tall parts; slicer `polygonize` (changes output).</li><li>Launch time (pyvistaqt).</li></ul> | CIRC orientation on the robot (`#PATH`): T1 check. |
| **KRL safety** | <ul><li>PTP joints as Simulate (`post/joints.py`).</li><li>Flip check inside LIN/CIRC.</li><li>E1 changes on gun-off moves only; no stale E1 root in collision.</li><li>Split, TOOL/BASE pre-flight and S/T modules (off).</li></ul> | <ul><li>PTP sampled in joint space and HOME legs simulated (finding 3).</li><li>Export uses the split / pre-flight modules (`wp/F2fit`).</li><li>Axis fallback in sampling (finding 8).</li><li>Gantry carriage in collision (7a).</li></ul> | S/T bits 0 and 1 unverified. CIRC `#PATH` and `C_DIS` unverified. |

## Order
1. **Merge the sidebar-fit branches** (F3 → F1 → F2fit, then U20), then the full suite and a by-hand check in the app.
2. **K1 arcs:** rebase, review the hash diff, full suite, T1 on a fillet.
3. **Review findings 3, 7a, 8.**
4. **Open user decisions:** listed in NEXT_SESSION.
5. **Robot cell checks:** the README section "Before running on the real robot", each in T1 first.
