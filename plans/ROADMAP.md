# Roadmap: where each feature stands (2026-10-29, after session 11)

Status by feature. Next steps and open decisions: [NEXT_SESSION.md](NEXT_SESSION.md); by-hand checks: [HAND_CHECK.md](HAND_CHECK.md); per-agent detail: `plans/reports/` (in main's history).

**Nothing has run on the real robot yet.** Everything below is checked in simulation and tests only.

| Feature | Works now | Left | Risks |
|---|---|---|---|
| **Part and position** | STEP / built-in parts; turntable or fixed in the cell; constraints (9 types) picked in 3D; Auto position; SpaceMouse; **undo/redo** of Position edits incl. purge (Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y); **KRL base** by face + edge, 3 planes or vertex (hover preview), spin / Flip Z. | Spin box with focus takes Ctrl+Z itself. | — |
| **Job files** | `*.csjob` (YAML, versioned): part, placement, constraints, KRL base, all process settings, E1, purge, export options. Job button with recent jobs, Ctrl+S / Ctrl+O. | View checkboxes not saved. Open does not regenerate (press Generate). | — |
| **Layers** | zigzag, oneway, concentric, spiral, islands; contour passes and tilt; overrun from the part outline; gun stays on between layers at jump 0; part is not a collision body (not built yet). | Rotating the contour seam (changes KRL hashes). | Wrist flips on contour tilt: T1 first. |
| **Surface** | Coat-face picks, split per CAD face (gun off between); **step-over measured along the surface** over flat→curve transitions; turns keep the next stroke's roll (repair patch top + 4 sides fully reachable). | Ring cuts on cylinders keep plain spacing; holes still sprayed across. | Blends tilted past `surface_max_tilt_deg` are skipped. |
| **Revolve** | Tube-style parts on the turntable. | — | — |
| **Arcs** | CIRC + merged LINs behind `ProcessConfig.arcs` (off by default). | T1 check: `#PATH` orientation, C_DIS, `$CIRC_TYPE` in split parts. | Unverified on the robot. |
| **Simulate checks** | Reach, gun > 70° down (warning), flips inside LIN, collisions (robot, gun, table, gantry carriage, cell), **HOME legs checked as the KRL moves them**, **A4/A6 cable wind-up warning** (`cell.cable_windup_deg`, 270° placeholder). PTP moves drawn along their joint path. | HOME legs are not drawn. HOME at E1 2730 and E1 5500 collide in the model (S9F: check on the cell). Filtration-unit hits along whole paths come from the elbow-down first IK pick, not the model; Solve fixes them (elbow-up weighting is a user decision). | Wind-up limit is a guess: set from the cable dressing. |
| **Thickness and estimates** | Predicted coating thickness map (Gaussian track, FWHM = track width, calibrated to layer thickness), Simulate toggle with min/mean/max and % thin (grey = uncoated); cycle time and powder use (`powder_feed_g_min`) in Simulate and Export. **Build-up** toggle (session 10): Layers shows finished layers solid and the current layer track by track; other modes fill the thickness map in with playback. | Runs on the GUI thread (~4 s on test_tube); no shadowing or deposition efficiency. Build-up: flat strips, turns not drawn, overrun lost once a layer is finished, deposit steps kept in memory. | Model only: calibrate against a sprayed coupon. |
| **Solver** | Levers: gantry E1, arm, gun angle, re-route, E1 per layer, purge point (XYZ only); "trying …" status; Apply suggested set-up; HOME-leg issues reported. 2.1× faster in run 8; session 11: reroute 29 → 10 s, gantry 53 → 41 s (noisy). | No lever fixes HOME-leg problems directly; gantry lever still slow (checker per E1). | — |
| **Purge** | Visits at start/end/every N layers, suggest point at the filtration unit (+Y), clearance over the part on every trip; spray on from first to last visit. | Drag the marker. | Spray is manual for now. |
| **KRL export** | PTP joints as simulated; E1 changes on gun-off PTPs; KRL base on the part; **pre-flight HALT, split into parts and spray-by-hand (`$OUT` as comments) now wired into Save**. | Controller checks (to-do list). | S/T bits unverified; robot dimensions not yet confirmed against `$machine.dat`. |
| **Cell data** | Cell button: Tool tab (TOOL_DATA per nozzle, pendant paste, CAD comparison; KRL selects the nozzle's tool number), Base tab (BASE_DATA stored), HOME tab (jog, live robot, collisions, save `home_deg`). | Taught bases not in the KRL header; 3D TCP refresh needs a restart; HOME save not blocked on collisions. | Radial tool number 2 is a guess. |
| **Tooling** | CI on GitHub (KRL hashes, imports, planner/post units); `tools/bench_solve.py`, `tools/bench_plan.py`; golden solver test. | — | — |

## Order
1. By-hand check in the app ([HAND_CHECK.md](HAND_CHECK.md)).
2. T1 on the robot, spray off: robot dimensions vs `$machine.dat` / `$POS_ACT`, then test_block (arcs off, then on), split program, pre-flight HALT. README "Before running on the real robot".
3. Calibrate the thickness model and powder feed against a sprayed coupon; set the wind-up limit from the cable dressing.
4. Remaining items in NEXT_SESSION's to-do list.
