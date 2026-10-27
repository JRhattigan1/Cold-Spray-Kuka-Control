# Feature plan: placement constraints, new toolpaths, purge point, simple/advanced UI

Date: 2026-10-27 · Repo: Cold-Spray-Kuka-Control · Base commit: `2b3505f`

> First action once this plan is approved: copy this file to
> `plans/2026-10-27-feature-plan.md` in the repo. Not `docs/`, which is gitignored.
> Then commit it on `main`. This file is the shared brief for every agent and the progress tracker.

---

## 0. Original request (verbatim)

> okay we are going to make a big plan doc that will be exceuted agentically. I am going to list many features to be done, each agent will have 45 mins to get as far as they can then they will have to stop, write a summary report on what was done and feed that back to a review agent so that it can be picked up and resumed if left unfinihsed.
>
> Here is a big list of features to be implemented:
> Constraint system like simens NX, touch constraints, distance, centring, parralel etc. part of this Add togglable planes to the table for x, y , z constraints
> new slicing path generation, not just contouring and zig zag
> optional tilt on boundary passes
> ability to select a single or group of faces to be coated to define the toolpath area
> general UI improvements i think we should have simple and advanced options that are split and togglable
> ability to define a warm up point in free space, this would be a powder purging point that could be returened to before and after the deposition
> ability to make turning points off of the component, ie, for a big zig zag path the guns turning points are past a given face so that overspary could be captured with a mask
>
> plan all this out, i'll give feedback then we will exectute agentically
> save this promt and the following plan in a markdown file for futrure reference

### Decisions made while planning
| Topic | Decision |
|---|---|
| Constraints | **Part ↔ cell references only.** Faces, cylinder axes and edges of the placed part are constrained to the faceplate, the substrate top and the new X/Y/Z datum planes. Bodies in the STEP keep their CAD relationship to each other. |
| Face selection | **New "Surface" strategy** next to Layers/Revolve. Conformal raster over the selected faces with the gun normal to the surface at standoff, N coats, curved faces allowed. |
| Purge point | **Spray on, stays on.** PTP to the purge point, gun ON, dwell, then travel to the part with the spray still on (lifted, rapid). Return to the purge point at the end, and optionally every N layers. Gun OFF only at the end. |
| Execution | **Parallel waves in git worktrees.** A review agent merges each wave before the next starts. |

---

## 1. Codebase primer (read this first, every agent)

Python 3.12, PySide6 + PyVista GUI, OCP for CAD, shapely/trimesh for geometry, pydantic configs. It produces a KRL program for a KUKA KR 50 R2500 on gantry E1, with tilt/rotate turntable E2/E5.

Data flow, in [pipeline.py](src/coldspray/pipeline.py) `plan_part()`:

STEP → [cad/step_io.py](src/coldspray/cad/step_io.py) `tessellate_bodies` → [cad/part.py](src/coldspray/cad/part.py) `Part`
(mesh, `face_ids`, `planar: {face_id: PlanarFace}`, `placement` 4×4 part-CAD→BASE, `base_face`, `start_face`, `build_body`)
→ placement in the Position step (`constrain`, `snap_face_down`, `drop_on_plate`)
→ planner: [planner/layers.py](src/coldspray/planner/layers.py) `plan_layers` (contour loops + zig-zag raster, greedy `_order`, jumps), or [planner/revolve.py](src/coldspray/planner/revolve.py)
→ [toolpath/model.py](src/coldspray/toolpath/model.py) `Toolpath` of `Move(pose, speed, gun_on, kind, axes)`. Poses are TCP frames in BASE, tool X = spray direction. Built with `spray_frame(point, spray_dir, travel_dir)`.
→ reach and gun angle checks ([planner/position.py](src/coldspray/planner/position.py) `auto_position`, `gun_down_deg`), FCL collisions ([sim/collision.py](src/coldspray/sim/collision.py))
→ [post/krl.py](src/coldspray/post/krl.py) `to_krl`.

Config: YAML in [config/](src/coldspray/config/) → pydantic models in [models.py](src/coldspray/models.py). Each `ProcessConfig` field's `description` is also its GUI tooltip and its YAML comment.

GUI: [gui/app.py](src/coldspray/gui/app.py) `MainWindow` is built from one mixin per step in [gui/steps/](src/coldspray/gui/steps/) (part, position, toolpath, simulate, export). The steps are joined by a step bar with done/stale states (`_mark_done`, `_invalidate`). Shared widgets are in [gui/widgets.py](src/coldspray/gui/widgets.py): `Section` (collapsible, with a chip), `spin`, `primary_button`, `wrap_label`. The cell 3D scene is [sim/viewer.py](src/coldspray/sim/viewer.py) `Scene`. The part-only view for step 1 is [sim/part_view.py](src/coldspray/sim/part_view.py) `PartView`, which handles face picking via `start_pick` and `pick_triangle`. Built-in test parts are `test_block`, `test_tube` and `test_repair` in `pipeline.BUILT_IN`.

Style rules: match the existing code. That means short modules, dense code, and docstrings that say *what/why* in plain English. Write pure geometry in planner/cad modules with no Qt, and keep GUI code thin. Every new pure function gets a pytest test. Never commit anything under `assets/` or `docs/`.

---

## 2. How the agentic run works

### 2.1 Roles
- **Worker agent**: owns one work package (WP) for one **30-minute** slot (WP0 had 45 minutes; cut to 30 from wave 1 on, 2026-10-27, to fit the morning).
- **Review agent**: runs between waves. It reviews, tests and merges, updates the tracker (§5) and writes resume briefs.
- **Human (Joe)**: approves this plan and does the GUI smoke test after each wave in §6, which is optional but recommended.

### 2.2 Worker protocol (paste into every worker prompt)
1. **Start:** run `date` and record the start time. Read §1, your WP section, and any earlier reports for this WP in `plans/reports/`.
2. **Workspace:** use your own worktree **outside OneDrive** to avoid sync churn:
   `git worktree add C:\Users\qrb21117\worktrees\cs-<WP> -b wp/<WP> main`. If you are resuming, check out the existing branch `wp/<WP>` instead.
   Then link the CAD assets, which are not in git: `cmd /c mklink /J C:\Users\qrb21117\worktrees\cs-<WP>\assets "<main repo>\assets"`.
3. **Tests must import the worktree, not main.** The venv has an editable install that points at the main checkout, so always run:
   `$env:PYTHONPATH="C:\Users\qrb21117\worktrees\cs-<WP>\src"; C:\Users\qrb21117\.venvs\coldspray\Scripts\python.exe -m pytest -q`
   GUI tests use `QT_QPA_PLATFORM=offscreen`; copy the pattern in `tests/test_view_bar.py`.
4. **Work order:** pure logic and tests first, then wiring, then GUI. Commit on `wp/<WP>` after every green step, with small commits and the attribution trailer. Never push. Never commit to `main`.
5. **Stay in your lane:** edit only the files your WP lists. If you need a change elsewhere, write it down in the report under "needs from other WPs" and don't make it. Exception: one-line hooks explicitly allowed by the WP.
6. **Time:** run `date` between steps. **At 22 minutes, stop coding.** Get to a commit where the tests pass. If you can't, revert the half-done change, and if it's worth keeping, stash it to a `wp/<WP>-wip` branch.
7. **Report:** write `plans/reports/<WP>-<attempt>.md` using the template in §2.4 and commit it on your branch. The report is your final message as well.

### 2.3 Review agent protocol (after each wave)
For each WP branch in the wave, in the merge order the wave gives:
1. Read the report and `git diff main...wp/<WP>`. Check the change against the WP's acceptance criteria and the style rules in §1.
2. Run the full test suite on the branch, with `PYTHONPATH` set as in §2.2.
3. Merge into `main` with `git merge --no-ff wp/<WP>`. Resolve conflicts, which should be trivial by design. Rerun the full suite on `main`. If a merge breaks the tests and the fix isn't obvious, don't merge; mark the WP "blocked" with the reason.
4. Partial work that is still green is merged too, and the WP is marked `partial`. Then write `plans/reports/<WP>-resume.md`. It covers what is done (files and functions), what is left as a checklist, known pitfalls, and the exact next step. The next wave's worker for that WP starts from that file.
5. Update the tracker in §5 of `plans/2026-10-27-feature-plan.md` and commit it on `main`.
6. Remove merged worktrees with `git worktree remove ...`, but keep the branches.
7. Write `plans/reports/wave-<n>-review.md` with what merged, test count before and after, anything blocked, and anything for the human to check in the GUI.

### 2.4 Report template
```markdown
# <WP id> attempt <n> - <done | partial | blocked>
Branch: wp/<WP> · last commit: <sha> · time used: <min>
## Done
- <behaviour>, in <file:function>, tested by <test>
## Not done / remaining (checklist)
- [ ] ...
## Decisions & deviations from the plan
## Needs from other WPs / follow-ups
## Exact next step if resumed
## Test result
<pytest summary line>
```

---

## 3. Work packages

Each WP lists its owned files and acceptance criteria. "New" means a new file. The WPs were sized for 45 minutes; with 30-minute slots expect the larger ones (A1, A3, B1, D2, F2, E2) to end partial and be resumed. All new config fields go in **WP0**, so later WPs never edit `models.py` or `process.yaml` concurrently.

### WP0: Foundations and shared formats (wave 0, run alone)
Prepares every shared format so the later parallel WPs don't collide.
- **`toolpath/model.py`:** add `Move.tag: str = ""` (`"contour"`, `"infill"`, `"turn"`, `"jump"`, `"purge"`, `"approach"`) and `Move.dwell_s: float = 0.0` (wait at this point after arriving). `transformed()` and `rolled()` must copy both fields. `plan_layers` tags its moves.
- **`post/krl.py`:** if `dwell_s > 0`, write `WAIT SEC <dwell_s>` after the LIN/PTP, and make sure the move before it is not blended with `C_DIS`. Add a test in `test_krl.py`.
- **`models.py`:** add these new `ProcessConfig` fields, each with a default that keeps today's output byte-identical:
  `infill_pattern: Literal["zigzag","oneway","concentric","spiral","crosshatch","islands"]="zigzag"`,
  `contour_tilt_deg: float=0` (0 = off),
  `turn_mode: Literal["inside","overrun","beyond_plane"]="inside"`, `turn_plane: str=""` (a datum-plane name, used with `beyond_plane`); `overrun_mm` already exists,
  `surface_coats: int=1`, `surface_max_tilt_deg: float=60`.
  Every field takes a `json_schema_extra={"level": "basic"|"advanced"}`. This replaces the `BASIC` tuple in [gui/steps/toolpath.py](src/coldspray/gui/steps/toolpath.py). Add the new fields to `process.yaml` too.
- **`models.py`:** add a new `PurgeConfig` model with `enabled`, `point` (x, y, z WORLD mm), `spray_dir` (WORLD vector, default horizontal toward the cell's spray booth, CHECK on the cell), `dwell_s`, `at_start`, `at_end`, `every_n_layers: int=0`, `approach_mm` (lead-in distance, back from the point along the spray dir). Then add `CellConfig.purge: PurgeConfig | None = None`. Add a commented example block to `cell.yaml`.
- **`cad/step_io.py`, `cad/part.py`:** record cylindrical faces as well as planes. Add `CylFace(point, axis, radius, area)` and `Part.cylinders: dict[int, CylFace]`. This feeds the constraints (centring) and surface coating. `Part.without()` must filter the cylinders the same way it filters `planar`.
- **`gui/steps/toolpath.py`:** the process form is built generically. Currently `int` fields get a `QSpinBox` and `float` fields a `QDoubleSpinBox`. Add: `Literal` → `QComboBox`, `bool` → `QCheckBox`, `str` → hidden unless it is handled. `read_process()` must read all of these.
- **Accept when:** the full suite is green. Generating the 3 built-in parts gives the same `to_krl` output as before. Add a regression test that compares against `main` behaviour, e.g. a hash of `to_krl` for `test_block`.

### Feature A: Constraint system (NX-style) and datum planes
**A1 Constraint solver (pure).** New files: `cad/constraints.py`, `tests/test_constraints.py`.
- Entities: `Plane(point, normal)`, `Axis(point, dir)`, `PointRef`. They are built from a part face (`planar`, `cylinders`; transformed by the placement) or from a cell reference: `faceplate` (z=0), `substrate_top`, datum planes `XY`, `XZ`, `YZ` through the table centre, each with an optional offset.
- Constraint types:
  - `touch`: coincident planes, opposed normals
  - `align`: coincident planes, same normals
  - `distance(d)`: parallel planes at offset d
  - `parallel`
  - `perpendicular`
  - `angle(θ)`
  - `centre`: cylinder axis on a datum axis or on another axis (concentric), or a point on an axis
  - `centre_between`: midplane of two opposite part faces on a datum plane (symmetric)
  - `fix`: lock the current pose
- Solver: `solve(placement, constraints) -> SolveResult(placement, residuals, dof_left, status)`. Use `scipy.optimize.least_squares` over a rotation vector plus translation, starting from the current placement so the result is the smallest move. Detect `over_constrained` / `conflicting` (residual > tol) and report the remaining DOF from the Jacobian rank.
- "Keep on plate" is equivalent to `touch(base_face, floor)`. Add a test that `constrain()` and the solver agree. Don't remove `constrain()` yet.
- Accept: tests cover each type on `test_block`/`test_tube`/`test_repair`, plus the conflict case, DOF counting, and solve time under 50 ms.

**A2 Datum planes in the cell view.** New file `sim/datums.py`. Hooks allowed in `sim/viewer.py` (≤ 15 lines) and [gui/view_toolbar.py](src/coldspray/gui/view_toolbar.py).
- Three translucent, togglable planes XY/XZ/YZ on the faceplate (they move with tilt/rotation like BASE), sized to the faceplate. Each has an editable offset and a label. A view-toolbar button opens a menu with a checkbox per plane.
- Planes must be pickable by name, so `Scene` returns `"datum:XZ"` from a pick. This is shared with A3.
- Accept: offscreen test that toggles each plane, checks the actor's visibility and transform at tilt 30°, and picks the plane by name.

**A3 Constraint UI in the Position step.** Owned files: [gui/steps/position.py](src/coldspray/gui/steps/position.py), a new `gui/constraints_panel.py`, and pick support in `sim/viewer.py`, which can pick part faces in the cell view, reusing the `highlight_face` path.
- A "Constraints" Section with an "Add" menu of types. Adding one starts a two-pick flow: a part face, then a reference (a datum plane, the faceplate or the substrate). The value is edited in place. Each row can be suppressed, deleted and shows a status icon (ok / conflict). There is a DOF chip, e.g. "2 DOF free". Each placement change re-solves the constraints.
- Dragging and the SpaceMouse are projected onto the free DOF: after each drag, solve with the drag pose as the start.
- Constraints are kept on the window, like `roles`, and saved as part of the Part step choices so they reset when a different part is loaded.
- Accept: offscreen GUI test that adds a touch and a centre on `test_tube`, checks the placement, and tests suppress and delete. Manual check: drag the part with constraints on.

### Feature B: New slicing and infill patterns
**B1 Pattern library (pure).** New files: `planner/patterns.py`, `tests/test_patterns.py`.
- Move `contour_loops`, `raster_segments` and `_order` here out of `layers.py`. Leave re-exports in `layers.py` so imports still work.
- Each pattern has the signature `fn(region, proc, layer_index) -> list[stroke ndarray]`, collected in a `PATTERNS` dict. The patterns:
  - `zigzag`: today's behaviour
  - `oneway`: all strokes in the same direction, with a lifted return
  - `concentric`: inward offset loops at `step_over`
  - `spiral`: concentric loops joined into one continuous path
  - `crosshatch`: two perpendicular rasters per layer, each at half density or a full layer each
  - `islands`: checkerboard tiles with alternating raster direction, to reduce residual stress
- Seam control: rotate each layer's contour start point so seams don't stack.
- Accept: per pattern, coverage ≥ 95 % of the region (union of strokes buffered by track_width/2), no stroke outside the region, deterministic output. `zigzag` matches the old output exactly.

**B2 Wire patterns into Layers.** Owned files: `planner/layers.py`, `sim/viewer.py` (colour by `Move.tag`, ≤ 20 lines).
- `plan_layers` uses `PATTERNS[proc.infill_pattern]`. The combo box comes from WP0's generic form. Path lines are coloured contour vs infill vs jump.
- Accept: every pattern generates on all built-in parts, and the reach and collision pipeline runs. Snapshot tests on the move counts.

### Feature C: Optional tilt on boundary passes
**C1** Owned files: `planner/layers.py` (contour section only), new `tests/test_contour_tilt.py`.
- When `contour_tilt_deg > 0`, contour strokes are built with `spray_dir` = DOWN rotated *outward*, towards the local outward edge normal of the ring, by the tilt angle. The aim point stays half a track inside the edge, and the nozzle sits back along the tilted axis at the standoff.
- Smooth the normals with a moving average over about one track width, so corners don't flip the wrist. Limit the change in orientation per move. Use the ring orientation from shapely to get the outward direction for both exteriors and holes.
- Infill stays vertical. Leave the `gun_max_down_deg` check to the existing checks.
- Accept: on `test_block` with a 20° tilt, every contour pose is 20° ± 0.5° from vertical and leans outward (including in the hole), and the standoff distance to the aim point equals `standoff_mm`. Reach is still OK after auto-position.

### Feature D: Face selection → Surface coating strategy
**D1 Face selection in the Part step.** Owned files: [gui/steps/part.py](src/coldspray/gui/steps/part.py), [sim/part_view.py](src/coldspray/sim/part_view.py), `cad/part.py` (`coat_faces` field only).
- Add `Part.coat_faces: set[int]` and a "Coat faces" pick mode: a click toggles a face, Shift+click adds, Ctrl+click removes, and a "Grow to tangent faces" button adds tangent-connected faces (normals within 10° across shared edges). Show the list and count, highlighted in a distinct colour, with Clear.
- `Part.without()` keeps the `coat_faces` that are still present.
- Accept: offscreen test that picks via `pick_triangle`, plus a pure test of tangent growth on `test_tube` (the outer cylinder grows to the outer cylinder only).

**D2 Surface planner (pure).** New files: `planner/surface.py`, `tests/test_surface.py`.
- `plan_surface(vertices, faces, coat_mask, proc, name, table_axes) -> Toolpath`:
  1. Take the submesh of the selected triangles.
  2. Pick the raster direction: the longest extent of the projection along the mean normal.
  3. Cut the submesh with parallel planes `step_over` apart (`trimesh.intersections.mesh_plane`) and chain the segments into polylines.
  4. Resample the polylines every ≤ 2 mm.
  5. Set the gun to `-smoothed vertex normal`, positioned at the standoff.
  6. Zig-zag between lines, with turns outside the area by `overrun_mm`, extended tangentially (this covers Feature G for surfaces).
  7. Repeat for `surface_coats`, alternating direction.
- Points where the normal is more than `surface_max_tilt_deg` from the mean normal are skipped and the count reported.
- Accept: the top face of `test_block` gives a flat raster equal to a plate raster. The outer face of `test_tube` with the gun normal gives a radius check at the standoff. Two adjacent faces of a box (an edge) give continuous strokes over the edge with a smooth normal blend.

**D3 Surface mode wiring.** Owned files: `pipeline.py`, [gui/steps/toolpath.py](src/coldspray/gui/steps/toolpath.py) (mode-specific enabling only).
- `MODES["surface"]`. `plan_part` routes to `plan_surface` using `part.coat_faces`. The coat faces may be on any body, not only the build body. If `coat_faces` is empty, show a clear error that sends the user back to the Part step.
- Auto-position works on the result, because it consumes a plan callback.
- Accept: generate in surface mode on `test_repair`, coating the patch's top face plus the sides, and reach, collisions and KRL export all run.

### Feature E: UI with Simple / Advanced split
**E1 Framework (wave 1).** Owned files: `gui/widgets.py`, [gui/app.py](src/coldspray/gui/app.py) (header toggle), [gui/steps/toolpath.py](src/coldspray/gui/steps/toolpath.py) (process form only).
- A `Simple | Advanced` segmented toggle in the header next to the theme button, persisted with `QSettings`. Add `widgets.advanced(w)` to register a widget as advanced-only; switching mode shows or hides all registered widgets.
- Process fields use `level` from WP0. In Simple mode, any advanced field that differs from the defaults is still shown, with a "changed" chip. Simple mode must never hide a value that changes the output without saying so.
- Rule for all later WPs: new controls that most users won't need get wrapped in `advanced()`.
- Accept: offscreen test that toggling hides and shows the registered widgets and that the mode survives a restart.

**E2 Sweep and polish (final wave, run alone).** Touches all GUI step files.
- Apply simple/advanced to the Part, Position, Simulate and Export panels. Proposed Simple set: Part = open, roles, start/base/coat faces. Position = X/Y/A, Keep on plate, Auto position, purge on/off. Toolpath = mode, pattern, basic process values. Everything else is Advanced.
- Polish list, picked from the review reports plus:
  - consistent units and decimals
  - tooltips on every control
  - keyboard shortcuts for the steps (Ctrl+1…5) and Generate (Ctrl+G)
  - empty states
  - remove dead "v1 limitation" docstrings
  - update the README's "Run" section for every new feature
- Accept: README updated, screenshots taken (offscreen) of each step in both modes and attached to the report for the human to review.

### Feature F: Warm-up / powder purge point
**F1 Purge in the path and KRL (pure).** New files: `planner/purge.py`, `tests/test_purge.py`. Touches `post/krl.py` only if WP0 left something out.
- `add_purge(path, cell) -> Toolpath` returns a copy of the path with the purge moves added. It works in WORLD, because the purge point is fixed in the cell while BASE moves with the table. To convert, use `inv(base_world(cell, axes)) @ world_pose` with the table axes in force at that move ([cell.py](src/coldspray/cell.py)).
- Sequence:
  - **Start:** PTP to the purge approach pose (gun off) → LIN to the purge pose, `gun_on=True`, `dwell_s` → LIN rapid with the spray on, to the first move's approach height.
  - **End:** after the last move, LIN rapid with the spray on to the purge pose → gun OFF → PTP home.
  - **Every N layers:** detour after the layer ends, using `tag` boundaries from WP0.
  - Purge moves are tagged `"purge"`.
- `plan_part` calls `add_purge` at the end when `cell.purge.enabled`. This is a one-line hook in `pipeline.py`, allowed for this WP only.
- Accept: test the poses in WORLD equal the configured point for tilt 0/30/60. Test the gun state sequence, that KRL contains the dwell, and that the KRL spray output does not toggle between the purge and the part.

**F2 Purge UI and marker.** Owned files: [gui/steps/position.py](src/coldspray/gui/steps/position.py) (new Section only), new `sim/markers.py`, ≤ 10-line hook in `sim/viewer.py`.
- Add a "Warm-up / purge point" Section with: Enabled, X/Y/Z, a spray direction preset (±X, ±Y, down-angled) plus a custom option, dwell, start/end/every N.
- The marker is a ghost gun at the purge pose and can be dragged with the existing affine widget pattern. Add a "Move gun here" preview with IK, showing a red chip if unreachable at the current E1.
- "Save to cell.yaml" writes `purge:` while keeping the comments: edit that block only.
- Accept: offscreen test that changing the values updates `cell.purge` and the marker transform. A manual check that the path in the Simulate step visits the point.

### Feature G: Turning points off the component (overspray mask)
**G1** Owned files: `planner/patterns.py` (raster functions), `tests/test_patterns.py`.
- `turn_mode="overrun"`: extend each raster stroke at both ends by `overrun_mm` along its direction. The extension is tagged `"turn"` with the spray on, and the turn connector between strokes lies entirely outside the region.
- `turn_mode="beyond_plane"`: extend each stroke until it crosses `turn_plane` (a datum plane from A2, or a picked part/fixture face, stored as plane point + normal in BASE), plus `overrun_mm`. If the plane is not ahead of the stroke, fall back to overrun and add a warning.
- In both modes, connectors between neighbouring strokes are kept outside: greedy `_order` must not jump across the part. Pair the strokes so the U-turn is off the part.
- Contours are unchanged. Surface mode already supports overrun (D2).
- GUI: `turn_plane` gets a combo box listing the datum planes plus "Pick face…". This is the one place G1 edits `gui/steps/toolpath.py`, after D3 has merged.
- Accept: on `test_block`, every turn point is ≥ `overrun_mm` outside the region's bounding box along the raster direction, and coverage is unchanged. Beyond-plane with the YZ datum offset to 60 mm puts all turns past x = 60.

---

## 4. Waves (run order and parallelism)

Files owned per wave don't overlap, except for the small listed hooks, which the review agent merges.

| Wave | WPs run in parallel | Merge order | Why grouped |
|---|---|---|---|
| 0 | **WP0** | - | Shared formats and config fields first |
| 1 | **A1**, **B1**, **D2**, **F1**, **E1** | B1, A1, D2, F1, E1 | Pure modules in new files. E1 is the only GUI work |
| 2 | **B2**, **A2**, **D1**, **F2** | B2, A2, D1, F2 | Wiring. viewer.py hooks are small and separate |
| 3 | **A3**, **C1**, **D3** | C1, D3, A3 | position.py, layers.py, pipeline/toolpath.py: no overlap |
| 4 | **G1** + resumes of any partial WPs | - | patterns.py; needs A2 datums and D3 |
| 5 | **E2** (alone), then the final review | - | Touches every GUI file |

Partial WPs are resumed by a fresh agent in the next wave, using their `-resume.md` brief. A resume never runs in the same wave as a WP that owns the same files.

---

## 5. Progress tracker (the review agent updates this)

| WP | Status | Attempts | Branch | Last report |
|---|---|---|---|---|
| WP0 | todo | 0 | wp/WP0 | - |
| A1 | todo | 0 | wp/A1 | - |
| A2 | todo | 0 | wp/A2 | - |
| A3 | todo | 0 | wp/A3 | - |
| B1 | todo | 0 | wp/B1 | - |
| B2 | todo | 0 | wp/B2 | - |
| C1 | todo | 0 | wp/C1 | - |
| D1 | todo | 0 | wp/D1 | - |
| D2 | todo | 0 | wp/D2 | - |
| D3 | todo | 0 | wp/D3 | - |
| E1 | todo | 0 | wp/E1 | - |
| E2 | todo | 0 | wp/E2 | - |
| F1 | todo | 0 | wp/F1 | - |
| F2 | todo | 0 | wp/F2 | - |
| G1 | todo | 0 | wp/G1 | - |

Status values: `todo | in-progress | partial | done | blocked`.

---

## 6. Verification

- **Each WP:** the full pytest suite is green on its branch, with `PYTHONPATH` set to the worktree, and the WP's acceptance tests have been added.
- **Each wave (review agent):** the suite is green on `main` after the merges. The `to_krl` regression hash from WP0 is unchanged unless a WP deliberately changes output and says so.
- **Human GUI check after each wave**, about 5 minutes with `.\run`:
  - Wave 2: patterns on `test_block`, datum planes, coat-face picking, purge marker.
  - Wave 3: constraints on `test_tube` (centre + touch), contour tilt, surface mode on `test_repair`.
  - Wave 4: turn-off-part with the YZ plane.
  - Wave 5: walk through all five steps in Simple and Advanced.
- **Before the real robot:** every new motion runs in T1 with the spray off first. Purge transit with the spray on crosses free space, so check the collision results for purge moves specifically. The purge `spray_dir` default needs confirming on the cell (marked CHECK in `cell.yaml`).

## 7. Risks and notes
- **Editable install points at the main checkout.** Forgetting `PYTHONPATH` means a worktree silently tests `main`. This is the most likely agent mistake, so the review agent re-checks it.
- **OneDrive:** worktrees go in `C:\Users\qrb21117\worktrees\`, never inside the synced folder.
- **Wrist flips:** contour tilt (C1) and surface normals (D2) introduce orientation changes; normal smoothing is required. Watch for KRL singularities in Simulate.
- **Cell-view face picking (A3)** is new: hardware picking already works for drag handles (MSAA fix in `679db4d`). Reuse that approach.
- **Commits and merging:** approving this plan authorises agents to commit on `wp/*` branches and the review agent to merge into local `main`. Nothing is pushed.
