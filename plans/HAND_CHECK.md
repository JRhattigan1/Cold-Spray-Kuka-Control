# By-hand check in the app

Run `.\run` from the repo (everything is on main). Allow about 55 min.
For each line, tick it or write what you saw. Anything odd: note the part, mode and the exact click.

## 1. Window and layout (10 min)
Do this at 100 % and at 150 % Windows scaling (Settings → Display → Scale; restart the app after changing it).
- [ ] Starts at a sensible size, not off screen. Close it and reopen it: the sidebar width is remembered.
- [ ] Drag the splitter: the 3D view takes the space. The sidebar won't go narrower than its minimum.
- [ ] Step through Part → Position → Toolpath → Simulate → Export in **Simple** and **Advanced** mode. Check that nothing is clipped on the right, there's no horizontal scroll, and Next stays pinned at the bottom.
- [ ] Light and dark themes: the 3D background, toolbars, markers and step-bar labels are all readable. Done steps show crisp labels, not faint ones.
- [ ] The text "90°" shows correctly (no "90Â°").
- [ ] SpaceMouse plugged in: the "SpaceMouse moves part" checkbox appears, its tooltip names the device, and it doesn't widen the panel.

## 2. Part and Position (10 min)
- [ ] Load **WearPlate2** (the slow one): note the load time, and check the window doesn't freeze.
- [ ] Load test_tube. **Fixed in the cell**: the part stays put and the table goes to E2 = 0, E5 = 0.
- [ ] **Constraint picking**: pick a face and an edge, apply them, and check the part moves as expected. Undo works.
- [ ] **Gun hold** setting: changing it changes the path orientation after Generate.
- [ ] **KRL base** picked on the part: the base marker sits where you'd teach it.
- [ ] **Suggest purge point**: the point sits on the +Y side (back, filtration unit) and the spray arrow points +Y. It must not be inside the filtration unit box or behind the robot.
- [ ] **Purge actions** dropdown in Position works.

## 3. Toolpath (10 min)
- [ ] Mode combo: Layers / Revolve / Surface. Each one generates on test_tube.
- [ ] **Surface** on the tube end: try each pattern. Crosshatch should be gone.
- [ ] Layers with **jump height 0**: the gun stays on between layers (no off/on, no settle wait in the timeline).
- [ ] Layers with **turn mode = overrun** and 1–2 contour passes: the turns sit `overrun_mm` past the part edge, not just past the infill.
- [ ] **Surface**, pick the tube side and its end face together: each face gets its own raster, and the gun goes off between faces. Try the repair patch (top + 4 sides): all sides should be reachable (fixed in S9S).
- [ ] **Arcs** (Advanced → arcs on) on test_block: the hole contour becomes CIRCs, and Simulate follows the arc.
- [ ] **Cancel** during a long Generate: the old path stays and the app stays responsive. Generate again with the same settings: it's instant (plan cache). Change one setting: it re-plans.

## 4. Simulate (10 min)
- [ ] Play the whole program. Watch the purge → first point leg: it goes over the part, not through it.
- [ ] HOME → first move and last move → HOME are animated and collision-checked.
- [ ] **Layers** mode: no collisions reported against the part itself (it isn't built yet). Collisions with the plate, fixture and table are still reported.
- [ ] **Revolve / Surface**: part collisions ARE reported.
- [ ] A gun angle steeper than 70° below horizontal shows as a **warning** row, not a failure.
- [ ] **Solve problems**: the bottom bar shows "Solver: trying …" for each fix in turn, then a summary. Cancel mid-solve keeps the old path.
- [ ] Solve shows **"Suggested gantry E1 = … mm"** (and a purge point, if purge moves had problems). **Apply suggested set-up** fills the E1 / purge boxes; Generate again and the problems are gone. Note how long Solve takes on WearPlate2.
- [ ] If Solve changes E1: the gantry carriage moves in the 3D view and nothing collides with it.

## 5. Export (5 min)
- [ ] Save a KRL program, once with the split option and once with the pre-flight checkbox. Open the `.src`: the header shows BASE/TOOL, E1 changes come only on gun-off PTPs, and there are no NaN or odd joint values.
- [ ] Re-export the same path: the file is identical (diff it).

## 6. Session 9 and 10 additions (10 min)
- [ ] **Job**: set up a part fully (placement, constraints, KRL base, Surface faces, settings, E1, purge), Job → Save job…; close the app, Job → Open job (or Recent): every step comes back; Generate gives the same KRL.
- [ ] **Undo** in Position: move the part, add a constraint, Ctrl+Z twice, Ctrl+Shift+Z: placement and rows follow. ↶ ↷ in the step bar too.
- [ ] **Surface over a fillet** (flat face running into a blend): passes evenly spaced on the curve, not bunched.
- [ ] **Thickness** toggle in Simulate (Revolve / Surface): coloured map with a colour bar; min/mean/max and % thin look sensible. Note how long it takes on WearPlate2.
- [ ] **Build-up** toggle in Simulate: Layers: during playback finished layers are solid (copper) and the layer being sprayed appears track by track (two alternating shades); path lines hide; scrub back too. Revolve / Surface: the thickness map fills in as the gun passes (grey = uncoated). Playback stays smooth.
- [ ] **Estimates**: cycle time and powder line in Simulate and above Save in Export.
- [ ] **HOME legs**: problems on HOME → first / last → HOME show in Simulate and Solve; with move 0 a LIN, the leg is a straight line.
- [ ] **A4/A6 wind-up** warning appears on a path that winds the wrist.
- [ ] **KRL base methods** (Part step): 3 planes on a block corner (top, front, side): the faces turn purple in turn, the hint names the next one, origin at the corner, X along the top/front edge into the part; two parallel faces give a status message and start again.
- [ ] **KRL base vertex**: hover near corners: a small sphere and faded arrows follow, the view stays smooth; click places it. Spin ±90° and Flip Z update the arrows and label live; save and reopen the job: the turned base comes back, same KRL.
- [ ] **Export** (Advanced): untick "Program switches spray": the `$OUT` lines are comments. Tick "Split long programs" on a long path: main + `_Pnn` files next to it.

## After this, on the controller (ROADMAP order step 2)
- Copy the split program over and check that all parts load.
- Check that the pre-flight HALT triggers with the wrong TOOL/BASE.
- Then follow README → "Before running on the real robot", in T1, spray off.
