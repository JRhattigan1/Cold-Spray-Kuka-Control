"""Step 2 Position: the part on the faceplate, substrate plate, table and gantry, auto position."""

import numpy as np
from scipy.spatial.transform import Rotation
from pydantic import ValidationError
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFormLayout, QGridLayout, QMessageBox, QPushButton, QVBoxLayout,
    QWidget,
)

from coldspray.cad.part import (
    Substrate, build_frame, centre_on_faceplate, constrain, drop_on_plate, lower_to_floor, placed_vertices,
    snap_face_down,
)
from coldspray.cell import robroot
from coldspray.gui.constraints_panel import ConstraintsPanel
from coldspray.gui.stepper import TOOLPATH
from coldspray.gui.widgets import Section, primary_button, spin, wrap_label
from coldspray.pipeline import check_placement, plan_part
from coldspray.planner.position import auto_position, layer_tilts
from coldspray.sim.cell_model import OPTIONAL
from coldspray.sim.markers import DIRECTION_PRESETS, marker_pose, preset_for
from coldspray.sim.viewer import camera_axes
from coldspray.transforms import frame, xyzabc


class PositionStep:
    def _build_position_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)

        self.cell_part_label = wrap_label()
        layout.addWidget(self.cell_part_label)

        place_box = self.placement_section = Section("Placement on the faceplate")
        grid = QFormLayout()
        self.place = {}
        for key, lo, hi, unit, tip in (
                ("X", -500, 500, " mm", "Along the faceplate X axis"),
                ("Y", -500, 500, " mm", "Along the faceplate Y axis"),
                ("Z", -500, 1000, " mm", "Height above the faceplate (locked while Keep on plate is ticked)"),
                ("A", -180, 180, " °", "Turn about the faceplate Z axis"),
                ("B", -90, 90, " °", "Turn about Y (locked while Keep on plate is ticked)"),
                ("C", -180, 180, " °", "Turn about X (locked while Keep on plate is ticked)")):
            box = spin(lo, hi, 0, unit, tip=tip)
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self._placement_typed)
            grid.addRow(key, box)
            self.place[key] = box
        place_box.add(grid)
        self.keep_on_plate = QCheckBox("Keep on plate")
        self.keep_on_plate.setToolTip("Stand the part on its base face on the plate (or the faceplate)\n"
                                      "and lock Z, B and C. Untick to place it anywhere in 3D\n"
                                      "(held in a fixture or on a riser).")
        self.keep_on_plate.setEnabled(False)
        self.keep_on_plate.toggled.connect(self._keep_toggled)
        place_box.add(self.keep_on_plate)
        snap_btn = self.snap_btn = QPushButton("Snap base face down")
        snap_btn.setToolTip("Once: turn the part so its base face points down, lowest point on the floor")
        snap_btn.clicked.connect(lambda: self.part and self.part.base_face is not None and self._set_placement(
            snap_face_down(self.part, self.part.base_face, self._floor())))
        drop_btn = self.drop_btn = QPushButton("Drop onto plate")
        drop_btn.setToolTip("Once: move the part straight down (or up) so its lowest point is on the floor")
        drop_btn.clicked.connect(lambda: self.part and self._set_placement(lower_to_floor(self.part, self._floor())))
        centre_btn = self.centre_btn = QPushButton("Centre on faceplate")
        centre_btn.setToolTip("Move the part onto the table axis (needed for Revolve)")
        centre_btn.clicked.connect(lambda: self.part and self._set_placement(centre_on_faceplate(self.part)))
        reset_btn = self.reset_btn = QPushButton("Reset")
        reset_btn.setToolTip("Back to the start: standing on the base face (or as in the CAD), centred")
        reset_btn.clicked.connect(self._reset_placement)
        buttons = QGridLayout()
        for k, btn in enumerate((snap_btn, drop_btn, centre_btn, reset_btn)):
            buttons.addWidget(btn, k // 2, k % 2)
        place_box.add(buttons)
        self.face_label = wrap_label()
        place_box.add(self.face_label)
        self.sm_part = QCheckBox("SpaceMouse moves the part")
        self.sm_part.setToolTip("Off: the SpaceMouse flies the camera. On (setup only): it moves the part,\n"
                                "kept standing on its base face while Keep on plate is ticked.\n"
                                "Left SpaceMouse button toggles this; right button shows the part.")
        self.sm_part.setVisible(False)
        place_box.add(self.sm_part)
        layout.addWidget(place_box)
        self.constraints = ConstraintsPanel()
        self.constraints.changed.connect(lambda: self.part is not None and self._set_placement(self.part.placement))
        layout.addWidget(self.constraints)

        sub_box = self.sub_section = Section("Substrate plate")
        self.sub_check = QCheckBox("Build on a substrate plate")
        self.sub_check.setChecked(True)
        self.sub_check.setToolTip("Untick for no plate: the part (or the component being repaired)\n"
                                  "stands directly on the faceplate")
        self.sub_check.toggled.connect(self._substrate_changed)
        sub_box.add(self.sub_check)
        sform = QFormLayout()
        s = self.substrate
        self.sub_fields = {
            "length": spin(10, 1000, s.length, " mm", tip="Along the faceplate X axis"),
            "width": spin(10, 1000, s.width, " mm", tip="Along the faceplate Y axis"),
            "thickness": spin(1, 200, s.thickness, " mm", tip="The part is built on its top surface"),
        }
        for key, box in self.sub_fields.items():
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self._substrate_changed)
            sform.addRow(key.capitalize(), box)
        sub_box.add(sform)
        layout.addWidget(sub_box)

        cell_box = self.table_section = Section("Table and gantry")
        cform = QFormLayout()
        self.nozzle = QComboBox()
        self.nozzle.setToolTip("Nozzle fitted to the gun (sets the TCP)")
        self.tilt = spin(0, 1, 0, " °", tip="Turntable tilt for this program (90° = faceplate facing the tailstock)")
        self.rotation = spin(0, 1, 0, " °", tip="Turntable rotation for Layers mode (Revolve turns the table itself)")
        self.e1 = spin(0, 1, 0, " mm", tip="Gantry position for this program (+E1 moves towards the turntable)")
        self.roll = spin(-180, 180, 0, " °", tip="Gun turned about its spray axis (changes how the wrist reaches;\n"
                                                 "the spray itself is unchanged)")
        cform.addRow("Nozzle", self.nozzle)
        cform.addRow("Table tilt", self.tilt)
        cform.addRow("Table rotation", self.rotation)
        cform.addRow("Gantry E1", self.e1)
        cform.addRow("Gun roll", self.roll)
        for box in (self.e1, self.tilt, self.rotation, self.roll):
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self._enter_setup)
        self.nozzle.currentIndexChanged.connect(self._enter_setup)
        cell_box.add(cform)
        self.auto_btn = QPushButton("Auto position (gun level)")
        self.auto_btn.setToolTip("Choose table tilt, gun roll and gantry E1 so the gun is as level as allowed\n"
                                 "and the whole path is reachable (also done when a part is loaded)")
        self.auto_btn.clicked.connect(self._auto_position)
        cell_box.add(self.auto_btn)
        layout.addWidget(cell_box)
        layout.addWidget(self._build_purge_section())

        show_box = self.show_section = Section("Show")
        self.group_checks = {}
        for label, (_, default, _) in OPTIONAL.items():
            check = QCheckBox(label)
            check.setChecked(default)
            check.toggled.connect(lambda on, lbl=label: self._show_group(lbl, on))
            show_box.add(check)
            self.group_checks[label] = check
        layout.addWidget(show_box)

        layout.addWidget(primary_button("Next: Toolpath  →", lambda: self.go_to(TOOLPATH)))
        layout.addStretch(1)
        return inner

    def _show_group(self, label, on):
        if self.scene:
            self.scene.set_group_visible(label, on)
            self.plotter.render()

    # --- into the cell ---------------------------------------------------------
    def _enter_cell(self):
        """Put the part into the cell as chosen in the Part step. Kept as it is
        (with the user's placement) if those choices haven't changed."""
        src = self.source
        if src is None or self.scene is None:
            return
        key = self._part_key()
        if key == self.cell_key and self.part is not None:
            return
        ignored = [b for b, r in enumerate(self.roles) if r == "ignore"]
        self.part = src.without(ignored)
        self.part.placement = self._start_placement(self.part)
        self.cell_key = key
        self.constraints.set_part(self.part, self.substrate)
        has_base = self.part.base_face is not None
        self.keep_on_plate.blockSignals(True)
        self.keep_on_plate.setChecked(has_base)
        self.keep_on_plate.setEnabled(has_base)
        self.keep_on_plate.blockSignals(False)
        self.snap_btn.setEnabled(has_base)
        self.scene.set_part(self.part)
        self.scene.set_constrained(self._kept())
        self.scene.highlight_face(self.part.start_face)
        self.cell_part_label.setText(f"<b>{src.name}</b>: {self.part.n_bodies} bodies in the cell"
                                     if self.part.n_bodies > 1 else f"<b>{src.name}</b>")
        self._enter_setup()
        self._show_placement()
        found = self._auto_position(quiet=True)
        self.scene.focus_on_part()
        self.plotter.render()
        if found:
            self._status(f"{src.name} in the cell: adjust the placement if needed, then Next: Toolpath")

    def _start_placement(self, part):
        """Standing on the base face if one is set, else as in the CAD; centred on the table."""
        part.placement = np.eye(4)
        if part.base_face is not None:
            part.placement = snap_face_down(part, part.base_face, self._floor())
        return drop_on_plate(part, self._floor())

    def _auto_position(self, *_, quiet=False) -> bool:
        """Set table tilt, gun roll and E1 for a level gun and a reachable path.
        `quiet`: report problems in the status bar only. False if nothing fully reachable was found."""
        if self.scene is None or self.part is None:
            return False
        try:
            proc = self.read_process()
        except ValidationError as e:
            QMessageBox.warning(self, "Invalid parameter", str(e))
            return False
        a, mode = self.assets, self.mode.currentData()
        cell = self._work_cell()
        if mode == "layers":       # spray against the build direction
            gun_dir = -build_frame(self.part)[:3, 2]
            tilts = layer_tilts(cell, gun_dir, self.rotation.value())
        else:
            tilts = [self.tilt.value()]

        def plan(tilt):
            return plan_part(self.part, proc, mode=mode, tilt_deg=tilt, rotation_deg=self.rotation.value(),
                             coats=self.coats.value(), cell=cell, substrate=self.substrate)[1]

        def progress(f):                     # runs here on the GUI thread: keep Cancel clickable
            self.job.update(f)
            QApplication.processEvents()

        self.job.start("Auto position...")
        try:
            pos = auto_position(plan, a.robot, cell, tilts, e1_now=self.e1.value(), log=self._status,
                                progress=progress, cancel=self.job.is_cancelled)
        except ValueError as e:              # placement not plannable yet
            self.job.finish()
            self._status(f"Auto position skipped: {e}")
            return False
        cancelled = self.job.is_cancelled()
        self.job.finish("Auto position cancelled" if cancelled else "")
        if pos.reach < 0:                    # cancelled before anything was tried
            return False
        for box, value in ((self.tilt, pos.tilt_deg), (self.roll, pos.roll_deg), (self.e1, pos.e1_mm)):
            if value is not None:
                box.blockSignals(True)
                box.setValue(value)
                box.blockSignals(False)
        self._enter_setup()
        msg = (f"Auto position: tilt {pos.tilt_deg:.0f}°, gun roll {pos.roll_deg:.0f}°"
               + (f", E1 {pos.e1_mm:.0f} mm" if pos.e1_mm is not None else ""))
        if pos.reach < 1.0:
            msg += f": best found reaches only {pos.reach:.0%} of the path"
            if not quiet:
                QMessageBox.warning(self, "Auto position", msg + "\nTry moving the part or another tilt.")
        self._status(msg)
        return pos.reach == 1.0

    # --- placement --------------------------------------------------------------
    def _floor(self) -> float:
        """Height the part stands on: plate top, or the faceplate."""
        return self.substrate.top if self.substrate else 0.0

    def _kept(self) -> bool:
        """Keep on plate: the part stays standing on its base face, Z/B/C locked."""
        return self.part is not None and self.part.base_face is not None and self.keep_on_plate.isChecked()

    def _keep_toggled(self, on):
        if self.part is None:
            return
        self.scene.set_constrained(self._kept())
        self._set_placement(self.part.placement)     # ticked: stand it on the base face now
        self.plotter.render()

    def _set_placement(self, placement, redraw=True):
        """New part placement (kept standing on its base face with Keep on plate); refresh view and boxes."""
        if self.part is None:
            return
        placement = np.asarray(placement, float)
        self.part.placement = constrain(self.part, placement, self._floor()) if self._kept() else placement
        self.part.placement = self.constraints.apply(self.part.placement)
        if redraw:
            self.scene.set_part_placement()
        self._show_placement()
        self._enter_setup()

    def _show_placement(self, placement=None):
        """Fill the X..C boxes and the base-face label."""
        if self.part is None:
            return
        values = xyzabc(self.part.placement if placement is None else placement)
        locked = self._kept()
        for (key, box), v in zip(self.place.items(), values):
            box.blockSignals(True)
            box.setValue(float(v))
            box.blockSignals(False)
            box.setEnabled(not (locked and key in "ZBC"))
        part, floor = self.part, "the plate" if self.substrate else "the faceplate"
        if locked:
            text = f"Keep on plate: standing on base face {part.base_face} (down on {floor}), Z, B and C locked"
        elif part.base_face is not None:
            text = f"Free placement (base face {part.base_face}): move and turn it anywhere"
        else:
            text = "No base face: free to move and turn"
        above = placed_vertices(part)[:, 2].min() - self._floor()
        if above > 0.1:
            text += f"<br>{above:.1f} mm above {floor}"
        text += (f"<br>Layers grow from start face {part.start_face} (green)" if part.start_face is not None
                 else "<br>Layers build up from the bottom of the build body")
        try:
            check_placement(placed_vertices(self.part), self.substrate, self.mode.currentData())
        except ValueError as e:
            text += f"<br><span style='color:#d93025'>{e}</span>"
        self.face_label.setText(text)

    def _placement_typed(self, *_):
        if self.part is not None:
            self._set_placement(frame(*(box.value() for box in self.place.values())))

    def _part_dragged(self, placement, final):
        if final:
            self._set_placement(placement)
        else:
            self._show_placement(placement)

    def _reset_placement(self):
        if self.part is not None:
            self._set_placement(self._start_placement(self.part))

    # --- substrate plate ---------------------------------------------------------
    def _substrate_ui(self):
        on = self.sub_check.isChecked()
        for box in self.sub_fields.values():
            box.setEnabled(on)
        self.sub_section.set_chip("" if on else "off")

    def _substrate_changed(self, *_):
        old = self._floor()
        self.substrate = (Substrate(**{k: box.value() for k, box in self.sub_fields.items()})
                          if self.sub_check.isChecked() else None)
        self._substrate_ui()
        self.constraints.substrate = self.substrate
        if self.scene:
            self.scene.set_substrate(self._substrate_mesh())
        if self.part is not None:
            placement = self.part.placement.copy()
            placement[2, 3] += self._floor() - old     # the part stays standing on the floor
            self._set_placement(placement)
        else:
            self._enter_setup()

    # --- SpaceMouse moving the part -----------------------------------------------
    def _spacemouse_part(self, t, r, dt, speed=120.0, turn_rate=60.0):
        """Move the part with the view: right / away (along the plate) / up, tilt and twist
        about the part centre. mm/s and deg/s at full deflection."""
        f = self.scene.setup_frame
        n = f[:3, 2]                          # faceplate normal
        right, fwd, _ = camera_axes(self.plotter)
        along = fwd - fwd @ n * n
        along = along / np.linalg.norm(along) if np.linalg.norm(along) > 1e-3 else np.cross(n, right)
        side = np.cross(along, n)
        move = (t[0] * side + t[1] * along + t[2] * n) * speed * dt
        spin_ = (-r[0] * side + r[1] * along + r[2] * n) * np.radians(turn_rate) * dt
        to_base = f[:3, :3].T
        rot = Rotation.from_rotvec(to_base @ spin_).as_matrix()
        v = placed_vertices(self.part)
        centre = (v.min(axis=0) + v.max(axis=0)) / 2
        step = np.eye(4)
        step[:3, :3] = rot
        step[:3, 3] = centre - rot @ centre + to_base @ move
        placement = step @ self.part.placement
        self.part.placement = constrain(self.part, placement, self._floor()) if self._kept() else placement
        self.part.placement = self.constraints.apply(self.part.placement)
        self.scene.set_part_placement()
        self._show_placement()
        self.sm_moved_part = True

    # --- warm-up / powder purge point (F2) ----------------------------------------
    # Settings live in `cell.purge` (PurgeConfig) when the model has that field, else
    # in the plain dict `self.purge_cfg`. The ghost gun shows the purge pose in WORLD.
    PURGE_DEFAULTS = dict(enabled=False, point=(-884.56, 400.0, 2000.0), spray_dir=(0.0, -1.0, 0.0),
                          dwell_s=5.0, at_start=True, at_end=True, every_n_layers=0, approach_mm=100.0)

    def _build_purge_section(self) -> Section:
        box = self.purge_section = Section("Warm-up / purge point")
        form = QFormLayout()
        self.purge_on = QCheckBox("Go to the purge point")
        self.purge_on.setToolTip("Spray on at a point in free space before (and after) the part")
        self.purge_xyz = [spin(-20000, 20000, 0, " mm", tip=f"Nozzle exit target, WORLD {a}") for a in "XYZ"]
        self.purge_dir = QComboBox()
        self.purge_dir.addItems([*DIRECTION_PRESETS, "Custom"])
        self.purge_dir.setToolTip("Spray direction at the purge point (WORLD)")
        self.purge_dwell = spin(0, 600, 5, " s", tip="Spray on at the point before moving off")
        self.purge_start = QCheckBox("At start")
        self.purge_end = QCheckBox("At end")
        self.purge_every = spin(0, 1000, 0, " layers", decimals=0, tip="Also go back every N layers (0: never)")
        form.addRow(self.purge_on)
        for axis, box_ in zip("XYZ", self.purge_xyz):
            form.addRow(axis, box_)
        form.addRow("Direction", self.purge_dir)
        self.purge_custom = [spin(-1, 1, 0, "", decimals=3, tip=f"Custom spray direction, WORLD {a}") for a in "XYZ"]
        for axis, box_ in zip("XYZ", self.purge_custom):
            form.addRow(f"  dir {axis}", box_)
        self.purge_save_btn = QPushButton("Save to cell.yaml")
        self.purge_save_btn.setToolTip("Write these settings into the purge: block of cell.yaml")
        self.purge_save_btn.clicked.connect(self._purge_save)
        form.addRow("Dwell", self.purge_dwell)
        form.addRow(self.purge_start)
        form.addRow(self.purge_end)
        form.addRow("Every", self.purge_every)
        form.addRow(self.purge_save_btn)
        self.purge_goto_btn = QPushButton("Move gun here")
        self.purge_goto_btn.setToolTip("Preview: put the robot at the purge pose (gantry at the current E1)")
        self.purge_goto_btn.clicked.connect(self._purge_goto)
        form.addRow(self.purge_goto_btn)
        box.add(form)
        self._purge_form = form
        self.purge_cfg = dict(self.PURGE_DEFAULTS)
        self._purge_loading = False
        self._purge_load()
        for w in (*self.purge_xyz, *self.purge_custom, self.purge_dwell, self.purge_every):
            w.setKeyboardTracking(False)
            w.valueChanged.connect(self._purge_changed)
        for w in (self.purge_on, self.purge_start, self.purge_end):
            w.toggled.connect(self._purge_changed)
        self.purge_dir.currentIndexChanged.connect(self._purge_changed)
        box.toggled.connect(lambda on: on and self._purge_load())
        if hasattr(self, "e1"):                 # reach depends on the gantry position
            self.e1.valueChanged.connect(lambda *_: self._purge_show())
        return box

    def _purge_cell(self):
        a = getattr(self, "assets", None)
        return a.cell if a is not None else None

    def _purge_load(self):
        """Fill the widgets from `cell.purge` if set (else the current settings)."""
        cell = self._purge_cell()
        cfg = getattr(cell, "purge", None) if cell is not None else None
        if cfg is not None:
            self.purge_cfg = cfg.model_dump() if hasattr(cfg, "model_dump") else dict(cfg)
        c = self.purge_cfg
        self._purge_loading = True
        self.purge_on.setChecked(c["enabled"])
        for box, v in zip(self.purge_xyz, c["point"]):
            box.setValue(v)
        self.purge_dir.setCurrentText(preset_for(c["spray_dir"]))
        for box, v in zip(self.purge_custom, c["spray_dir"]):
            box.setValue(v)
        self.purge_dwell.setValue(c["dwell_s"])
        self.purge_start.setChecked(c["at_start"])
        self.purge_end.setChecked(c["at_end"])
        self.purge_every.setValue(c["every_n_layers"])
        self._purge_loading = False
        self._purge_show()

    def _purge_changed(self, *_):
        if self._purge_loading:
            return
        c = self.purge_cfg
        name = self.purge_dir.currentText()
        c.update(enabled=self.purge_on.isChecked(), point=tuple(b.value() for b in self.purge_xyz),
                 spray_dir=DIRECTION_PRESETS.get(name) or self._purge_custom_dir(c["spray_dir"]), dwell_s=self.purge_dwell.value(),
                 at_start=self.purge_start.isChecked(), at_end=self.purge_end.isChecked(),
                 every_n_layers=int(self.purge_every.value()))
        cell = self._purge_cell()
        if cell is not None and "purge" in type(cell).model_fields:
            from coldspray.models import PurgeConfig
            cell.purge = PurgeConfig(**c)
        self._purge_show()

    def _purge_custom_dir(self, old):
        """Unit vector from the Custom X/Y/Z boxes (`old` if they are all zero)."""
        v = np.array([b.value() for b in self.purge_custom])
        n = np.linalg.norm(v)
        return tuple(float(x) for x in v / n) if n > 1e-9 else tuple(old)

    def _purge_save(self):
        from coldspray.config_io import save_purge
        from coldspray.models import PurgeConfig
        try:
            path = save_purge(PurgeConfig(**self.purge_cfg))
        except (OSError, ValidationError) as e:
            QMessageBox.warning(self, "Save purge point", f"Could not save: {e}")
            return
        self._status(f"Purge point saved to {path}")

    def _purge_ik(self, pose):
        """IK solutions for the gun at `pose` (WORLD), gantry at the current E1; None without assets."""
        a = getattr(self, "assets", None)
        if a is None:
            return None
        cell = self._work_cell()
        return a.robot.ik(np.linalg.inv(robroot(cell)) @ pose, cell.home_deg)

    def _purge_reachable(self, pose) -> bool | None:
        """Can the robot put the gun at `pose` (WORLD) with the gantry at the current E1?"""
        sols = self._purge_ik(pose)
        return None if sols is None else bool(sols)

    def _purge_goto(self):
        """Show the robot at the purge pose (first IK solution), or say why not."""
        c = self.purge_cfg
        sols = self._purge_ik(marker_pose(c["point"], c["spray_dir"]))
        if not sols:
            self._status("Purge point: no part/cell loaded" if sols is None
                         else "Purge point is out of reach at this gantry position")
            return
        if getattr(self, "scene", None) is not None:
            self.scene.set_robot_pose(sols[0])
        self._status("Robot shown at the purge point")

    def _purge_show(self):
        """Move the ghost gun and set the chip: off / reachable / out of reach."""
        c = self.purge_cfg
        custom = self.purge_dir.currentText() == "Custom"
        for box in self.purge_custom:
            self._purge_form.setRowVisible(box, custom)
        pose = marker_pose(c["point"], c["spray_dir"])
        ok = self._purge_reachable(pose) if c["enabled"] else None
        if not c["enabled"]:
            self.purge_section.set_chip("off")
        elif ok is None:
            self.purge_section.set_chip("")
        else:
            self.purge_section.set_chip("reachable" if ok else "out of reach", "good" if ok else "bad")
        if getattr(self, "scene", None) is not None:
            self.scene.set_purge_marker(pose, visible=c["enabled"], reachable=ok is not False)
