"""Cold spray toolpath window: parameters on the left, 3D cell view on the right.

Run with:  python -m coldspray.gui
"""

import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from pydantic import ValidationError
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton, QScrollArea, QSlider, QSpinBox,
    QVBoxLayout, QWidget,
)
from pyvistaqt import QtInteractor

from coldspray.cad.part import (
    Substrate, centre_on_faceplate, constrain, drop_on_plate, placed_vertices, snap_face_down,
)
from coldspray.cell import robroot, table_axis_names
from coldspray.kinematics.robot import Robot
from coldspray.models import PROJECT_ROOT, ProcessConfig, load_process, save_process
from coldspray.pipeline import MODES, check_placement, load_part, plan_part
from coldspray.post.krl import KRLError, to_krl
from coldspray.sim.cell_model import OPTIONAL
from coldspray.sim.scene import gantry_offset, load_assets, make_gun
from coldspray.sim.viewer import Scene
from coldspray.transforms import frame, xyzabc

UNITS = {"_mm_s": " mm/s", "_mm": " mm", "_deg": " °", "_s": " s"}
CHECK_EVERY = 3        # collision-check every 3rd sampled point (~6 mm of travel)


def _label(field: str) -> tuple[str, str]:
    for suffix, unit in UNITS.items():
        if field.endswith(suffix):
            return field[: -len(suffix)].replace("_", " ").capitalize(), unit
    return field.replace("_", " ").capitalize(), ""


def estimate_time_s(path, proc: ProcessConfig) -> float:
    """Rough program time: robot moves at their speed over the part, plus spray settling.
    Pure table rotations count at their speed over the part too."""
    total, on = 0.0, False
    for a, b in zip(path.moves, path.moves[1:]):
        total += np.linalg.norm(b.pose[:3, 3] - a.pose[:3, 3]) / b.speed
        if b.gun_on and not on:
            total += proc.spray_settle_s
        on = b.gun_on
    return total


def _spin(lo, hi, value, suffix, decimals=1, tip=""):
    box = QDoubleSpinBox()
    box.setDecimals(decimals)
    box.setRange(lo, hi)
    box.setValue(value)
    box.setSuffix(suffix)
    box.setToolTip(tip)
    return box


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cold Spray KUKA Control")
        self.resize(1600, 950)
        self.assets = None
        self.scene = None
        self.part = None        # coldspray.cad.part.Part
        self.substrate = Substrate()
        self.picked_face = None # flat face picked, not yet snapped to the plate
        self.collisions = []
        self.result = None      # (path, proc, cell, robot)
        self.play_timer = QTimer(self, interval=30, timeout=self._tick)

        # Collision checking runs on one worker thread (FCL objects stay on it)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.checker = None
        self.checker_e1 = None
        self.check_future = None
        self.check_id = 0
        self.poll_timer = QTimer(self, interval=200, timeout=self._poll_check)

        self.plotter = QtInteractor(self)
        panel = self._build_panel()
        view = QWidget()
        vbox = QVBoxLayout(view)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.addWidget(self.plotter.interactor, 1)
        vbox.addLayout(self._build_playback())

        root = QWidget()
        hbox = QHBoxLayout(root)
        hbox.addWidget(panel)
        hbox.addWidget(view, 1)
        self.setCentralWidget(root)
        self.statusBar().showMessage("Starting...")
        QTimer.singleShot(50, self._startup)

    # --- layout ---------------------------------------------------------------
    def _build_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)

        part_box = QGroupBox("Part")
        pl = QVBoxLayout(part_box)
        self.part_label = QLabel("test block (built in)")
        self.part_label.setWordWrap(True)
        open_btn = QPushButton("Open STEP file...")
        open_btn.clicked.connect(self._open_step)
        row = QHBoxLayout()
        for text, key in (("Test block", "test_block"), ("Test tube", "test_tube")):
            btn = QPushButton(text)
            btn.clicked.connect(lambda _=False, k=key: self._set_part(k))
            row.addWidget(btn)
        pl.addWidget(self.part_label)
        pl.addWidget(open_btn)
        pl.addLayout(row)
        layout.addWidget(part_box)

        place_box = QGroupBox("Placement on the faceplate")
        vbox = QVBoxLayout(place_box)
        grid = QFormLayout()
        self.place = {}
        for key, lo, hi, unit, tip in (
                ("X", -500, 500, " mm", "Along the faceplate X axis"),
                ("Y", -500, 500, " mm", "Along the faceplate Y axis"),
                ("Z", -500, 1000, " mm", "Height above the faceplate (locked once a face is on the plate)"),
                ("A", -180, 180, " °", "Turn about the faceplate Z axis"),
                ("B", -90, 90, " °", "Turn about Y (locked once a face is on the plate)"),
                ("C", -180, 180, " °", "Turn about X (locked once a face is on the plate)")):
            box = _spin(lo, hi, 0, unit, tip=tip)
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self._placement_typed)
            grid.addRow(key, box)
            self.place[key] = box
        vbox.addLayout(grid)
        self.pick_btn = QPushButton("Pick build face")
        self.pick_btn.setCheckable(True)
        self.pick_btn.setToolTip("Click a flat face of the part: the face the build starts from")
        self.pick_btn.toggled.connect(self._pick_toggled)
        self.snap_btn = QPushButton("Snap face to plate")
        self.snap_btn.setToolTip("Turn the part so the picked face lies on the substrate plate")
        self.snap_btn.clicked.connect(self._snap_face)
        self.snap_btn.setEnabled(False)
        centre_btn = QPushButton("Centre on faceplate")
        centre_btn.setToolTip("Move the part onto the table axis (needed for Revolve)")
        centre_btn.clicked.connect(lambda: self.part and self._set_placement(centre_on_faceplate(self.part)))
        reset_btn = QPushButton("Reset")
        reset_btn.setToolTip("Back to the CAD orientation, lying on the plate, centred; clears the build face")
        reset_btn.clicked.connect(self._reset_placement)
        for pair in ((self.pick_btn, self.snap_btn), (centre_btn, reset_btn)):
            row = QHBoxLayout()
            for btn in pair:
                row.addWidget(btn)
            vbox.addLayout(row)
        self.face_label = QLabel("Build face: none")
        self.face_label.setWordWrap(True)
        vbox.addWidget(self.face_label)
        layout.addWidget(place_box)

        sub_box = QGroupBox("Substrate plate")
        sform = QFormLayout(sub_box)
        s = self.substrate
        self.sub_fields = {
            "length": _spin(10, 1000, s.length, " mm", tip="Along the faceplate X axis"),
            "width": _spin(10, 1000, s.width, " mm", tip="Along the faceplate Y axis"),
            "thickness": _spin(1, 200, s.thickness, " mm", tip="The part is built on its top surface"),
        }
        for key, box in self.sub_fields.items():
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self._substrate_changed)
            sform.addRow(key.capitalize(), box)
        layout.addWidget(sub_box)

        plan_box = QGroupBox("Strategy")
        pform = QFormLayout(plan_box)
        self.mode = QComboBox()
        for key, text in MODES.items():
            self.mode.addItem(text, key)
        self.mode.setToolTip("Layers: slice the part and build it up layer by layer.\n"
                             "Revolve: spin the turntable and coat the outside of a round part.")
        self.mode.currentIndexChanged.connect(self._mode_changed)
        self.coats = QSpinBox()
        self.coats.setRange(1, 500)
        self.coats.setToolTip("Revolve: number of complete coats; each backs off by the layer thickness")
        pform.addRow("Mode", self.mode)
        pform.addRow("Coats", self.coats)
        layout.addWidget(plan_box)

        proc_box = QGroupBox("Process")
        form = QFormLayout(proc_box)
        self.fields = {}
        proc = load_process()
        for name, info in ProcessConfig.model_fields.items():
            text, unit = _label(name)
            if info.annotation is int:
                box = QSpinBox()
                box.setRange(0, 1000)
            else:
                box = QDoubleSpinBox()
                box.setDecimals(2)
                box.setRange(-360.0 if name.endswith("_deg") else 0.0, 100000.0)
            box.setValue(getattr(proc, name))
            box.setSuffix(unit)
            box.setToolTip(info.description)
            form.addRow(text, box)
            self.fields[name] = box
        layout.addWidget(proc_box)

        cell_box = QGroupBox("Gun and cell")
        cform = QFormLayout(cell_box)
        self.nozzle = QComboBox()
        self.nozzle.setToolTip("Nozzle fitted to the gun (sets the TCP)")
        self.e1 = _spin(0, 1, 0, " mm", tip="Gantry position for this program (+E1 moves towards the turntable)")
        self.tilt = _spin(0, 1, 0, " °", tip="Turntable tilt for this program (90° = faceplate facing the tailstock)")
        self.rotation = _spin(0, 1, 0, " °", tip="Turntable rotation for Layers mode (Revolve turns the table itself)")
        cform.addRow("Nozzle", self.nozzle)
        cform.addRow("Gantry E1", self.e1)
        cform.addRow("Table tilt", self.tilt)
        cform.addRow("Table rotation", self.rotation)
        for box in (self.e1, self.tilt, self.rotation):
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self._enter_setup)
        self.nozzle.currentIndexChanged.connect(self._enter_setup)
        self.check_box = QCheckBox("Check collisions")
        self.check_box.setChecked(True)
        cform.addRow(self.check_box)
        layout.addWidget(cell_box)

        show_box = QGroupBox("Show")
        sbox = QVBoxLayout(show_box)
        self.group_checks = {}
        for label, (_, default, _) in OPTIONAL.items():
            check = QCheckBox(label)
            check.setChecked(default)
            check.toggled.connect(lambda on, lbl=label: self._show_group(lbl, on))
            sbox.addWidget(check)
            self.group_checks[label] = check
        layout.addWidget(show_box)

        self.generate_btn = QPushButton("Generate toolpath")
        self.generate_btn.setStyleSheet("font-weight: bold; padding: 8px;")
        self.generate_btn.clicked.connect(self.generate)
        self.save_krl_btn = QPushButton("Save KRL program...")
        self.save_krl_btn.clicked.connect(self._save_krl)
        self.save_krl_btn.setEnabled(False)
        save_settings = QPushButton("Save process settings")
        save_settings.setToolTip("Write these process values to config/process.yaml")
        save_settings.clicked.connect(self._save_settings)
        for w in (self.generate_btn, self.save_krl_btn, save_settings):
            layout.addWidget(w)

        self.info = QLabel("")
        self.info.setWordWrap(True)
        layout.addWidget(self.info)
        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidget(inner)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(380)
        self._mode_changed()
        return scroll

    def _build_playback(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.play_btn = QPushButton("Play")
        self.play_btn.clicked.connect(self._toggle_play)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self._on_slider)
        row.addWidget(self.play_btn)
        row.addWidget(self.slider, 1)
        return row

    def _mode_changed(self):
        revolve = self.mode.currentData() == "revolve"
        self.coats.setEnabled(revolve)
        self.rotation.setEnabled(not revolve)
        self._show_placement()
        self._enter_setup()

    def _show_group(self, label, on):
        if self.scene:
            self.scene.set_group_visible(label, on)
            self.plotter.render()

    # --- startup --------------------------------------------------------------
    def _status(self, msg):
        self.statusBar().showMessage(msg)
        QApplication.processEvents()

    def _startup(self):
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.assets = a = load_assets(log=self._status)
            if a.tool.gun:
                for key, n in a.tool.gun.nozzles.items():
                    self.nozzle.addItem(f"{n.label} ({n.length:.0f} mm)", key)
                self.nozzle.setCurrentIndex(self.nozzle.findData(a.tool.gun.nozzle))
            else:
                self.nozzle.setEnabled(False)
            if a.cell.gantry:
                self.e1.setRange(*a.cell.gantry.limits_mm)
                self.e1.setValue(a.cell.gantry.position_mm)
            else:
                self.e1.setEnabled(False)
            tt = a.cell.turntable
            if tt:
                self.tilt.setRange(*tt.tilt.limits_deg)
                self.tilt.setValue(tt.tilt.position_deg)
                self.rotation.setRange(*tt.rotation.limits_deg)
                self.rotation.setValue(tt.rotation.position_deg)
                self.tilt.setPrefix(f"{tt.tilt.axis}  ")
                self.rotation.setPrefix(f"{tt.rotation.axis}  ")
            else:
                self.tilt.setEnabled(False)
                self.rotation.setEnabled(False)
            self.scene = Scene(self.plotter, a.robot, robroot(a.cell), a.meshes, gantry_offset(a.cell),
                               a.cell.robot_cad_pose_deg, make_gun(a.tool, a.gun_mesh))
            self.scene.on_part_moved = self._part_dragged
            self.scene.set_substrate(self.substrate.mesh())
            self._set_part(None)
            self.scene.focus_on_work()
        finally:
            QApplication.restoreOverrideCursor()

    # --- actions --------------------------------------------------------------
    def _open_step(self):
        start = str(PROJECT_ROOT / "assets" / "cad")
        fn, _ = QFileDialog.getOpenFileName(self, "Open part", start, "STEP files (*.stp *.step);;All files (*)")
        if fn:
            self._set_part(fn)

    def _set_part(self, source):
        """`source`: a STEP path, a built-in part name, or None (test block)."""
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self._status("Reading part...")
            part = load_part(source)
            part.placement = drop_on_plate(part, self.substrate.top)
            self.part = part
            builtin = source is None or not Path(str(source)).suffix
            self.part_label.setText(f"{part.name} (built in)" if builtin else Path(source).name)
            if source == "test_tube":
                self.mode.setCurrentIndex(self.mode.findData("revolve"))
            elif builtin:
                self.mode.setCurrentIndex(self.mode.findData("layers"))
        except Exception as e:  # bad or unreadable STEP file
            QApplication.restoreOverrideCursor()
            QMessageBox.warning(self, "Could not load part", str(e))
            return
        finally:
            QApplication.restoreOverrideCursor()
        self.pick_btn.setChecked(False)
        self.picked_face = None
        self.snap_btn.setEnabled(False)
        self.scene.set_part(self.part)
        self.scene.set_constrained(False)
        self._enter_setup()
        self._show_placement()
        self.scene.focus_on_part()
        self.plotter.render()
        self._status(f"Loaded {self.part.name}: set it up, then Generate")

    # --- part setup -----------------------------------------------------------
    def _work_cell(self):
        """Cell config with the gantry at the chosen E1."""
        cell = self.assets.cell.model_copy(deep=True)
        if cell.gantry:
            cell.gantry.position_mm = self.e1.value()
        return cell

    def _table_axes(self, cell) -> dict:
        names = table_axis_names(cell)
        return dict(zip(names, (self.tilt.value(), self.rotation.value())))

    def _enter_setup(self, *_):
        """Show the part set-up for the current settings; any toolpath is out of date."""
        if self.scene is None:
            return
        self.play_timer.stop()
        self.play_btn.setText("Play")
        self.check_id += 1                 # a running collision check no longer applies
        self.poll_timer.stop()
        self.result, self.collisions = None, []
        self.save_krl_btn.setEnabled(False)
        cell = self._work_cell()
        self.scene.set_placement(robroot(cell), gantry_offset(cell))
        self.scene.enter_setup(cell, self._table_axes(cell), home=cell.home_deg)
        self.slider.blockSignals(True)
        self.slider.setRange(0, 0)
        self.slider.blockSignals(False)
        if self.part is not None:
            self.info.setText(f"<b>{self.part.name}</b><br>"
                              "<span style='color:#e37400'>Toolpath out of date: press Generate</span>")
        self.plotter.render()

    def _set_placement(self, placement, redraw=True):
        """New part placement (kept on the plate if a build face is set); refresh view and boxes."""
        if self.part is None:
            return
        self.part.placement = constrain(self.part, np.asarray(placement, float), self.substrate.top)
        if redraw:
            self.scene.set_part_placement()
        self._show_placement()
        self._enter_setup()

    def _show_placement(self, placement=None):
        """Fill the X..C boxes and the build-face label."""
        if self.part is None:
            return
        values = xyzabc(self.part.placement if placement is None else placement)
        locked = self.part.build_face is not None
        for (key, box), v in zip(self.place.items(), values):
            box.blockSignals(True)
            box.setValue(float(v))
            box.blockSignals(False)
            box.setEnabled(not (locked and key in "ZBC"))
        if self.part.build_face is not None:
            face = self.part.planar[self.part.build_face]
            text = f"Build face {self.part.build_face} ({face.area:,.0f} mm²) flat on the plate ✓"
        elif self.picked_face is not None:
            face = self.part.planar[self.picked_face]
            text = f"Picked face {self.picked_face} ({face.area:,.0f} mm²): press Snap face to plate"
        else:
            text = "Build face: none (pick a flat face)"
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

    def _pick_toggled(self, on):
        if self.scene is None:
            return
        if not on:
            self.scene.cancel_face_pick()
            self.plotter.render()
            return
        if self.scene.mode != "setup":
            self._enter_setup()
        if not self.scene.start_face_pick(self._face_picked):
            self.pick_btn.setChecked(False)
            return
        self._status("Click a flat face of the part (the build-start face)")
        self.plotter.render()

    def _face_picked(self, face_id):
        if face_id not in self.part.planar:
            self._status("Not a flat face: pick a flat face")
            self.scene.start_face_pick(self._face_picked)   # keep picking
            return
        self.pick_btn.blockSignals(True)
        self.pick_btn.setChecked(False)
        self.pick_btn.blockSignals(False)
        self.picked_face = face_id
        self.scene.highlight_face(face_id)
        self.snap_btn.setEnabled(True)
        self._show_placement()
        self._status(f"Picked face {face_id}")
        self.plotter.render()

    def _snap_face(self):
        if self.part is None or self.picked_face is None:
            return
        placement = snap_face_down(self.part, self.picked_face, self.substrate.top)
        self.part.build_face, self.picked_face = self.picked_face, None
        self.snap_btn.setEnabled(False)
        self.scene.set_constrained(True)
        self._set_placement(placement)

    def _reset_placement(self):
        if self.part is None:
            return
        self.pick_btn.setChecked(False)
        self.part.build_face = self.picked_face = None
        self.part.placement = np.eye(4)
        self.snap_btn.setEnabled(False)
        self.scene.highlight_face(None)
        self.scene.set_constrained(False)
        self._set_placement(drop_on_plate(self.part, self.substrate.top))

    def _substrate_changed(self, *_):
        old = self.substrate.top
        self.substrate = Substrate(**{k: box.value() for k, box in self.sub_fields.items()})
        if self.scene:
            self.scene.set_substrate(self.substrate.mesh())
        if self.part is not None:
            placement = self.part.placement.copy()
            placement[2, 3] += self.substrate.top - old     # the part stays on the plate
            self._set_placement(placement)
        else:
            self._enter_setup()

    def read_process(self) -> ProcessConfig:
        return ProcessConfig(**{name: box.value() for name, box in self.fields.items()})

    def generate(self):
        if self.scene is None or self.part is None:
            return
        try:
            proc = self.read_process()
        except ValidationError as e:
            QMessageBox.warning(self, "Invalid parameter", str(e))
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            t0 = time.time()
            a = self.assets
            tool = a.tool.model_copy(deep=True)
            if tool.gun:
                tool.gun.nozzle = self.nozzle.currentData()
            robot = Robot(a.robot.cfg, tool)
            gun = make_gun(tool, a.gun_mesh)
            self.scene.set_gun(robot, gun)

            cell = self._work_cell()
            self.scene.set_placement(robroot(cell), gantry_offset(cell))

            self.pick_btn.setChecked(False)
            name, mode = self.part.name, self.mode.currentData()
            self._status("Planning...")
            summary, path = plan_part(self.part, proc, mode=mode, tilt_deg=self.tilt.value(),
                                      rotation_deg=self.rotation.value(), coats=self.coats.value(), cell=cell,
                                      substrate=self.substrate)
            if not path.moves:
                raise ValueError("No toolpath: the part gave no layers (is it thinner than one layer?)")
            self._status("Checking reach...")
            part_mesh = (placed_vertices(self.part), self.part.faces)
            substrate_mesh = self.substrate.mesh()
            joints = self.scene.set_path(path, cell, part=part_mesh, substrate=substrate_mesh, step=2.0,
                                         home=cell.home_deg, angle_step=5.0)
            self.result = (path, proc, cell, robot)
        except Exception as e:
            QApplication.restoreOverrideCursor()
            QMessageBox.warning(self, "Could not generate toolpath", str(e))
            self._status("Generate failed")
            return
        QApplication.restoreOverrideCursor()

        self.slider.blockSignals(True)
        self.slider.setRange(0, self.scene.n_points - 1)
        self.slider.setValue(self.scene.first_reachable())
        self.slider.blockSignals(False)
        self.unreachable = int(np.isnan(joints[:, 0]).sum())
        self.collision_text = "collision check off" if not self.check_box.isChecked() else "checking collisions..."
        self.summary_head = (f"<b>{name}</b><br>{summary}, {len(path.moves)} moves<br>"
                             f"Estimated time: {estimate_time_s(path, proc) / 60:.1f} min<br>")
        self._update_info()
        self.save_krl_btn.setEnabled(self.unreachable == 0)
        self.plotter.render()
        self._status(f"Generated in {time.time() - t0:.1f} s")
        if self.check_box.isChecked() and a.meshes is not None:
            self._start_collision_check(robot, gun, cell, part_mesh, substrate_mesh, joints)

    def _update_info(self):
        reach = (f"<span style='color:#d93025'>{self.unreachable} unreachable points</span>" if self.unreachable
                 else "<span style='color:green'>All points reachable</span>")
        self.info.setText(self.summary_head + reach + "<br>" + self.collision_text)

    # --- collision checking (background) -------------------------------------------
    def _start_collision_check(self, robot, gun, cell, part, substrate, joints):
        from coldspray.sim.collision import CollisionChecker

        self.check_id += 1
        job, a = self.check_id, self.assets
        axes_at = self.scene.samples.axes_at
        e1 = cell.gantry.position_mm if cell.gantry else None

        def work():
            if job != self.check_id:
                return None                       # superseded by a newer Generate
            if self.checker is None or self.checker_e1 != e1:
                self.checker = CollisionChecker(robot, cell, a.meshes, gun, part, cell.robot_cad_pose_deg,
                                                substrate=substrate)
                self.checker_e1 = e1
            else:
                self.checker.set_gun(robot, gun)
                self.checker.set_workpiece(part, substrate)
                self.checker.cell = cell
            return job, self.checker.check_path(joints, axes_at, every=CHECK_EVERY)

        self.check_future = self.pool.submit(work)
        self.poll_timer.start()

    def _poll_check(self):
        if self.check_future is None or not self.check_future.done():
            return
        self.poll_timer.stop()
        future, self.check_future = self.check_future, None
        try:
            out = future.result()
        except Exception as e:
            self.collision_text = f"<span style='color:#d93025'>collision check failed: {e}</span>"
            self._update_info()
            return
        if out is None or out[0] != self.check_id:
            return
        hits = out[1]
        self.scene.set_collisions(hits)
        self.plotter.render()
        if hits:
            pairs = sorted({p for h in hits for p in h.pairs})
            shown = ", ".join(f"{a}/{b}" for a, b in pairs[:4]) + (" ..." if len(pairs) > 4 else "")
            self.collision_text = (f"<span style='color:#c2185b'>{len(hits)} collision points</span> "
                                   f"(magenta): {shown}")
        else:
            self.collision_text = "<span style='color:green'>No collisions found</span>"
        self.collisions = hits
        self._update_info()

    def _save_krl(self):
        path, proc, cell, robot = self.result
        hits = self.collisions
        if hits and QMessageBox.question(
                self, "Collisions found",
                f"The simulation found {len(hits)} collision points. Save the program anyway?") \
                != QMessageBox.StandardButton.Yes:
            return
        out = PROJECT_ROOT / "output"
        out.mkdir(exist_ok=True)
        fn, _ = QFileDialog.getSaveFileName(self, "Save KRL program", str(out / f"{path.name}.src"),
                                            "KRL source (*.src)")
        if not fn:
            return
        try:
            Path(fn).write_text(to_krl(path, robot, cell, proc), newline="\r\n")
        except (KRLError, ValueError) as e:
            QMessageBox.warning(self, "Could not write program", str(e))
            return
        self._status(f"Saved {fn}")

    def _save_settings(self):
        try:
            proc = self.read_process()
        except ValidationError as e:
            QMessageBox.warning(self, "Invalid parameter", str(e))
            return
        save_process(proc)
        self._status("Saved process settings to config/process.yaml")

    # --- playback -------------------------------------------------------------
    def _on_slider(self, i):
        if self.scene and self.scene.mode == "preview":
            self.scene.show_point(i)
            self.plotter.render()

    def _toggle_play(self):
        if self.play_timer.isActive():
            self.play_timer.stop()
            self.play_btn.setText("Play")
        elif self.scene and self.scene.n_points:
            self.play_timer.start()
            self.play_btn.setText("Pause")

    def _tick(self):
        i = self.slider.value() + 1
        if i > self.slider.maximum():
            self._toggle_play()
            return
        self.slider.setValue(i)

    def closeEvent(self, event):
        self.play_timer.stop()
        self.poll_timer.stop()
        self.check_id += 1            # let a running check finish without effect
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.plotter.close()
        super().closeEvent(event)


def main():
    app = QApplication.instance() or QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
