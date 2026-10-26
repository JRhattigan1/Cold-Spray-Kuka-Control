"""Cold spray toolpath window, in two tabs:

  1 Part - the part or assembly on its own: body roles (build / fixture /
           ignore), hide bodies to reach hidden faces, pick the start face the
           layers grow from and the base face it stands on.
  2 Cell - position it on the faceplate, gun and table settings, generate and
           simulate the toolpath, save the KRL program.

Run with:  python -m coldspray.gui
"""

import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from pydantic import ValidationError
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton, QScrollArea, QSlider, QSpinBox,
    QTabWidget, QVBoxLayout, QWidget,
)
from pyvistaqt import QtInteractor

from coldspray.cad.part import (
    Substrate, build_frame, build_mesh, centre_on_faceplate, constrain, drop_on_plate,
    lower_to_floor, other_mesh, placed_vertices, snap_face_down,
)
from coldspray.cell import robroot, table_axis_names
from coldspray.kinematics.robot import Robot
from coldspray.models import PROJECT_ROOT, ProcessConfig, load_process, save_process
from coldspray.pipeline import MODES, check_placement, load_part, plan_part
from coldspray.planner.position import auto_position, layer_tilts
from coldspray.post.krl import KRLError, to_krl
from coldspray.sim.cell_model import OPTIONAL
from coldspray.sim.scene import gantry_offset, load_assets, make_gun
from coldspray.sim.part_view import ROLES, PartView
from coldspray.sim.spacemouse import SpaceMouse
from coldspray.sim.viewer import Scene, camera_axes, fly
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
        self.source = None      # the part as loaded, with the choices made in the Part tab
        self.roles: list[str] = []       # per body: "build", "fixture" or "ignore"
        self.part = None        # working part in the cell: ignored bodies left out, placed
        self.cell_key = None    # Part tab choices the cell's part was made from
        self.substrate = Substrate()     # None: no plate, the part stands on the faceplate
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

        # SpaceMouse (if one is plugged in): flies the camera, or moves the part in setup
        self.spacemouse = None
        self.sm_timer = QTimer(self, interval=20, timeout=self._spacemouse_tick)
        self.sm_last = time.time()
        self.sm_moved_part = False

        self.part_plotter = QtInteractor(self)
        self.part_view = PartView(self.part_plotter)
        self.plotter = QtInteractor(self)
        view = QWidget()
        vbox = QVBoxLayout(view)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.addWidget(self.plotter.interactor, 1)
        vbox.addLayout(self._build_playback())

        self.tabs = QTabWidget()
        for title, panel, widget in (("1  Part", self._build_part_panel(), self.part_plotter.interactor),
                                     ("2  Cell", self._build_panel(), view)):
            page = QWidget()
            hbox = QHBoxLayout(page)
            hbox.addWidget(panel)
            hbox.addWidget(widget, 1)
            self.tabs.addTab(page, title)
        self.tabs.currentChanged.connect(self._tab_changed)
        self.setCentralWidget(self.tabs)
        self.statusBar().showMessage("Starting...")
        QTimer.singleShot(50, self._startup)

    # --- layout ---------------------------------------------------------------
    @staticmethod
    def _scroll(inner) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidget(inner)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(380)
        return scroll

    def _build_part_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)

        part_box = QGroupBox("Part")
        pl = QVBoxLayout(part_box)
        self.part_label = QLabel("test block (built in)")
        self.part_label.setWordWrap(True)
        open_btn = QPushButton("Open STEP file...")
        open_btn.clicked.connect(self._open_step)
        row = QHBoxLayout()
        for text, key in (("Test block", "test_block"), ("Test tube", "test_tube"),
                          ("Test repair", "test_repair")):
            btn = QPushButton(text)
            btn.clicked.connect(lambda _=False, k=key: self._set_part(k))
            row.addWidget(btn)
        pl.addWidget(self.part_label)
        pl.addWidget(open_btn)
        pl.addLayout(row)
        layout.addWidget(part_box)

        body_box = QGroupBox("Bodies")
        bl = QVBoxLayout(body_box)
        self.body_grid = QGridLayout()
        self.body_rows: list[tuple[QCheckBox, QComboBox]] = []
        bl.addLayout(self.body_grid)
        row = QHBoxLayout()
        isolate = QPushButton("Isolate build body")
        isolate.setToolTip("Hide the other bodies, e.g. to reach the underside of a repair patch")
        isolate.clicked.connect(lambda: self._show_bodies(only_build=True))
        show_all = QPushButton("Show all")
        show_all.clicked.connect(lambda: self._show_bodies(only_build=False))
        row.addWidget(isolate)
        row.addWidget(show_all)
        bl.addLayout(row)
        roles_help = QLabel("<small>Build: sliced and sprayed. Fixture: the component being repaired, "
                            "clamps - checked for collisions. Ignore: left out.</small>")
        roles_help.setWordWrap(True)
        bl.addWidget(roles_help)
        layout.addWidget(body_box)

        face_box = QGroupBox("Faces")
        fl = QVBoxLayout(face_box)
        self.start_btn = QPushButton("Pick start face")
        self.start_btn.setCheckable(True)
        self.start_btn.setToolTip("Click a flat face of the build body: the layers grow away from it,\n"
                                  "parallel to it (e.g. the underside of a repair patch)")
        self.start_btn.toggled.connect(lambda on: self._pick_face("start", on))
        self.start_label = QLabel()
        self.start_label.setWordWrap(True)
        self.base_btn = QPushButton("Pick base face (optional)")
        self.base_btn.setCheckable(True)
        self.base_btn.setToolTip("Click a flat face (any body) for the part to stand on: it is turned\n"
                                 "to face down onto the plate, or the faceplate without one")
        self.base_btn.toggled.connect(lambda on: self._pick_face("base", on))
        self.base_label = QLabel()
        self.base_label.setWordWrap(True)
        clear_btn = QPushButton("Clear faces")
        clear_btn.clicked.connect(self._clear_faces)
        for w in (self.start_btn, self.start_label, self.base_btn, self.base_label, clear_btn):
            fl.addWidget(w)
        layout.addWidget(face_box)

        next_btn = QPushButton("Next: position in the cell  →")
        next_btn.setStyleSheet("font-weight: bold; padding: 8px;")
        next_btn.clicked.connect(lambda: self.tabs.setCurrentIndex(1))
        layout.addWidget(next_btn)
        layout.addStretch(1)
        return self._scroll(inner)

    def _build_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)

        self.cell_part_label = QLabel()
        self.cell_part_label.setWordWrap(True)
        layout.addWidget(self.cell_part_label)

        place_box = QGroupBox("Placement on the faceplate")
        vbox = QVBoxLayout(place_box)
        grid = QFormLayout()
        self.place = {}
        for key, lo, hi, unit, tip in (
                ("X", -500, 500, " mm", "Along the faceplate X axis"),
                ("Y", -500, 500, " mm", "Along the faceplate Y axis"),
                ("Z", -500, 1000, " mm", "Height above the faceplate (locked while Keep on plate is ticked)"),
                ("A", -180, 180, " °", "Turn about the faceplate Z axis"),
                ("B", -90, 90, " °", "Turn about Y (locked while Keep on plate is ticked)"),
                ("C", -180, 180, " °", "Turn about X (locked while Keep on plate is ticked)")):
            box = _spin(lo, hi, 0, unit, tip=tip)
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self._placement_typed)
            grid.addRow(key, box)
            self.place[key] = box
        vbox.addLayout(grid)
        self.keep_on_plate = QCheckBox("Keep on plate")
        self.keep_on_plate.setToolTip("Stand the part on its base face on the plate (or the faceplate)\n"
                                      "and lock Z, B and C. Untick to place it anywhere in 3D\n"
                                      "(held in a fixture or on a riser).")
        self.keep_on_plate.setEnabled(False)
        self.keep_on_plate.toggled.connect(self._keep_toggled)
        vbox.addWidget(self.keep_on_plate)
        snap_btn = QPushButton("Snap base face down")
        snap_btn.setToolTip("Once: turn the part so its base face points down, lowest point on the floor")
        snap_btn.clicked.connect(lambda: self.part and self.part.base_face is not None and self._set_placement(
            snap_face_down(self.part, self.part.base_face, self._floor())))
        self.snap_btn = snap_btn
        drop_btn = QPushButton("Drop onto plate")
        drop_btn.setToolTip("Once: move the part straight down (or up) so its lowest point is on the floor")
        drop_btn.clicked.connect(lambda: self.part and self._set_placement(lower_to_floor(self.part, self._floor())))
        centre_btn = QPushButton("Centre on faceplate")
        centre_btn.setToolTip("Move the part onto the table axis (needed for Revolve)")
        centre_btn.clicked.connect(lambda: self.part and self._set_placement(centre_on_faceplate(self.part)))
        reset_btn = QPushButton("Reset")
        reset_btn.setToolTip("Back to the start: standing on the base face (or as in the CAD), centred")
        reset_btn.clicked.connect(self._reset_placement)
        buttons = QGridLayout()
        for k, btn in enumerate((snap_btn, drop_btn, centre_btn, reset_btn)):
            buttons.addWidget(btn, k // 2, k % 2)
        vbox.addLayout(buttons)
        self.face_label = QLabel()
        self.face_label.setWordWrap(True)
        vbox.addWidget(self.face_label)
        self.sm_part = QCheckBox("SpaceMouse moves the part")
        self.sm_part.setToolTip("Off: the SpaceMouse flies the camera. On (setup only): it moves the part,\n"
                                "kept standing on its base face while Keep on plate is ticked.\n"
                                "Left SpaceMouse button toggles this; right button shows the part.")
        self.sm_part.setVisible(False)
        vbox.addWidget(self.sm_part)
        layout.addWidget(place_box)

        sub_box = self.sub_box = QGroupBox("Substrate plate")
        sub_box.setCheckable(True)
        sub_box.setChecked(True)
        sub_box.setToolTip("Untick for no plate: the part (or the component being repaired)\n"
                           "stands directly on the faceplate")
        sub_box.toggled.connect(self._substrate_changed)
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
        self.roll = _spin(-180, 180, 0, " °", tip="Gun turned about its spray axis (changes how the wrist reaches;\n"
                                                  "the spray itself is unchanged)")
        cform.addRow("Gun roll", self.roll)
        for box in (self.e1, self.tilt, self.rotation, self.roll):
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self._enter_setup)
        self.nozzle.currentIndexChanged.connect(self._enter_setup)
        auto_btn = QPushButton("Auto position (gun level)")
        auto_btn.setToolTip("Choose table tilt, gun roll and gantry E1 so the gun is as level as allowed\n"
                            "and the whole path is reachable (also done when a part is loaded)")
        auto_btn.clicked.connect(self._auto_position)
        cform.addRow(auto_btn)
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
        self._mode_changed()
        return self._scroll(inner)

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
            self.scene.set_substrate(self._substrate_mesh())
            self._set_part(None)
            self.scene.focus_on_work()
            self.spacemouse = SpaceMouse.open()
            if self.spacemouse:
                self.sm_part.setText(f"{self.spacemouse.name} moves the part")
                self.sm_part.setVisible(True)
                self.sm_timer.start()
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
            if part.n_bodies > 1:          # an assembly: the component stands on the faceplate
                self.sub_box.blockSignals(True)
                self.sub_box.setChecked(False)
                self.sub_box.blockSignals(False)
                self.substrate = None
                self.scene.set_substrate(None)
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
        self.source, self.part, self.cell_key = part, None, None
        self.roles = ["build" if b == part.build_body else "fixture" for b in range(part.n_bodies)]
        self.scene.set_part(None)
        self._enter_setup()
        self._fill_bodies()
        self.part_view.set_part(part, self.roles)
        self._show_faces()
        if self.tabs.currentIndex() != 0:
            self.tabs.setCurrentIndex(0)
        self._status(f"Loaded {part.name}: choose the build body and start face, then Next")

    # --- Part tab ---------------------------------------------------------------
    def _fill_bodies(self):
        """One row per body: visible tick with its size, and its role."""
        while self.body_grid.count():
            self.body_grid.takeAt(0).widget().deleteLater()
        self.body_rows = []
        part = self.source
        for b in range(part.n_bodies):
            faces = part.faces if part.body_ids is None else part.faces[part.body_ids == b]
            size = np.ptp(part.vertices[np.unique(faces)], axis=0)
            shown = QCheckBox(f"Body {b + 1}: {size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm")
            shown.setChecked(True)
            shown.setToolTip("Untick to hide this body (e.g. to pick a face behind it)")
            shown.toggled.connect(lambda on, body=b: self.part_view.set_visible(body, on))
            role = QComboBox()
            for r in ROLES:
                role.addItem(r.capitalize(), r)
            role.setCurrentIndex(ROLES.index(self.roles[b]))
            role.currentIndexChanged.connect(lambda _, body=b: self._role_changed(body))
            role.setEnabled(part.n_bodies > 1)
            self.body_grid.addWidget(shown, b, 0)
            self.body_grid.addWidget(role, b, 1)
            self.body_rows.append((shown, role))

    def _role_changed(self, body):
        part, role = self.source, self.body_rows[body][1].currentData()
        if body == part.build_body and role != "build":
            self._status("One body must be the build body: set another body to Build instead")
            role = "build"
        elif role == "build":
            self.roles[part.build_body] = "fixture"
            part.build_body = body
            if part.start_face is not None and part.body_of_face(part.start_face) != body:
                part.start_face = None
                self.part_view.highlight("start", None)
        if role == "ignore" and part.base_face is not None and part.body_of_face(part.base_face) == body:
            part.base_face = None
            self.part_view.highlight("base", None)
        self.roles[body] = role
        for b, (_, combo) in enumerate(self.body_rows):
            combo.blockSignals(True)
            combo.setCurrentIndex(ROLES.index(self.roles[b]))
            combo.blockSignals(False)
        self.part_view.set_roles(self.roles)
        self._show_faces()

    def _show_bodies(self, only_build: bool):
        for b, (shown, _) in enumerate(self.body_rows):
            shown.setChecked(not only_build or b == self.source.build_body)
        self.part_view.fit()

    def _pick_face(self, kind: str, on: bool):
        """Start (or cancel) picking the start or base face in the Part view."""
        other = self.base_btn if kind == "start" else self.start_btn
        if not on:
            self.part_view.cancel_pick()
            self.part_plotter.render()
            return
        other.blockSignals(True)
        other.setChecked(False)
        other.blockSignals(False)
        hint = ("Click the face the build starts from (on the build body)" if kind == "start"
                else "Click the face the part stands on")
        if not self.part_view.start_pick(lambda f, b: self._face_picked(kind, f, b), hint):
            (self.start_btn if kind == "start" else self.base_btn).setChecked(False)

    def _face_picked(self, kind, face_id, body):
        part = self.source
        problem = None
        if face_id not in part.planar:
            problem = "Not a flat face: pick a flat face"
        elif kind == "start" and body != part.build_body:
            problem = f"That face is on body {body + 1}, not the build body"
        if problem:
            self._status(problem)
            self.part_view.start_pick(lambda f, b: self._face_picked(kind, f, b), problem)
            return
        btn = self.start_btn if kind == "start" else self.base_btn
        btn.blockSignals(True)
        btn.setChecked(False)
        btn.blockSignals(False)
        if kind == "start":
            part.start_face = face_id
        else:
            part.base_face = face_id
        self.part_view.highlight(kind, face_id)
        self._show_faces()

    def _clear_faces(self):
        self.source.start_face = self.source.base_face = None
        for kind in ("start", "base"):
            self.part_view.highlight(kind, None)
        self._show_faces()

    def _face_text(self, face_id) -> str:
        part = self.source
        face = part.planar[face_id]
        body = f", body {part.body_of_face(face_id) + 1}" if part.n_bodies > 1 else ""
        n = np.round(face.normal, 2) + 0.0
        return f"face {face_id}: {face.area:,.0f} mm²{body}, facing ({n[0]:g}, {n[1]:g}, {n[2]:g})"

    def _show_faces(self):
        part = self.source
        self.start_label.setText(
            f"<span style='color:#2bb673'>■</span> Start {self._face_text(part.start_face)}"
            if part.start_face is not None else
            "No start face: layers build straight up from the bottom of the build body")
        self.base_label.setText(
            f"<span style='color:#1a73e8'>■</span> Base {self._face_text(part.base_face)}"
            if part.base_face is not None else "No base face: placed as drawn in the CAD")

    def _tab_changed(self, index):
        self.part_view.cancel_pick()
        for btn in (self.start_btn, self.base_btn):
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)
        if index == 1:
            self._enter_cell()

    def _enter_cell(self):
        """Put the part into the cell as chosen in the Part tab. Kept as it is
        (with the user's placement) if those choices haven't changed."""
        src = self.source
        if src is None or self.scene is None:
            return
        key = (id(src), tuple(self.roles), src.build_body, src.start_face, src.base_face)
        if key == self.cell_key and self.part is not None:
            return
        ignored = [b for b, r in enumerate(self.roles) if r == "ignore"]
        self.part = src.without(ignored)
        self.part.placement = self._start_placement(self.part)
        self.cell_key = key
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
            self._status(f"{src.name} in the cell: adjust the placement if needed, then Generate")

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

        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            pos = auto_position(plan, a.robot, cell, tilts, e1_now=self.e1.value(), log=self._status)
        except ValueError as e:              # placement not plannable yet
            self._status(f"Auto position skipped: {e}")
            return False
        finally:
            QApplication.restoreOverrideCursor()
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

    def _floor(self) -> float:
        """Height the part stands on: plate top, or the faceplate."""
        return self.substrate.top if self.substrate else 0.0

    def _substrate_mesh(self):
        return self.substrate.mesh() if self.substrate else None

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

    def _substrate_changed(self, *_):
        old = self._floor()
        self.substrate = (Substrate(**{k: box.value() for k, box in self.sub_fields.items()})
                          if self.sub_box.isChecked() else None)
        if self.scene:
            self.scene.set_substrate(self._substrate_mesh())
        if self.part is not None:
            placement = self.part.placement.copy()
            placement[2, 3] += self._floor() - old     # the part stays standing on the floor
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

            name, mode = self.part.name, self.mode.currentData()
            self._status("Planning...")
            summary, path = plan_part(self.part, proc, mode=mode, tilt_deg=self.tilt.value(),
                                      rotation_deg=self.rotation.value(), coats=self.coats.value(), cell=cell,
                                      substrate=self.substrate, roll_deg=self.roll.value())
            if not path.moves:
                raise ValueError("No toolpath: the part gave no layers (is it thinner than one layer?)")
            self._status("Checking reach...")
            part_mesh, fixture_mesh = build_mesh(self.part), other_mesh(self.part)
            substrate_mesh = self._substrate_mesh()
            joints = self.scene.set_path(path, cell, part=part_mesh, substrate=substrate_mesh,
                                         fixture=fixture_mesh, step=2.0,
                                         home=cell.home_deg, angle_step=5.0,
                                         gun_limit_deg=cell.gun_max_down_deg)
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
            self._start_collision_check(robot, gun, cell, part_mesh, substrate_mesh, fixture_mesh, joints)

    def _update_info(self):
        reach = (f"<span style='color:#d93025'>{self.unreachable} unreachable points</span>" if self.unreachable
                 else "<span style='color:green'>All points reachable</span>")
        steep, limit = int(self.scene.steep.sum()), self.assets.cell.gun_max_down_deg
        gun = (f"<span style='color:#b38600'>{steep} points with the gun more than {limit:.0f}° down "
               f"(max {self.scene.gun_down.max():.0f}°, yellow)</span>" if steep
               else f"<span style='color:green'>Gun within {limit:.0f}° of level "
                    f"(max {max(self.scene.gun_down.max(), 0):.0f}° down)</span>")
        self.info.setText(self.summary_head + reach + "<br>" + gun + "<br>" + self.collision_text)

    # --- collision checking (background) -------------------------------------------
    def _start_collision_check(self, robot, gun, cell, part, substrate, fixture, joints):
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
                                                substrate=substrate, fixture=fixture)
                self.checker_e1 = e1
            else:
                self.checker.set_gun(robot, gun)
                self.checker.set_workpiece(part, substrate, fixture)
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
        steep, limit = int(self.scene.steep.sum()), cell.gun_max_down_deg
        if steep and QMessageBox.question(
                self, "Gun pointing down",
                f"At {steep} points the gun points more than {limit:.0f}° below horizontal "
                f"(down to {self.scene.gun_down.max():.0f}°), which is bad for the equipment. "
                "Save the program anyway?") != QMessageBox.StandardButton.Yes:
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

    # --- SpaceMouse ------------------------------------------------------------
    def _spacemouse_tick(self):
        now = time.time()
        dt, self.sm_last = min(now - self.sm_last, 0.1), now
        pressed = self.spacemouse.poll()
        t, r = self.spacemouse.motion()
        if self.tabs.currentIndex() == 0:            # Part tab: camera only
            if 1 in pressed:
                self.part_view.fit()
            if t.any() or r.any():
                fly(self.part_plotter, t, r, dt)
                self.part_plotter.render()
            return
        if 0 in pressed:
            self.sm_part.toggle()
        if 1 in pressed:
            (self.scene.focus_on_part if self.scene.mode == "setup" else self.scene.focus_on_work)()
            self.plotter.render()
        if not (t.any() or r.any()):
            if self.sm_moved_part:           # finished moving: settle like a drag release
                self.sm_moved_part = False
                self._set_placement(self.part.placement, redraw=False)
            return
        if self.sm_part.isChecked() and self.scene.mode == "setup" and self.part is not None:
            self._spacemouse_part(t, r, dt)
        else:
            fly(self.plotter, t, r, dt)
        self.plotter.render()

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
        spin = (-r[0] * side + r[1] * along + r[2] * n) * np.radians(turn_rate) * dt
        to_base = f[:3, :3].T
        rot = Rotation.from_rotvec(to_base @ spin).as_matrix()
        v = placed_vertices(self.part)
        centre = (v.min(axis=0) + v.max(axis=0)) / 2
        step = np.eye(4)
        step[:3, :3] = rot
        step[:3, 3] = centre - rot @ centre + to_base @ move
        placement = step @ self.part.placement
        self.part.placement = constrain(self.part, placement, self._floor()) if self._kept() else placement
        self.scene.set_part_placement()
        self._show_placement()
        self.sm_moved_part = True

    def closeEvent(self, event):
        self.play_timer.stop()
        self.poll_timer.stop()
        self.sm_timer.stop()
        if self.spacemouse:
            self.spacemouse.close()
        self.check_id += 1            # let a running check finish without effect
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.plotter.close()
        self.part_plotter.close()
        super().closeEvent(event)


def main():
    app = QApplication.instance() or QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
