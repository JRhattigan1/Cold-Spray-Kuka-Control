"""Step 1 Part: load a part or assembly, body roles and visibility, start, base and coat faces."""

from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QGridLayout, QHBoxLayout, QMessageBox, QPushButton,
    QVBoxLayout, QWidget,
)

from coldspray.cad.part import grow_tangent
from coldspray.gui.stepper import PART, POSITION, TOOLPATH
from coldspray.gui.widgets import Section, primary_button, wrap_label
from coldspray.models import PROJECT_ROOT
from coldspray.pipeline import load_part
from coldspray.sim.part_view import ROLES


class PartStep:
    def _build_part_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)

        part_box = self.part_section = Section("Part")
        self.part_label = wrap_label("test block (built in)")
        open_btn = QPushButton("Open STEP file...")
        open_btn.clicked.connect(self._open_step)
        row = QHBoxLayout()
        for text, key in (("Test block", "test_block"), ("Test tube", "test_tube"),
                          ("Test repair", "test_repair")):
            btn = QPushButton(text)
            btn.clicked.connect(lambda _=False, k=key: self._set_part(k))
            row.addWidget(btn)
        part_box.add(self.part_label)
        part_box.add(open_btn)
        part_box.add(row)
        layout.addWidget(part_box)

        body_box = self.bodies_section = Section("Bodies")
        self.body_grid = QGridLayout()
        self.body_rows: list[tuple[QCheckBox, QComboBox]] = []
        body_box.add(self.body_grid)
        row = QHBoxLayout()
        isolate = QPushButton("Isolate build body")
        isolate.setToolTip("Hide the other bodies, e.g. to reach the underside of a repair patch")
        isolate.clicked.connect(lambda: self._show_bodies(only_build=True))
        show_all = QPushButton("Show all")
        show_all.clicked.connect(lambda: self._show_bodies(only_build=False))
        row.addWidget(isolate)
        row.addWidget(show_all)
        body_box.add(row)
        body_box.add(wrap_label("<small>Build: sliced and sprayed. Fixture: the component being repaired, "
                                "clamps - checked for collisions. Ignore: left out.</small>"))
        layout.addWidget(body_box)

        face_box = self.faces_section = Section("Faces")
        self.start_btn = QPushButton("Pick start face")
        self.start_btn.setCheckable(True)
        self.start_btn.setToolTip("Click a flat face of the build body: the layers grow away from it,\n"
                                  "parallel to it (e.g. the underside of a repair patch)")
        self.start_btn.toggled.connect(lambda on: self._pick_face("start", on))
        self.start_label = wrap_label()
        self.base_btn = QPushButton("Pick base face (optional)")
        self.base_btn.setCheckable(True)
        self.base_btn.setToolTip("Click a flat face (any body) for the part to stand on: it is turned\n"
                                 "to face down onto the plate, or the faceplate without one")
        self.base_btn.toggled.connect(lambda on: self._pick_face("base", on))
        self.base_label = wrap_label()
        clear_btn = QPushButton("Clear faces")
        clear_btn.clicked.connect(self._clear_faces)
        for w in (self.start_btn, self.start_label, self.base_btn, self.base_label, clear_btn):
            face_box.add(w)
        layout.addWidget(face_box)

        coat_box = self.coat_section = Section("Coat faces")
        self.coat_btn = QPushButton("Pick coat faces")
        self.coat_btn.setCheckable(True)
        self.coat_btn.setToolTip("Faces to coat in Surface mode, any body. Click toggles a face,\n"
                                 "Shift+click adds, Ctrl+click removes. Click the button again to stop")
        self.coat_btn.toggled.connect(self._pick_coat)
        grow = QPushButton("Grow to tangent faces")
        grow.setToolTip("Add the faces joined smoothly (normals within 10 degrees) to the chosen ones")
        grow.clicked.connect(self._grow_coat)
        clear_coat = QPushButton("Clear")
        clear_coat.clicked.connect(lambda: self._set_coat(set()))
        row = QHBoxLayout()
        row.addWidget(grow)
        row.addWidget(clear_coat)
        self.coat_label = wrap_label()
        coat_box.add(self.coat_btn)
        coat_box.add(row)
        coat_box.add(self.coat_label)
        layout.addWidget(coat_box)

        layout.addWidget(primary_button("Next: Position  →", lambda: self.go_to(POSITION)))
        layout.addStretch(1)
        return inner

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
                self.sub_check.blockSignals(True)
                self.sub_check.setChecked(False)
                self.sub_check.blockSignals(False)
                self.substrate = None
                self.scene.set_substrate(None)
                self._substrate_ui()
            builtin = source is None or not Path(str(source)).suffix
            self.part_label.setText(f"{part.name} (built in)" if builtin else Path(source).name)
            self.stepbar.set_chip_text(part.name if builtin else Path(source).name)
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
        self._reset_steps()                # a new part: the later steps are not reached yet
        self._fill_bodies()
        self.part_view.set_part(part, self.roles)
        self._show_faces()
        if self.step != PART:
            self.go_to(PART)
        self._status(f"Loaded {part.name}: choose the build body and start face, then Next")

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
        self.bodies_section.set_chip(str(part.n_bodies))

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
        for btn in (other, self.coat_btn):
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)
        hint = ("Click the face the build starts from (on the build body)" if kind == "start"
                else "Click the face the part stands on")
        if not self.part_view.start_pick(lambda f, b: self._face_picked(kind, f, b), hint):
            (self.start_btn if kind == "start" else self.base_btn).setChecked(False)

    def _cancel_pick(self):
        """Stop any face picking (leaving the Part step)."""
        self.part_view.cancel_pick()
        for btn in (self.start_btn, self.base_btn, self.coat_btn):
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)

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

    def _pick_coat(self, on: bool):
        """Start (or stop) picking coat faces: every click changes the set until stopped."""
        if not on:
            self.part_view.cancel_pick()
            self.part_plotter.render()
            return
        for btn in (self.start_btn, self.base_btn):
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)
        if not self.part_view.start_pick(self._coat_picked, "Click faces to coat (Shift adds, Ctrl removes)",
                                         keep=True):
            self.coat_btn.setChecked(False)

    def _coat_picked(self, face_id, _body):
        mods = QApplication.keyboardModifiers()
        faces = set(self.source.coat_faces)
        if mods & Qt.ShiftModifier:
            faces.add(face_id)
        elif mods & Qt.ControlModifier:
            faces.discard(face_id)
        else:
            faces ^= {face_id}
        self._set_coat(faces)

    def _grow_coat(self):
        if not self.source.coat_faces:
            self._status("Pick a coat face first, then grow it to the faces joined smoothly to it")
            return
        self._set_coat(grow_tangent(self.source, self.source.coat_faces))

    def _set_coat(self, faces):
        self.source.coat_faces = set(faces)
        if self.part is not None:       # already in the cell: the cell copy plans Surface mode
            self.part.coat_faces = self.source.coat_faces & set(self.part.face_ids.tolist())
            if self.result is not None and getattr(self, "result_mode", None) == "surface":
                self._invalidate(TOOLPATH)
        self.part_view.highlight_coat(self.source.coat_faces)
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
        """Face labels; called after every change of the Part choices, so also refreshes the step bar."""
        part = self.source
        self.start_label.setText(
            f"<span style='color:#2bb673'>■</span> Start {self._face_text(part.start_face)}"
            if part.start_face is not None else
            "No start face: layers build straight up from the bottom of the build body")
        self.base_label.setText(
            f"<span style='color:#1a73e8'>■</span> Base {self._face_text(part.base_face)}"
            if part.base_face is not None else "No base face: placed as drawn in the CAD")
        coat = sorted(part.coat_faces)
        self.coat_label.setText(
            f"<span style='color:#f28c28'>■</span> {len(coat)} face{'s' * (len(coat) != 1)}: "
            + ", ".join(map(str, coat)) if coat else "No coat faces (only needed for Surface mode)")
        self.coat_section.set_chip(str(len(coat)) if coat else "")
        self._refresh_steps()
