"""Step 1 Part: load a part or assembly, body roles and visibility, start, base and coat faces,
and the optional KRL base on the part."""

from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QHBoxLayout, QMessageBox, QPushButton,
    QSizePolicy, QVBoxLayout, QWidget, QFrame,
)

from coldspray.cad.part import (
    flip_z, grow_tangent, krl_base_at_vertex, krl_base_frame, krl_base_from_planes, spin_frame, vertex_edges,
)
from coldspray.gui.stepper import PART, POSITION, TOOLPATH
from coldspray.gui.style import tokens
from coldspray.gui.widgets import Section, fit_combo, primary_button, wrap_label
from coldspray.models import PROJECT_ROOT
from coldspray.pipeline import BUILT_IN_LABELS, load_part
from coldspray.sim.part_view import ROLES


def glass_card(widget_layout) -> QFrame:
    """Display only: a translucent rounded card holding `widget_layout`."""
    t = tokens()
    card = QFrame()
    card.setObjectName("glassCard")
    card.setStyleSheet(f"QFrame#glassCard {{ background: {t['soft']}; border: 1px solid {t['line']}; "
                       f"border-radius: 14px; }} QFrame#glassCard QLabel {{ background: transparent; }}")
    widget_layout.setContentsMargins(10, 10, 10, 10)
    widget_layout.setSpacing(8)
    card.setLayout(widget_layout)
    return card


def capsule(btn: QPushButton, pick: bool = False) -> QPushButton:
    """Display only: pill-shaped glass button; a pick toggle glows orange while checked."""
    t = tokens()
    btn.setMinimumHeight(30)
    btn.setCursor(Qt.PointingHandCursor)
    css = (f"QPushButton {{ background: {t['panel']}; border: 1px solid {t['line']}; border-radius: 15px; "
           f"padding: 4px 10px; }} QPushButton:hover {{ border-color: {t['accent']}; }}")
    if pick:
        css += (f" QPushButton:checked {{ background: {t['accent_soft']}; border: 2px solid {t['accent']}; "
                f"color: {t['accent']}; font-weight: 600; }}")
    btn.setStyleSheet(css)
    return btn


def capsule_row(*buttons, pick: bool = False) -> QHBoxLayout:
    row = QHBoxLayout()
    row.setSpacing(8)
    for b in buttons:
        row.addWidget(capsule(b, pick))
    return row


from coldspray.gui.progress import fit_panel


class PartStep:
    def _build_part_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setSpacing(12)

        part_box = self.part_section = Section("Part")
        self.part_label = wrap_label("test block (built in)")
        open_btn = QPushButton("Open STEP file...")
        open_btn.clicked.connect(self._open_step)
        self.test_parts = QComboBox()
        self.test_parts.setToolTip("Load a built-in test part instead of a STEP file")
        self.test_parts.addItem("Test part...", None)
        for key, text in BUILT_IN_LABELS.items():
            self.test_parts.addItem(text, key)
        self.test_parts.activated.connect(self._test_part_chosen)
        fit_combo(self.test_parts)
        card = QVBoxLayout()
        card.addWidget(self.part_label)
        card.addLayout(capsule_row(open_btn))
        card.addWidget(self.test_parts)
        part_box.add(glass_card(card))
        layout.addWidget(part_box)

        body_box = self.bodies_section = Section("Bodies")
        self.body_grid = QGridLayout()
        self.body_rows: list[tuple[QCheckBox, QComboBox]] = []
        body_box.add(self.body_grid)
        isolate = QPushButton("Isolate build")
        isolate.setToolTip("Hide the other bodies, e.g. to reach the underside of a repair patch")
        isolate.clicked.connect(lambda: self._show_bodies(only_build=True))
        show_all = QPushButton("Show all")
        show_all.clicked.connect(lambda: self._show_bodies(only_build=False))
        body_box.add(capsule_row(isolate, show_all))
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
        self.base_btn = QPushButton("Pick base face")
        self.base_btn.setCheckable(True)
        self.base_btn.setToolTip("Click a flat face (any body) for the part to stand on: it is turned\n"
                                 "to face down onto the plate, or the faceplate without one")
        self.base_btn.toggled.connect(lambda on: self._pick_face("base", on))
        self.base_label = wrap_label()
        clear_btn = QPushButton("Clear faces")
        clear_btn.clicked.connect(self._clear_faces)
        capsule(self.start_btn, True)
        capsule(self.base_btn, True)
        capsule(clear_btn)
        for w in (self.start_btn, self.start_label, self.base_btn, self.base_label, clear_btn):
            face_box.add(w)
        layout.addWidget(face_box)

        coat_box = self.coat_section = Section("Coat faces")
        self.coat_btn = QPushButton("Pick coat faces")
        self.coat_btn.setCheckable(True)
        self.coat_btn.setToolTip("Faces to coat in Surface mode, any body. Click toggles a face,\n"
                                 "Shift+click adds, Ctrl+click removes. Click the button again to stop")
        self.coat_btn.toggled.connect(self._pick_coat)
        grow = QPushButton("Grow tangent")
        grow.setToolTip("Add the faces joined smoothly (normals within 10 degrees) to the chosen ones")
        grow.clicked.connect(self._grow_coat)
        clear_coat = QPushButton("Clear")
        clear_coat.clicked.connect(lambda: self._set_coat(set()))
        row = capsule_row(grow, clear_coat)
        capsule(self.coat_btn, True)
        self.coat_label = wrap_label()
        coat_box.add(self.coat_btn)
        coat_box.add(row)
        coat_box.add(self.coat_label)
        layout.addWidget(coat_box)

        krl_box = self.krl_section = Section("KRL base (optional)")
        self.krl_btn = QPushButton("Pick KRL base")
        self.krl_btn.setCheckable(True)
        self.krl_btn.setToolTip("Write the program relative to a base on the part, taught on the real part")
        self.krl_btn.toggled.connect(self._pick_krl_base)
        clear_krl = QPushButton("Clear")
        clear_krl.clicked.connect(lambda: self._set_krl_base(None, None))
        row = capsule_row(self.krl_btn, clear_krl)
        capsule(self.krl_btn, True)
        self.krl_method = QComboBox()
        for text, key, tip in (
                ("Face + edge", "face", "Click a flat face near a corner, beside the edge to use as X:\n"
                                        "+Z out of the face, +X along that edge, origin at the corner"),
                ("3 planes", "planes", "Click three flat faces: origin where they meet, +Z out of the 1st,\n"
                                       "+X along the 1st/2nd face line, pointing into the part"),
                ("Vertex", "vertex", "Hover snaps to the nearest corner; click to place it there:\n"
                                     "+Z out of the face under the cursor, +X along the edge toward the cursor")):
            self.krl_method.addItem(text, key)
            self.krl_method.setItemData(self.krl_method.count() - 1, tip, Qt.ToolTipRole)
        self.krl_method.setToolTip("How the KRL base is picked")
        self.krl_method.currentIndexChanged.connect(lambda _: self.krl_btn.setChecked(False))
        fit_combo(self.krl_method)
        self.krl_spin = QDoubleSpinBox()
        self.krl_spin.setRange(-180.0, 180.0)
        self.krl_spin.setDecimals(1)
        self.krl_spin.setWrapping(True)
        self.krl_spin.setSuffix("°")
        self.krl_spin.setToolTip("Turn the base about its Z (origin and Z kept)")
        self.krl_spin.valueChanged.connect(lambda _: self._apply_krl_spin())
        turns = []
        for text, step in (("-90°", -90.0), ("+90°", 90.0)):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, d=step: self._turn_krl(d))
            turns.append(b)
        self.krl_flip = QPushButton("Flip Z")
        self.krl_flip.setCheckable(True)
        self.krl_flip.setToolTip("Reverse Z (origin and X kept, Y follows)")
        self.krl_flip.toggled.connect(lambda _: self._apply_krl_spin())
        spin_row = capsule_row(*turns)
        self.krl_spin.setMinimumWidth(90)   # the ±180.0° hint is wider than needed
        spin_row.insertWidget(0, self.krl_spin)
        self._krl_raw = None              # the base as picked, before spin / flip
        self._krl_how = ""                # how it was made, for the label
        self._krl_planes = []             # 3 planes: the faces picked so far
        self.krl_label = wrap_label()
        krl_box.add(self.krl_method)
        krl_box.add(row)
        krl_box.add(spin_row)
        krl_box.add(capsule_row(self.krl_flip))
        krl_box.add(self.krl_label)
        layout.addWidget(krl_box)

        layout.addWidget(primary_button("Next: Position  →", lambda: self.go_to(POSITION)))
        layout.addStretch(1)
        fit_panel(inner)
        return inner

    def _open_step(self):
        start = str(PROJECT_ROOT / "assets" / "cad")
        fn, _ = QFileDialog.getOpenFileName(self, "Open part", start, "STEP files (*.stp *.step);;All files (*)")
        if fn:
            self._set_part(fn)

    def _test_part_chosen(self, index):
        """Load the built-in part picked in the drop-down, then show the prompt again."""
        key = self.test_parts.itemData(index)
        self.test_parts.setCurrentIndex(0)
        if key:
            self._set_part(key)

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
            self.part_label.setText(f"{BUILT_IN_LABELS.get(source, part.name)} (built in)" if builtin
                                    else Path(source).name)
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
        self.part_source = source          # what was loaded (STEP path or built-in name), for job files
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
            item = self.body_grid.takeAt(0)
            item.widget().hide()
            item.widget().deleteLater()
        self.body_rows = []
        part = self.source
        for b in range(part.n_bodies):
            faces = part.faces if part.body_ids is None else part.faces[part.body_ids == b]
            size = np.ptp(part.vertices[np.unique(faces)], axis=0)
            shown = QCheckBox(f"Body {b + 1}")
            dims = wrap_label(f"<small>{size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm</small>")
            shown.setChecked(True)
            shown.setToolTip("Untick to hide this body (e.g. to pick a face behind it)")
            shown.toggled.connect(lambda on, body=b: self.part_view.set_visible(body, on))
            role = QComboBox()
            for r in ROLES:
                role.addItem(r.capitalize(), r)
            role.setCurrentIndex(ROLES.index(self.roles[b]))
            role.currentIndexChanged.connect(lambda _, body=b: self._role_changed(body))
            role.setEnabled(part.n_bodies > 1)
            fit_combo(role, 6)
            self.body_grid.addWidget(shown, 2 * b, 0)
            self.body_grid.addWidget(role, 2 * b, 1)
            self.body_grid.addWidget(dims, 2 * b + 1, 0, 1, 2)
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
        for btn in (other, self.coat_btn, self.krl_btn):
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
        for btn in (self.start_btn, self.base_btn, self.coat_btn, self.krl_btn):
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
        for btn in (self.start_btn, self.base_btn, self.krl_btn):
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

    def _pick_krl_base(self, on: bool):
        """Start (or cancel) picking the KRL base by the chosen method: one click on a flat
        face near a corner, three flat faces in turn, or a click on a (snapped) vertex."""
        self._krl_planes_clear()
        if not on:
            self.part_view.cancel_pick()
            self.part_plotter.render()
            return
        for btn in (self.start_btn, self.base_btn, self.coat_btn):
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)
        method = self.krl_method.currentData()
        if method == "planes":
            ok = self.part_view.start_pick(self._krl_plane_picked, self._krl_plane_hint())
        elif method == "vertex":
            from scipy.spatial import cKDTree
            v = self.source.vertices
            self._krl_tree = cKDTree(v)          # once per pick: a hover is one tree query
            self._krl_snap = 0.05 * float(np.ptp(v, axis=0).max())
            self._krl_hover_key = self._krl_edges = self._krl_preview = None
            ok = self.part_view.start_pick(self._krl_vertex_clicked, "Hover to snap to a corner, click to place",
                                           hover=self._krl_vertex_hover)
        else:
            ok = self.part_view.start_pick(self._krl_base_picked,
                                           "Click a flat face near the origin corner, beside the X edge")
        if not ok:
            self.krl_btn.setChecked(False)

    def _krl_done(self):
        self.krl_btn.blockSignals(True)
        self.krl_btn.setChecked(False)
        self.krl_btn.blockSignals(False)

    def _krl_base_picked(self, face_id, _body):
        if face_id not in self.source.planar:
            self._status("Not a flat face: pick a flat face for the KRL base")
            self.part_view.start_pick(self._krl_base_picked, "Not a flat face: pick a flat face")
            return
        self._krl_done()
        self._set_krl_base(krl_base_frame(self.source, face_id, self.part_view.pick_point), face_id)

    # 3 planes: the faces picked so far stay highlighted until the third
    def _krl_plane_hint(self, problem="") -> str:
        what = ("1st flat face (+Z out of it)", "2nd flat face (+X along its line with the 1st)",
                "3rd flat face (the origin is where all three meet)")[len(self._krl_planes)]
        return (problem + ": " if problem else "") + f"Click the {what}"

    def _krl_planes_clear(self):
        for k in range(len(self._krl_planes)):
            self.part_view.highlight(f"krl{k}", None)
        self._krl_planes = []

    def _krl_plane_picked(self, face_id, _body):
        part, problem = self.source, ""
        if face_id not in part.planar:
            problem = "Not a flat face"
        elif face_id in self._krl_planes:
            problem = "Already picked"
        else:
            self._krl_planes.append(face_id)
            self.part_view.highlight(f"krl{len(self._krl_planes) - 1}", face_id)
        if len(self._krl_planes) == 3:
            planes = [part.planar[f] for f in self._krl_planes]
            self._krl_planes_clear()
            try:
                t = krl_base_from_planes(planes, part.vertices.mean(0))
            except ValueError:
                problem = "Faces (nearly) parallel, start again"
            else:
                self._krl_done()
                self._set_krl_base(t, None, "from 3 planes")
                return
        if problem:
            self._status(self._krl_plane_hint(problem))
        self.part_view.start_pick(self._krl_plane_picked, self._krl_plane_hint(problem))

    # vertex: hover snaps to the nearest vertex and previews the base, click places it
    def _krl_at(self, face_id, point):
        """(vertex, base) snapped from part-CAD `point` on CAD face `face_id`, or (None, None)."""
        dist, i = self._krl_tree.query(np.asarray(point, float))
        if dist > self._krl_snap:
            return None, None
        vertex = self.source.vertices[i]
        key = tuple(np.round(vertex, 6))
        if key != self._krl_hover_key:            # the edges only change with the vertex
            self._krl_hover_key, self._krl_edges = key, vertex_edges(self.source, vertex)
        edges, normal = self._krl_edges
        z = self.source.planar[face_id].normal if face_id in self.source.planar else normal
        return vertex, krl_base_at_vertex(vertex, z, edges, np.asarray(point, float) - vertex)

    def _krl_vertex_hover(self, tri, point):
        vertex, t = (None, None) if tri is None else self._krl_at(int(self.source.face_ids[tri]), point)
        shown = self._krl_preview
        if (t is None and shown is None) or (t is not None and shown is not None and np.allclose(shown, t)):
            return                                # nothing new: no render
        self._krl_preview = t
        self.part_view.show_preview(vertex, t)

    def _krl_vertex_clicked(self, face_id, _body):
        self._krl_preview = None
        _, t = self._krl_at(face_id, self.part_view.pick_point)
        if t is None:
            self._status("No corner near the click: click closer to a vertex")
            self.part_view.start_pick(self._krl_vertex_clicked, "No corner there: click near a vertex",
                                      hover=self._krl_vertex_hover)
            return
        self._krl_done()
        self._set_krl_base(t, None, "at a vertex")

    def _set_krl_base(self, t, face_id, how=""):
        """Use KRL base `t` (part-CAD <- base, picked on `face_id`, None if not made on one face),
        or None for the faceplate BASE. Spin and flip start again from it.
        Only the KRL output changes, so the toolpath stays valid."""
        self._krl_raw, self._krl_how = (None if t is None else np.array(t, float)), how
        for w in (self.krl_spin, self.krl_flip):
            w.blockSignals(True)
        self.krl_spin.setValue(0.0)
        self.krl_flip.setChecked(False)
        for w in (self.krl_spin, self.krl_flip):
            w.blockSignals(False)
        for p in (self.source, self.part):
            if p is not None:
                p.krl_base_face = face_id
        self._apply_krl_spin()

    def _turn_krl(self, deg):
        v = self.krl_spin.value() + deg
        self.krl_spin.setValue((v + 180.0) % 360.0 - 180.0)

    def _apply_krl_spin(self):
        """The picked base turned by the spin about its Z, then Z flipped if ticked."""
        t = self._krl_raw
        if t is not None:
            t = spin_frame(t, self.krl_spin.value())
            if self.krl_flip.isChecked():
                t = flip_z(t)
        for p in (self.source, self.part):
            if p is not None:
                p.krl_base = t
        self.part_view.show_frame(t)
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
            f"<span style='color:#c2410c'>■</span> Base {self._face_text(part.base_face)}"
            if part.base_face is not None else "No base face: placed as drawn in the CAD")
        coat = sorted(part.coat_faces)
        self.coat_label.setText(
            f"<span style='color:#f28c28'>■</span> {len(coat)} face{'s' * (len(coat) != 1)}: "
            + ", ".join(map(str, coat)) if coat else "No coat faces (only needed for Surface mode)")
        self.coat_section.set_chip(str(len(coat)) if coat else "")
        if part.krl_base is None:
            self.krl_label.setText("No KRL base: the program is relative to the faceplate BASE")
            self.krl_section.set_chip("")
        else:
            o = np.round(part.krl_base[:3, 3], 1) + 0.0
            x = np.round(part.krl_base[:3, 0], 2) + 0.0
            cell = getattr(self.assets, "cell", None)
            number = f"[{cell.part_base_number}]" if cell is not None else ""
            on = (f"on {self._face_text(part.krl_base_face)}" if part.krl_base_face in part.planar
                  else self._krl_how or "on the part")
            if self.krl_spin.value():
                on += f", turned {self.krl_spin.value():g}° about Z"
            if self.krl_flip.isChecked():
                on += ", Z flipped"
            self.krl_label.setText(
                f"<span style='color:#d93025'>X</span> <span style='color:#188038'>Y</span> "
                f"<span style='color:#1a73e8'>Z</span> {on}<br>"
                f"Origin ({o[0]:g}, {o[1]:g}, {o[2]:g}), X ({x[0]:g}, {x[1]:g}, {x[2]:g}) in the CAD."
                f" Teach it on the part as BASE_DATA{number}")
            self.krl_section.set_chip("set")
        self._refresh_steps()
