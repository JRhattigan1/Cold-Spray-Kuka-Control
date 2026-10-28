"""The "Constraints" Section of the Position step: NX-style constraints between a part face and a cell reference.

Each row is: type, part face (a flat face, a cylinder axis or the part-CAD Z axis), reference (faceplate,
substrate, datum planes, table axis), value (distance mm / angle deg), suppress, delete and a status icon.
`centre_between` rows have a second part face (the reference plane lies midway between the two faces);
`fix` rows have no faces: they lock the pose captured when the row is added (or "Lock" is clicked).
`apply(placement)` solves the active rows starting from `placement` and returns the solved placement; the
header chip shows the degrees of freedom left.

Faces are picked graphically: a row's "Pick" toggle, then a click on a flat or cylindrical face of the part in
the cell view (`set_scene` gives the panel a sim.viewer.Scene). Esc or clicking Pick again cancels. The selected
row's faces are highlighted on the part. Rows can still be set programmatically (`add(kind, "face:<id>", ...)`).
"""

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import QFrame, QCheckBox, QComboBox, QHBoxLayout, QLabel, QMenu, QPushButton, QToolButton, QWidget

from coldspray.cad.constraints import TOL, Axis, Constraint, reference, solve
from coldspray.gui.style import tokens
from coldspray.gui.widgets import Section, spin

TYPES = ("touch", "align", "distance", "parallel", "perpendicular", "angle", "centre", "centre_between", "fix")
REFERENCES = ("faceplate", "substrate", "datum:XY", "datum:XZ", "datum:YZ", "table_axis")
PART_AXIS = "axis:Z"   # the part-CAD Z axis (until a cylinder face is picked)


def face_label(name: str) -> str:
    """Short label for a face name: "face:12" -> "F12", "axis:Z" -> "Z axis"."""
    if name == PART_AXIS:
        return "Z axis"
    return f"F{name.split(':', 1)[1]}" if name.startswith("face:") else name


def _rgba(hexcol: str, alpha: float) -> str:
    h = hexcol.lstrip("#")
    return f"rgba({int(h[0:2], 16)},{int(h[2:4], 16)},{int(h[4:6], 16)},{alpha})"


def glass_qss() -> str:
    """Liquid-glass look of a constraint row: translucent capsule, hairline border, orange accent."""
    t = tokens()
    return f"""
    QFrame#glassRow {{ background:{_rgba(t['panel'], 0.55)}; border:1px solid {_rgba(t['line'], 0.9)};
                       border-radius:16px; }}
    QFrame#glassRow QLabel {{ background:transparent; }}
    QLabel#faceTag {{ background:{_rgba(t['accent_soft'], 0.8)}; color:{t['ink']};
                     border:1px solid {_rgba(t['accent'], 0.35)}; border-radius:10px; padding:1px 8px; }}
    QToolButton#pickBtn {{ background:{_rgba(t['panel'], 0.7)}; border:1px solid {t['line']};
                          border-radius:13px; min-height:24px; padding:0 8px; }}
    QToolButton#pickBtn:hover {{ border-color:{t['accent']}; }}
    QToolButton#pickBtn:checked {{ background:{_rgba(t['accent'], 0.28)}; border:1px solid {t['accent']};
                                  color:{t['accent']}; }}
    """


class FaceField(QWidget):
    """A picked part face: a short label plus a "Pick" toggle (its menu offers the part Z axis).
    `currentText()` / `setCurrentText()` keep the combo-box API ("face:<id>" or PART_AXIS)."""

    changed = Signal()
    pick_toggled = Signal(bool)

    def __init__(self, faces: list, name: str | None, tip: str):
        super().__init__()
        self.faces = list(faces)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        self.label = QLabel()
        self.label.setObjectName("faceTag")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setMinimumWidth(42)
        self.pick = QToolButton()
        self.pick.setObjectName("pickBtn")
        self.pick.setText("Pick")
        self.pick.setCheckable(True)
        self.pick.setToolTip(f"{tip}: click Pick, then a flat or cylindrical face of the part in the view\n"
                             "(Esc or Pick again cancels). The arrow offers the part Z axis.")
        menu = QMenu(self.pick)
        menu.addAction("Part Z axis", lambda: self.setCurrentText(PART_AXIS))
        self.pick.setMenu(menu)
        self.pick.setPopupMode(QToolButton.MenuButtonPopup)
        self.pick.toggled.connect(self.pick_toggled)
        lay.addWidget(self.label)
        lay.addWidget(self.pick)
        self._name = ""
        self.setCurrentText(name if name is not None else (self.faces[0] if self.faces else PART_AXIS))

    def currentText(self) -> str:
        return self._name

    def setCurrentText(self, name: str):
        if name == self._name:
            return
        self._name = name
        self.label.setText(face_label(name))
        self.label.setToolTip(name)
        self.changed.emit()

    def face_id(self) -> int | None:
        return int(self._name.split(":", 1)[1]) if self._name.startswith("face:") else None

    def set_picking(self, on: bool):
        self.pick.blockSignals(True)
        self.pick.setChecked(on)
        self.pick.blockSignals(False)


class ConstraintRow(QFrame):
    changed = Signal()
    selected = Signal()

    def __init__(self, kind: str, faces: list, face: str | None = None, ref: str = "faceplate", value: float = 0.0,
                 face2: str | None = None, pose_source=None):
        super().__init__()
        self.pose_source, self.pose = pose_source, None
        self.setObjectName("glassRow")
        self.setStyleSheet(glass_qss())
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 4, 10, 4)
        self.status = QLabel("•")
        self.kind = QComboBox()
        self.kind.addItems(TYPES)
        self.kind.setCurrentText(kind)
        self.face = FaceField(faces, face, "Part face (or axis)")
        self.face2 = FaceField(faces, face2, "Second part face (centred between the two)")
        self.lock = QToolButton()
        self.lock.setText("Lock")
        self.lock.setToolTip("Lock the part at its current pose")
        self.lock.clicked.connect(self._lock)
        self.ref = QComboBox()
        self.ref.setToolTip("Cell reference")
        self.ref.addItems(REFERENCES)
        self.ref.setCurrentText(ref)
        self.value = spin(-1000, 1000, value, "", tip="Distance (mm) or angle (°); offset of the reference otherwise")
        self.suppress = QCheckBox("off")
        self.suppress.setToolTip("Suppress: keep the row but leave it out of the solve")
        self.delete = QToolButton()
        self.delete.setText("✕")
        self.delete.setToolTip("Delete this constraint")
        for w in (self.status, self.kind, self.face, self.face2, self.ref, self.value, self.lock, self.suppress,
                  self.delete):
            lay.addWidget(w)
        self.kind.currentIndexChanged.connect(self._kind_changed)
        for sig in (self.kind.currentIndexChanged, self.face.changed, self.face2.changed,
                    self.ref.currentIndexChanged, self.value.valueChanged, self.suppress.toggled):
            sig.connect(self.changed)
        self._kind_changed()

    def _kind_changed(self):
        kind = self.kind.currentText()
        fix = kind == "fix"
        for w in (self.face, self.ref, self.value):
            w.setVisible(not fix)
        self.lock.setVisible(fix)
        self.face2.setVisible(kind == "centre_between")
        if fix and self.pose is None:
            self._lock(emit=False)

    def face_ids(self) -> list[int]:
        """CAD ids of the row's picked faces (highlighted while the row is selected)."""
        kind = self.kind.currentText()
        if kind == "fix":
            return []
        fields = (self.face, self.face2) if kind == "centre_between" else (self.face,)
        return [i for i in (f.face_id() for f in fields) if i is not None]

    def mousePressEvent(self, event):
        self.selected.emit()
        super().mousePressEvent(event)

    def _lock(self, emit=True):
        """Capture the current pose (from `pose_source`) as the fix target."""
        if self.pose_source is not None:
            pose = self.pose_source()
            self.pose = None if pose is None else np.array(pose, float)
        if emit:
            self.changed.emit()

    def constraint(self, part, substrate) -> Constraint:
        kind, value = self.kind.currentText(), self.value.value()
        if kind == "fix":
            pose = self.pose if self.pose is not None else part.placement
            return Constraint("fix", pose=np.array(pose, float), suppressed=self.suppress.isChecked())
        face = self.face.currentText()
        a = Axis((0, 0, 0), (0, 0, 1), moving=True) if face == PART_AXIS else reference(face, part, substrate)
        offset = 0.0 if kind in ("distance", "angle") else value
        b = reference(self.ref.currentText(), part, substrate, offset)
        c = reference(self.face2.currentText(), part, substrate) if kind == "centre_between" else None
        return Constraint(kind, a, b, value, c=c, suppressed=self.suppress.isChecked())

    def set_status(self, residual: float | None):
        """Status as a tinted dot: grey suppressed, red conflict, green satisfied."""
        t = tokens()
        if residual is None:
            colour, tip, glyph = t["muted"], "Suppressed", "\u2022"
        elif residual > TOL:
            colour, tip, glyph = t["bad"], f"Conflict: residual {residual:.3g}", "\u2717"
        else:
            colour, tip, glyph = t["good"], "Satisfied", "\u2713"
        self.status.setText(glyph)
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setFixedSize(18, 18)
        c = colour.lstrip("#")
        rgb = f"{int(c[0:2], 16)},{int(c[2:4], 16)},{int(c[4:6], 16)}"
        self.status.setStyleSheet(f"color:{colour}; background:rgba({rgb},0.18); border:1px solid rgba({rgb},0.6); "
                                  "border-radius:9px; font-size:11px")
        self.status.setToolTip(tip)


class ConstraintsPanel(Section):
    """Constraint rows for the current part; `apply` solves them."""

    changed = Signal()
    message = Signal(str)          # status-bar text (pick prompts, rejected faces)

    def __init__(self, parent=None):
        super().__init__("Constraints", parent, open=False)
        self.part, self.substrate, self.rows, self.result = None, None, [], None
        self.scene, self.selected, self.picking = None, None, None     # picking: (row, FaceField)
        add = QPushButton("Add constraint")
        menu = QMenu(add)
        for kind in TYPES:
            menu.addAction(kind, lambda k=kind: self.add(k))
        add.setMenu(menu)
        self.body.addWidget(add)
        self.add_btn = add
        esc = QShortcut(QKeySequence(Qt.Key_Escape), self)
        esc.setContext(Qt.WidgetWithChildrenShortcut)
        esc.activated.connect(self.cancel_pick)

    def set_part(self, part, substrate=None):
        """New part: drop every row (constraints belong to one part)."""
        self.cancel_pick()
        for row in list(self.rows):
            self.remove(row)
        self.part, self.substrate = part, substrate

    def set_scene(self, scene):
        """The cell view (sim.viewer.Scene) faces are picked in and highlighted on."""
        self.cancel_pick()
        self.scene = scene

    def faces(self) -> list:
        if self.part is None:
            return [PART_AXIS]
        cyl = getattr(self.part, "cylinders", {}) or {}
        return [f"face:{i}" for i in self.part.planar] + [f"face:{i}" for i in cyl] + [PART_AXIS]

    # --- graphical face picking ----------------------------------------------
    def start_pick(self, row: ConstraintRow, field: FaceField) -> bool:
        """Pick `field`'s face in the cell view: the next flat / cylinder face clicked sets it."""
        self.cancel_pick()
        self.select(row)
        if self.scene is None or not self.scene.start_face_pick(self._face_picked, on_cancel=self._pick_cancelled):
            field.set_picking(False)
            self.message.emit("Pick a face: no part in the cell view")
            return False
        self.picking = (row, field)
        field.set_picking(True)
        self.message.emit("Click a flat or cylindrical face of the part (Esc cancels)")
        return True

    def cancel_pick(self):
        """Leave face picking (Esc, Pick clicked again, row deleted, panel hidden)."""
        if self.picking is None:
            return
        _, field = self.picking
        self.picking = None
        field.set_picking(False)
        if self.scene is not None:
            self.scene.cancel_face_pick()
        self.message.emit("Face pick cancelled")

    def _pick_cancelled(self):
        """Esc in the view (the scene has already left pick mode)."""
        if self.picking is not None:
            self.picking[1].set_picking(False)
            self.picking = None
            self.message.emit("Face pick cancelled")

    def valid_face(self, face_id: int) -> bool:
        if self.part is None:
            return False
        cyl = getattr(self.part, "cylinders", {}) or {}
        return face_id in self.part.planar or face_id in cyl

    def _face_picked(self, face_id: int):
        """A face was clicked while picking: flat / cylinder faces are taken, others rejected."""
        if self.picking is None:
            return
        if not self.valid_face(face_id):
            self.message.emit(f"Face {face_id} is neither flat nor cylindrical: pick another (Esc cancels)")
            return
        _, field = self.picking
        self.picking = None
        field.set_picking(False)
        if self.scene is not None:
            self.scene.cancel_face_pick()
        field.setCurrentText(f"face:{face_id}")
        self.message.emit(f"Constraint face set to {face_label(field.currentText())}")
        self._highlight()

    def select(self, row: ConstraintRow | None):
        """Make `row` the selected row: its faces are highlighted in the view."""
        self.selected = row
        self._highlight()

    def _highlight(self):
        if self.scene is not None:
            self.scene.highlight_part_faces([] if self.selected is None else self.selected.face_ids())
            self.scene.pl.render()

    def hideEvent(self, event):
        self.cancel_pick()
        super().hideEvent(event)

    # --- rows -------------------------------------------------------------------
    def add(self, kind: str, face: str | None = None, ref: str = "faceplate", value: float = 0.0,
            face2: str | None = None) -> ConstraintRow:
        """Add a row. `face` defaults to the part's base face (or the first face); `face2` is the second face of a
        centre_between row. A fix row locks the part's current placement."""
        if face is None and self.part is not None and self.part.base_face is not None:
            face = f"face:{self.part.base_face}"
        row = ConstraintRow(kind, self.faces(), face, ref, value, face2,
                            pose_source=lambda: None if self.part is None else self.part.placement)
        row.changed.connect(self.changed)
        row.changed.connect(lambda: row is self.selected and self._highlight())
        row.selected.connect(lambda: self.select(row))
        for field in (row.face, row.face2):
            field.pick_toggled.connect(lambda on, f=field: self.start_pick(row, f) if on else self.cancel_pick())
        row.delete.clicked.connect(lambda: self.remove(row))
        self.rows.append(row)
        self.body.addWidget(row)
        self.set_open(True)
        self.select(row)
        self.changed.emit()
        return row

    def remove(self, row: ConstraintRow):
        if self.picking is not None and self.picking[0] is row:
            self.cancel_pick()
        self.rows.remove(row)
        if row is self.selected:
            self.select(self.rows[-1] if self.rows else None)
        row.setParent(None)
        row.deleteLater()
        self.changed.emit()

    def rows_state(self) -> list[dict]:
        """The rows as plain dicts (undo snapshots); `set_rows` rebuilds them."""
        return [dict(kind=r.kind.currentText(), face=r.face.currentText(), face2=r.face2.currentText(),
                     ref=r.ref.currentText(), value=r.value.value(), suppressed=r.suppress.isChecked(),
                     pose=None if r.pose is None else np.array(r.pose, float)) for r in self.rows]

    def set_rows(self, state: list[dict]):
        """Replace every row with `state` (from `rows_state`) without emitting `changed`."""
        self.blockSignals(True)
        try:
            self.cancel_pick()
            for row in list(self.rows):
                self.remove(row)
            for s in state:
                row = self.add(s["kind"], s["face"] or None, s["ref"], s["value"], s["face2"] or None)
                row.pose = None if s["pose"] is None else np.array(s["pose"], float)
                row.suppress.setChecked(s["suppressed"])      # panel signals are blocked
        finally:
            self.blockSignals(False)
        if not self.rows:
            self.result = None
            self.chip.setVisible(False)

    def active(self) -> bool:
        return any(not r.suppress.isChecked() for r in self.rows)

    def apply(self, placement) -> np.ndarray:
        """Solve the rows from `placement`; update row icons and the DOF chip; return the solved placement."""
        placement = np.asarray(placement, float)
        if not self.rows or self.part is None:
            self.result = None
            self.chip.setVisible(False)
            return placement
        cons = [r.constraint(self.part, self.substrate) for r in self.rows]
        self.result = r = solve(placement, cons)
        for row, c, res in zip(self.rows, cons, r.residuals):
            row.set_status(None if c.suppressed else float(res))
        bad = r.status != "ok"
        self.set_chip("conflict" if bad else f"{r.dof_left} DOF free", "bad" if bad else "neutral")
        return r.placement
