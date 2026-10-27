"""The "Constraints" Section of the Position step: NX-style constraints between a part face and a cell reference.

Each row is: type, part face (a flat face, a cylinder axis or the part-CAD Z axis), reference (faceplate,
substrate, datum planes, table axis), value (distance mm / angle deg), suppress, delete and a status icon.
`centre_between` rows have a second part face (the reference plane lies midway between the two faces);
`fix` rows have no faces: they lock the pose captured when the row is added (or "Lock" is clicked).
`apply(placement)` solves the active rows starting from `placement` and returns the solved placement; the
header chip shows the degrees of freedom left.
"""

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QCheckBox, QComboBox, QHBoxLayout, QLabel, QMenu, QPushButton, QToolButton, QWidget

from coldspray.cad.constraints import TOL, Axis, Constraint, reference, solve
from coldspray.gui.widgets import Section, spin

TYPES = ("touch", "align", "distance", "parallel", "perpendicular", "angle", "centre", "centre_between", "fix")
REFERENCES = ("faceplate", "substrate", "datum:XY", "datum:XZ", "datum:YZ", "table_axis")
PART_AXIS = "axis:Z"   # the part-CAD Z axis (until a cylinder face is picked)


class ConstraintRow(QWidget):
    changed = Signal()

    def __init__(self, kind: str, faces: list, face: str | None = None, ref: str = "faceplate", value: float = 0.0,
                 face2: str | None = None, pose_source=None):
        super().__init__()
        self.pose_source, self.pose = pose_source, None
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.status = QLabel("•")
        self.kind = QComboBox()
        self.kind.addItems(TYPES)
        self.kind.setCurrentText(kind)
        self.face = QComboBox()
        self.face.setToolTip("Part face (or axis)")
        for name in faces:
            self.face.addItem(name)
        if face is not None:
            self.face.setCurrentText(face)
        self.face2 = QComboBox()
        self.face2.setToolTip("Second part face (centred between the two)")
        self.face2.addItems(faces)
        if face2 is not None:
            self.face2.setCurrentText(face2)
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
        for sig in (self.kind.currentIndexChanged, self.face.currentIndexChanged, self.face2.currentIndexChanged,
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
        if residual is None:
            self.status.setText("•")
            self.status.setToolTip("Suppressed")
        elif residual > TOL:
            self.status.setText("✗")
            self.status.setStyleSheet("color:#d93025")
            self.status.setToolTip(f"Conflict: residual {residual:.3g}")
        else:
            self.status.setText("✓")
            self.status.setStyleSheet("color:#188038")
            self.status.setToolTip("Satisfied")


class ConstraintsPanel(Section):
    """Constraint rows for the current part; `apply` solves them."""

    changed = Signal()

    def __init__(self, parent=None):
        super().__init__("Constraints", parent, open=False)
        self.part, self.substrate, self.rows, self.result = None, None, [], None
        add = QPushButton("Add constraint")
        menu = QMenu(add)
        for kind in TYPES:
            menu.addAction(kind, lambda k=kind: self.add(k))
        add.setMenu(menu)
        self.body.addWidget(add)
        self.add_btn = add

    def set_part(self, part, substrate=None):
        """New part: drop every row (constraints belong to one part)."""
        for row in list(self.rows):
            self.remove(row)
        self.part, self.substrate = part, substrate

    def faces(self) -> list:
        if self.part is None:
            return [PART_AXIS]
        cyl = getattr(self.part, "cylinders", {}) or {}
        return [f"face:{i}" for i in self.part.planar] + [f"face:{i}" for i in cyl] + [PART_AXIS]

    def add(self, kind: str, face: str | None = None, ref: str = "faceplate", value: float = 0.0,
            face2: str | None = None) -> ConstraintRow:
        """Add a row. `face` defaults to the part's base face (or the first face); `face2` is the second face of a
        centre_between row. A fix row locks the part's current placement."""
        if face is None and self.part is not None and self.part.base_face is not None:
            face = f"face:{self.part.base_face}"
        row = ConstraintRow(kind, self.faces(), face, ref, value, face2,
                            pose_source=lambda: None if self.part is None else self.part.placement)
        row.changed.connect(self.changed)
        row.delete.clicked.connect(lambda: self.remove(row))
        self.rows.append(row)
        self.body.addWidget(row)
        self.set_open(True)
        self.changed.emit()
        return row

    def remove(self, row: ConstraintRow):
        self.rows.remove(row)
        row.setParent(None)
        row.deleteLater()
        self.changed.emit()

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
