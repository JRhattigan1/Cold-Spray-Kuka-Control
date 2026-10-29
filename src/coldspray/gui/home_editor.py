"""HOME editor: A1-A6 (and E1) spin boxes; the robot in the cell view follows live,
a status line shows collisions and limit violations, Save writes `home_deg:` to cell.yaml."""
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QDoubleSpinBox, QFormLayout, QHBoxLayout, QLabel, QPushButton,
                               QVBoxLayout, QWidget)

from coldspray.cell import robroot, table_axis_names
from coldspray.config_io import save_home
from coldspray.models import load_cell, load_robot
from coldspray.sim.scene import gantry_offset

JOINTS = ("A1", "A2", "A3", "A4", "A5", "A6")


class HomeEditor(QWidget):
    pose_changed = Signal(list, float)          # joints (deg), E1 (mm)

    def __init__(self, win, path=None, parent=None):
        super().__init__(parent)
        self.win, self.path = win, path
        assets = getattr(win, "assets", None)
        self.cell = assets.cell if assets is not None else load_cell()      # the live CellConfig
        robot = getattr(assets, "robot", None)
        cfg = getattr(robot, "cfg", None) or load_robot()
        self.limits = [tuple(map(float, lim)) for lim in cfg.joint_limits_deg]

        form = QFormLayout()
        self.spins = []
        for name, (lo, hi) in zip(JOINTS, self.limits):
            s = QDoubleSpinBox()
            s.setRange(-360.0, 360.0)          # out-of-limit values are allowed but flagged
            s.setDecimals(1)
            s.setSuffix(" °")
            s.setToolTip(f"{name} limits {lo:g} .. {hi:g} °")
            s.valueChanged.connect(self._changed)
            form.addRow(name, s)
            self.spins.append(s)
        self.e1 = None
        if self.cell.gantry is not None:
            self.e1 = QDoubleSpinBox()
            self.e1.setRange(*map(float, self.cell.gantry.limits_mm))
            self.e1.setDecimals(1)
            self.e1.setSuffix(" mm")
            self.e1.valueChanged.connect(self._changed)
            form.addRow(f"{self.cell.gantry.axis} (gantry)", self.e1)

        self.status = QLabel()
        self.status.setWordWrap(True)
        reset = QPushButton("Reset to cell.yaml")
        reset.clicked.connect(self.reset)
        save = QPushButton("Save to cell.yaml")
        save.clicked.connect(self.save)
        buttons = QHBoxLayout()
        buttons.addWidget(reset)
        buttons.addWidget(save)
        lay = QVBoxLayout(self)
        lay.addLayout(form)
        lay.addWidget(self.status)
        lay.addLayout(buttons)
        lay.addStretch(1)
        self.reset()

    def joints(self) -> list[float]:
        return [s.value() for s in self.spins]

    def e1_mm(self) -> float:
        return self.e1.value() if self.e1 is not None else 0.0

    def reset(self):
        """Back to home_deg and the gantry position in the live cell config."""
        for s, v in zip(self.spins, self.cell.home_deg):
            s.blockSignals(True)
            s.setValue(float(v))
            s.blockSignals(False)
        if self.e1 is not None:
            self.e1.blockSignals(True)
            self.e1.setValue(float(self.cell.gantry.position_mm))
            self.e1.blockSignals(False)
        self._changed()

    def save(self):
        """Write home_deg to cell.yaml and update the live config (Simulate and KRL use it)."""
        q = self.joints()
        save_home(q, self.path)
        self.cell.home_deg = q
        self._changed()

    def showEvent(self, event):
        super().showEvent(event)
        self._changed()

    def hideEvent(self, event):
        """Put the robot back at the cell's HOME and gantry once the tab is left."""
        super().hideEvent(event)
        scene = getattr(self.win, "scene", None)
        if scene is not None:
            cell = self.win._work_cell() if hasattr(self.win, "_work_cell") else self.cell
            scene.set_placement(robroot(cell), gantry_offset(cell))
            scene.set_robot_pose(list(self.cell.home_deg))

    def _work_cell(self):
        cell = self.cell.model_copy(deep=True)
        if cell.gantry is not None:
            cell.gantry.position_mm = self.e1_mm()
        return cell

    def _changed(self, *_):
        q, e1 = self.joints(), self.e1_mm()
        self.pose_changed.emit(q, e1)
        cell = self._work_cell()
        scene = getattr(self.win, "scene", None)
        if scene is not None and self.isVisible():
            scene.set_placement(robroot(cell), gantry_offset(cell))
            scene.set_robot_pose(q)
        msgs = [f"{n} {v:g}° outside {lo:g} .. {hi:g}"
                for n, v, (lo, hi) in zip(JOINTS, q, self.limits) if not lo <= v <= hi]
        msgs.append(self._collisions(cell, q, e1))
        self.status.setText("<br>".join(msgs))

    def _collisions(self, cell, q, e1) -> str:
        checker = getattr(self.win, "checker", None)
        if checker is None:
            return "Collisions: checker not ready (run Simulate once)"
        axes = {}
        table = getattr(self.win, "_table_axes", None)
        if callable(table):
            try:
                axes.update(table(cell))
            except Exception:
                pass
        else:
            axes.update(dict.fromkeys(table_axis_names(cell), 0.0))
        if cell.gantry is not None:
            axes[cell.gantry.axis] = e1
        try:
            hits = checker.check(q, axes)
        except Exception as e:      # noqa: BLE001 - shown, not raised, while typing
            return f"Collisions: check failed ({e})"
        if not hits:
            return "<span style='color:#188038'>No collisions at this pose</span>"
        pairs = ", ".join(f"{a} / {b}" for a, b in sorted(hits))
        return f"<span style='color:#d93025'>Collision: {pairs}</span>"
