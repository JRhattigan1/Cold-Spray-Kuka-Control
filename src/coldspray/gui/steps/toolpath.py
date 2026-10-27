"""Step 3 Toolpath: strategy, process parameters, Generate."""

import time
from typing import Literal, get_args, get_origin

import numpy as np
from pydantic import ValidationError
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QFrame, QComboBox, QDoubleSpinBox, QFormLayout, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton,
    QSpinBox, QVBoxLayout, QWidget,
)

from coldspray.cad.part import build_mesh, other_mesh
from coldspray.cell import robroot
from coldspray.gui.stepper import SIMULATE, TOOLPATH
from coldspray.gui import plan_jobs, style, widgets
from coldspray.gui.widgets import Chip, Section, primary_button, wrap_label
from coldspray.kinematics.robot import Robot
from coldspray.models import ProcessConfig, field_level, load_process, save_process
from coldspray.pipeline import MODES, plan_part
from coldspray.planner.gun import wrist_side
from coldspray.sim.scene import gantry_offset, make_gun

UNITS = {"_mm_s": " mm/s", "_mm": " mm", "_deg": " °", "_s": " s"}


def _label(field: str) -> tuple[str, str]:
    for suffix, unit in UNITS.items():
        if field.endswith(suffix):
            return field[: -len(suffix)].replace("_", " ").capitalize(), unit
    return field.replace("_", " ").capitalize(), ""


def make_process_box(name: str, info, value, on_change) -> QWidget:
    """Editor widget for one ProcessConfig field: Literal -> combo, bool -> check box,
    int / float -> spin box, str -> line edit (kept out of the form)."""
    ann = info.annotation
    if get_origin(ann) is Literal:
        box = QComboBox()
        for choice in get_args(ann):
            box.addItem(str(choice).replace("_", " "), choice)
        box.setCurrentIndex(box.findData(value))
        box.currentIndexChanged.connect(on_change)
    elif ann is bool:
        box = QCheckBox()
        box.setChecked(bool(value))
        box.toggled.connect(on_change)
    elif ann is str:
        box = QLineEdit(str(value))
        box.editingFinished.connect(on_change)
    else:
        _, unit = _label(name)
        if ann is int:
            box = QSpinBox()
            box.setRange(0, 1000)
        else:
            box = QDoubleSpinBox()
            box.setDecimals(2)
            box.setRange(-360.0 if name.endswith("_deg") else 0.0, 100000.0)
        box.setValue(value)
        box.setSuffix(unit)
        box.setKeyboardTracking(False)
        box.valueChanged.connect(on_change)
    box.setToolTip(info.description)
    return box


def box_value(box: QWidget):
    if isinstance(box, QComboBox):
        return box.currentData()
    if isinstance(box, QCheckBox):
        return box.isChecked()
    if isinstance(box, QLineEdit):
        return box.text()
    return box.value()


def box_differs(box: QWidget, default) -> bool:
    value = box_value(box)
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return not np.isclose(value, default)
    return value != default


def advanced_row(text: str, box: QWidget, default) -> QWidget:
    """One advanced process field as a single row (label, editor, "changed" chip), registered
    with the Simple/Advanced mode: hidden in Simple unless it differs from process.yaml."""
    row = QWidget()
    h = QHBoxLayout(row)
    h.setContentsMargins(0, 0, 0, 0)
    label = QLabel(text)
    label.setMinimumWidth(140)
    chip = Chip()
    h.addWidget(label)
    h.addWidget(box, 1)
    h.addWidget(chip)
    return widgets.advanced(row, changed=lambda: box_differs(box, default), chip=chip)


PLANES = ("XY", "XZ", "YZ")


def split_plane(spec: str) -> tuple[str, float]:
    """"NAME" / "datum:NAME" / "NAME:offset" -> (NAME or "" , offset mm)."""
    name, offset = spec.removeprefix("datum:"), 0.0
    head, _, tail = name.rpartition(":")
    try:
        offset, name = float(tail), head
    except ValueError:
        pass
    return (name if name in PLANES else ""), offset


def join_plane(name: str, offset: float) -> str:
    """Inverse of split_plane: "" for no plane, else "NAME" or "NAME:offset"."""
    if not name:
        return ""
    return f"{name}:{offset:g}" if offset else name


def plane_row(edit: QLineEdit, on_change) -> QWidget:
    """Datum plane combo + offset spin that write the turn_plane string into `edit`."""
    row = QWidget()
    h = QHBoxLayout(row)
    h.setContentsMargins(0, 0, 0, 0)
    combo, spin = QComboBox(), QDoubleSpinBox()
    combo.addItem("none", "")
    for n in PLANES:
        combo.addItem(f"datum {n}", n)
    spin.setRange(-100000.0, 100000.0)
    spin.setDecimals(2)
    spin.setSuffix(" mm")
    spin.setKeyboardTracking(False)
    name, offset = split_plane(edit.text())
    combo.setCurrentIndex(combo.findData(name))
    spin.setValue(offset)
    spin.setEnabled(bool(name))

    def changed(*_):
        spin.setEnabled(bool(combo.currentData()))
        edit.setText(join_plane(combo.currentData(), spin.value()))
        on_change()

    combo.currentIndexChanged.connect(changed)
    spin.valueChanged.connect(changed)
    combo.setToolTip(edit.toolTip())
    h.addWidget(QLabel("Turn plane"))
    h.addWidget(combo, 1)
    h.addWidget(spin)
    row.combo, row.spin = combo, spin
    return row


def estimate_time_s(path, proc: ProcessConfig) -> float:
    """Rough program time: robot moves at their speed over the part, plus spray settling.
    Pure table rotations count at their speed over the part too; dwells add their wait."""
    total, on = sum(m.dwell_s for m in path.moves), False
    for a, b in zip(path.moves, path.moves[1:]):
        total += np.linalg.norm(b.pose[:3, 3] - a.pose[:3, 3]) / b.speed
        if b.gun_on and not on:
            total += proc.spray_settle_s
        on = b.gun_on
    return total


def _glass_css(name: str, radius: int = 14) -> str:
    t = style.tokens(style.current_mode())
    return (f"QFrame#{name} {{ background: {t['panel']}; border: 1px solid {t['line']}; border-radius: {radius}px; }}"
            f"QFrame#{name} QLabel {{ background: transparent; }}")


class SegmentedCapsule(QFrame):
    """Glass segmented capsule that mirrors a QComboBox (the combo stays the data API)."""

    def __init__(self, combo: QComboBox):
        super().__init__()
        self.setObjectName("segcap")
        self.combo, self.buttons = combo, []
        lay = QHBoxLayout(self)
        lay.setContentsMargins(3, 3, 3, 3)
        lay.setSpacing(2)
        for i in range(combo.count()):
            b = QPushButton(combo.itemText(i))
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, i=i: combo.setCurrentIndex(i))
            lay.addWidget(b, 1)
            self.buttons.append(b)
        combo.currentIndexChanged.connect(self._sync)
        combo.hide()
        self._sync()

    def _sync(self, *_):
        t = style.tokens(style.current_mode())
        self.setStyleSheet(
            f"QFrame#segcap {{ background: {t['soft']}; border: 1px solid {t['line']}; border-radius: 16px; }}"
            f"QPushButton {{ background: transparent; border: none; border-radius: 13px; padding: 5px 10px; color: {t['muted']}; }}"
            f"QPushButton:checked {{ background: {t['accent']}; color: #ffffff; font-weight: 600; }}")
        for i, b in enumerate(self.buttons):
            b.setChecked(i == self.combo.currentIndex())


class StatTiles(QWidget):
    """Summary as glass stat tiles; keeps the QLabel.setText API used by generate."""

    def __init__(self):
        super().__init__()
        self._lay = QHBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 0)
        self._tiles = {}
        self._text = ""
        for key, cap in (("moves", "Moves"), ("time", "Time"), ("layers", "Layers")):
            f = QFrame()
            f.setObjectName("tile_" + key)
            f.setStyleSheet(_glass_css("tile_" + key))
            v = QVBoxLayout(f)
            v.setContentsMargins(10, 8, 10, 8)
            val, lab = QLabel("-"), QLabel(cap)
            val.setStyleSheet(f"font-size: 18px; font-weight: 600; color: {style.tokens(style.current_mode())['accent']};")
            lab.setStyleSheet(f"color: {style.tokens(style.current_mode())['muted']};")
            v.addWidget(val)
            v.addWidget(lab)
            self._lay.addWidget(f)
            self._tiles[key] = (f, val)
        self.setVisible(False)

    def text(self):
        return self._text

    def setText(self, text: str):
        import re
        self._text = text
        pats = {"moves": r"([\d,]+)\s+moves", "time": r"≈\s*([\d.]+)\s*min", "layers": r"([\d,]+)\s+(?:layers|coats)"}
        shown = False
        for key, pat in pats.items():
            m = re.search(pat, text)
            frame, val = self._tiles[key]
            frame.setVisible(bool(m) or key != "layers")
            val.setText((m.group(1) + (" min" if key == "time" else "")) if m else "-")
            shown = shown or bool(m)
        self.setVisible(bool(text))


class ToolpathStep:
    def _build_toolpath_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)

        plan_box = self.strategy_section = Section("Strategy")
        pform = QFormLayout()
        self.mode = QComboBox()
        for key, text in MODES.items():
            self.mode.addItem(text, key)
        self.mode.setToolTip("Layers: slice the part and build it up layer by layer.\n"
                             "Revolve: spin the turntable and coat the outside of a round part.")
        self.mode.currentIndexChanged.connect(self._mode_changed)
        self.coats = QSpinBox()
        self.coats.setRange(1, 500)
        self.coats.setToolTip("Revolve: number of complete coats; each backs off by the layer thickness")
        pform.addRow("Mode", SegmentedCapsule(self.mode))
        pform.addRow(self.mode)
        pform.addRow("Coats", self.coats)
        plan_box.add(pform)
        layout.addWidget(plan_box)

        proc_box = self.process_section = Section("Process")
        self.advanced_section = Section("Advanced", open=False)
        self.process_form, advanced_form = QFormLayout(), QFormLayout()
        for f in (self.process_form, advanced_form):
            f.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            f.setHorizontalSpacing(12)
            f.setVerticalSpacing(8)
        self.fields = {}
        self.process_defaults = proc = load_process()
        for name, info in ProcessConfig.model_fields.items():
            box = make_process_box(name, info, getattr(proc, name), self._process_changed)
            self.fields[name] = box
            if isinstance(box, QLineEdit):
                if name == "turn_plane":
                    self.plane_row = plane_row(box, self._process_changed)
                    advanced_form.addRow(self.plane_row)
                continue            # free text (e.g. a datum plane name): set elsewhere, not in the form
            text, _ = _label(name)
            if field_level(name) == "basic":
                self.process_form.addRow(text, box)
            else:
                advanced_form.addRow(advanced_row(text, box, getattr(proc, name)))
        proc_box.add(self.process_form)
        self.advanced_section.add(advanced_form)
        proc_box.setObjectName("procglass")
        proc_box.setAttribute(Qt.WA_StyledBackground, True)
        proc_box.setStyleSheet(_glass_css("procglass", 16).replace("QFrame#", "QWidget#", 1))
        layout.addWidget(proc_box)
        layout.addWidget(self.advanced_section)
        widgets.on_mode_changed(lambda _mode: self._sync_advanced())
        self._process_changed()

        self.generate_btn = primary_button("Generate toolpath", self.generate)
        _t = style.tokens(style.current_mode())
        self.generate_btn.setMinimumHeight(40)
        self.generate_btn.setStyleSheet(
            f"QPushButton {{ background: {_t['accent']}; color: #ffffff; border: none; border-radius: 20px; padding: 8px 18px; font-size: 14px; }}"
            f"QPushButton:disabled {{ background: {_t['soft']}; color: {_t['muted']}; }}")
        self.save_settings_btn = QPushButton("Save as default")
        self.save_settings_btn.setToolTip("Write these process values to config/process.yaml")
        self.save_settings_btn.clicked.connect(self._save_settings)
        layout.addWidget(self.generate_btn)
        layout.addWidget(self.save_settings_btn)
        self.summary = StatTiles()
        layout.addWidget(self.summary)

        layout.addWidget(primary_button("Next: Simulate  →", lambda: self.go_to(SIMULATE)))
        layout.addStretch(1)
        self._mode_changed()
        return inner

    def _mode_changed(self):
        mode = self.mode.currentData()
        revolve = mode == "revolve"
        self.coats.setEnabled(revolve)          # surface coats come from the process (surface_coats)
        self.rotation.setEnabled(not revolve)
        part = getattr(self, "source", None)    # the Part step's live choice, not the stale cell copy
        if mode == "surface" and part is not None and not getattr(part, "coat_faces", None):
            self._status("Surface mode: pick the faces to coat in the Part step first")
        self._show_placement()
        self._enter_setup()

    def _process_changed(self, *_):
        """An edited process value: re-sync the advanced rows; a toolpath is now out of date."""
        self._sync_advanced()
        row = getattr(self, "plane_row", None)
        if row is not None:             # the plane only matters when passes turn beyond it
            row.setVisible(box_value(self.fields["turn_mode"]) == "beyond_plane")
        if getattr(self, "result", None) is not None:
            self._invalidate(TOOLPATH)

    def _sync_advanced(self):
        """Show how many advanced values differ from process.yaml and apply the Simple/Advanced mode."""
        # free-text fields (QLineEdit, e.g. turn_plane) have no row in the form: don't count them
        changed = [n for n, box in self.fields.items()
                   if field_level(n) != "basic" and not isinstance(box, QLineEdit)
                   and box_differs(box, getattr(self.process_defaults, n))]
        self.advanced_section.set_chip(f"{len(changed)} changed" if changed else "", "warn")
        widgets.refresh()
        # Simple mode: the Advanced section only shows (and opens) when it holds a changed value
        simple = widgets.is_simple()
        self.advanced_section.setVisible(not simple or bool(changed))
        if simple and changed:
            self.advanced_section.set_open(True)

    def read_process(self) -> ProcessConfig:
        return ProcessConfig(**{name: box_value(box) for name, box in self.fields.items()})

    def generate(self):
        if self.scene is None or self.part is None:
            return
        try:
            proc = self.read_process()
        except ValidationError as e:
            QMessageBox.warning(self, "Invalid parameter", str(e))
            return
        if plan_jobs.busy(self):
            return
        t0 = time.time()
        a = self.assets
        tool = a.tool.model_copy(deep=True)
        if tool.gun:
            tool.gun.nozzle = self.nozzle.currentData()
        robot = Robot(a.robot.cfg, tool)
        gun = make_gun(tool, a.gun_mesh)
        cell = self._work_cell()
        mode = self.mode.currentData()
        kw = dict(mode=mode, tilt_deg=self.tilt.value(), rotation_deg=self.rotation.value(),
                  coats=self.coats.value(), cell=cell, substrate=self.substrate, roll_deg=self.roll.value(),
                  wrist=wrist_side(robot.tool))
        part = plan_jobs.snapshot(self.part)       # values read here: the worker touches no widget
        key = plan_jobs.plan_key(id(self.part), part, proc, kw)
        cache = getattr(self, "plan_cache", None)

        def work():
            return plan_jobs.cached_plan(cache, key, lambda: plan_part(part, proc, **kw))

        self.check_id += 1                 # an older collision check / solve no longer applies
        self.poll_timer.stop()
        self.check_future = self.solve_future = None
        self._status("Planning...")
        plan_jobs.run(self, "Planning...", work,
                      lambda out, err: self._planned(out, err, t0, proc, cell, robot, gun, mode),
                      abandon_on_cancel=True)

    def _planned(self, out, err, t0, proc, cell, robot, gun, mode):
        """GUI thread: the plan is back; reach check (IK), scene, summary, collision check."""
        a = self.assets
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            if err is not None:
                raise err
            summary, path = out
            if not path.moves:
                raise ValueError("No toolpath: the part gave no layers (is it thinner than one layer?)")
            self.job.label.setText("Checking reach...")
            self._status("Checking reach...")
            self.scene.set_gun(robot, gun)
            self.scene.set_placement(robroot(cell), gantry_offset(cell))
            part_mesh, fixture_mesh = build_mesh(self.part), other_mesh(self.part)
            substrate_mesh = self._substrate_mesh()
            joints = self.scene.set_path(path, cell, part=part_mesh, substrate=substrate_mesh,
                                         fixture=fixture_mesh, step=2.0,
                                         home=cell.home_deg, angle_step=5.0,
                                         gun_limit_deg=cell.gun_max_down_deg)
            self.result = (path, proc, cell, robot)
            self.result_mode = mode
        except Exception as e:
            QApplication.restoreOverrideCursor()
            self.job.finish("Generate failed")
            QMessageBox.warning(self, "Could not generate toolpath", str(e))
            self._status("Generate failed")
            return
        QApplication.restoreOverrideCursor()
        self.job.finish()
        name = self.part.name

        self.slider.blockSignals(True)
        self.slider.setRange(0, self.scene.n_points - 1)
        self.slider.setValue(self.scene.first_reachable())
        self.slider.blockSignals(False)
        self.unreachable = int(np.isnan(joints[:, 0]).sum())
        minutes = estimate_time_s(path, proc) / 60
        self.collision_text = "collision check off" if not self.check_box.isChecked() else "checking collisions..."
        self.summary_head = (f"<b>{name}</b><br>{summary}, {len(path.moves)} moves<br>"
                             f"Estimated time: {minutes:.1f} min<br>")
        self.summary.setText(f"{summary} · {len(path.moves)} moves · ≈ {minutes:.1f} min")
        self.times, self.collision_failed = None, None   # new samples: rebuild timeline and checks
        self._update_info()
        self._show_program()
        self.save_krl_btn.setEnabled(self.unreachable == 0)
        self.plotter.render()
        self._status(f"Generated in {time.time() - t0:.1f} s")
        self._invalidate(SIMULATE)         # a new toolpath: check and save it again
        self._mark_done(TOOLPATH)
        if self.check_box.isChecked() and a.meshes is not None:
            self._start_collision_check(robot, gun, cell, part_mesh, substrate_mesh, fixture_mesh, joints)
        else:
            self._mark_done(SIMULATE)

    def _save_settings(self):
        try:
            proc = self.read_process()
        except ValidationError as e:
            QMessageBox.warning(self, "Invalid parameter", str(e))
            return
        save_process(proc)
        self.process_defaults = proc
        self._process_changed()
        self._status("Saved process settings to config/process.yaml")
