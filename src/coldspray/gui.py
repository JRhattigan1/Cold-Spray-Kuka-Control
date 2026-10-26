"""Cold spray toolpath window: parameters on the left, 3D cell view on the right.

Run with:  python -m coldspray.gui
"""

import sys
import time
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

from coldspray.cell import base_in_robot, robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import PROJECT_ROOT, ProcessConfig, load_process, save_process
from coldspray.pipeline import load_part, plan_part
from coldspray.post.krl import KRLError, to_krl
from coldspray.sim.cell_model import OPTIONAL
from coldspray.sim.scene import gantry_offset, load_assets, make_gun
from coldspray.sim.viewer import Scene

UNITS = {"_mm_s": " mm/s", "_mm": " mm", "_deg": " °", "_s": " s"}


def _label(field: str) -> tuple[str, str]:
    for suffix, unit in UNITS.items():
        if field.endswith(suffix):
            return field[: -len(suffix)].replace("_", " ").capitalize(), unit
    return field.replace("_", " ").capitalize(), ""


def estimate_time_s(path, proc: ProcessConfig) -> float:
    total, on = 0.0, False
    for a, b in zip(path.moves, path.moves[1:]):
        total += np.linalg.norm(b.pose[:3, 3] - a.pose[:3, 3]) / b.speed
        if b.gun_on and not on:
            total += proc.spray_settle_s
        on = b.gun_on
    return total


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cold Spray KUKA Control")
        self.resize(1600, 950)
        self.assets = None
        self.scene = None
        self.part = None        # (name, vertices, faces)
        self.result = None      # (path, proc, cell, robot)
        self.play_timer = QTimer(self, interval=30, timeout=self._tick)

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
        block_btn = QPushButton("Use test block")
        block_btn.clicked.connect(lambda: self._set_part(None))
        for w in (self.part_label, open_btn, block_btn):
            pl.addWidget(w)
        layout.addWidget(part_box)

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
        self.e1 = QDoubleSpinBox()
        self.e1.setDecimals(1)
        self.e1.setSuffix(" mm")
        self.e1.setToolTip("Gantry position for this program (+E1 moves towards the turntable)")
        cform.addRow("Nozzle", self.nozzle)
        cform.addRow("Gantry E1", self.e1)
        layout.addWidget(cell_box)

        show_box = QGroupBox("Show")
        sbox = QVBoxLayout(show_box)
        self.group_checks = {}
        for label, (_, default) in OPTIONAL.items():
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
        scroll.setFixedWidth(360)
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
            self.scene = Scene(self.plotter, a.robot, robroot(a.cell), a.meshes, gantry_offset(a.cell),
                               a.cell.robot_cad_pose_deg, make_gun(a.tool, a.gun_mesh))
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

    def _set_part(self, step_file):
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self._status("Reading part..." if step_file else "Building test block...")
            self.part = load_part(step_file)
            self.part_label.setText(Path(step_file).name if step_file else "test block (built in)")
        except Exception as e:  # bad or unreadable STEP file
            QApplication.restoreOverrideCursor()
            QMessageBox.warning(self, "Could not load part", str(e))
            return
        finally:
            QApplication.restoreOverrideCursor()
        self.generate()

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
            self.scene.set_gun(robot, make_gun(tool, a.gun_mesh))

            cell = a.cell.model_copy(deep=True)
            if cell.gantry:
                cell.gantry.position_mm = self.e1.value()
            self.scene.set_placement(robroot(cell), gantry_offset(cell))

            name, v, f = self.part
            self._status("Slicing and planning...")
            layers, path = plan_part(v, f, proc, name)
            if not path.moves:
                raise ValueError("No toolpath: the part gave no layers (is it thinner than one layer?)")
            self._status("Checking reach...")
            plate = tuple(v[:, :2].max(axis=0) + 10)
            joints = self.scene.set_path(path, base_in_robot(cell), part=(v, f), plate_size=plate,
                                         step=2.0, home=cell.home_deg)
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
        bad = int(np.isnan(joints[:, 0]).sum())
        minutes = estimate_time_s(path, proc) / 60
        self.info.setText(
            f"<b>{name}</b><br>{len(layers)} layers, {len(path.moves)} moves<br>"
            f"Estimated spray time: {minutes:.1f} min<br>"
            + (f"<span style='color:#d93025'>{bad} unreachable points</span>" if bad
               else "<span style='color:green'>All points reachable</span>"))
        self.save_krl_btn.setEnabled(bad == 0)
        self.plotter.render()
        self._status(f"Generated in {time.time() - t0:.1f} s")

    def _save_krl(self):
        path, proc, cell, robot = self.result
        out = PROJECT_ROOT / "output"
        out.mkdir(exist_ok=True)
        fn, _ = QFileDialog.getSaveFileName(self, "Save KRL program", str(out / f"{path.name}.src"),
                                            "KRL source (*.src)")
        if not fn:
            return
        try:
            Path(fn).write_text(to_krl(path, robot, cell, proc), newline="\r\n")
        except KRLError as e:
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
        if self.scene:
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
        self.plotter.close()
        super().closeEvent(event)


def main():
    app = QApplication.instance() or QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
