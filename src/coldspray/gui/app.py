"""Main window: step bar on top, the current step's panel on the left, the 3D view
on the right (part view for step 1, cell view for steps 2-5), status bar below.

Shared state and helpers live here; each step's panel and handlers are a mixin in gui/steps/.
"""

import sys
import time
from concurrent.futures import ThreadPoolExecutor

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QHBoxLayout, QMainWindow, QMenu, QPushButton, QScrollArea, QStackedWidget, QToolButton,
    QVBoxLayout, QWidget,
)
from pyvistaqt import QtInteractor

from coldspray.cad.part import Substrate
from coldspray.cell import robroot, table_axis_names
from coldspray.gui import icons, style, theme, view_toolbar
from coldspray.gui.progress import JobProgress
from coldspray.gui.stepper import EXPORT, PART, POSITION, SIMULATE, TOOLPATH, StepBar
from coldspray.gui.steps import ExportStep, PartStep, PositionStep, SimulateStep, ToolpathStep
from coldspray.gui.widgets import ModeToggle, wrap_label
from coldspray.sim.part_view import PartView
from coldspray.sim.scene import gantry_offset, load_assets, make_gun
from coldspray.sim.spacemouse import SpaceMouse
from coldspray.sim.viewer import Scene, fly

PANEL_WIDTH = 380


class MainWindow(QMainWindow, PartStep, PositionStep, ToolpathStep, SimulateStep, ExportStep):
    def __init__(self, settings=None):
        """settings: QSettings for the saved UI mode (default: widgets.settings(); tests pass their own)."""
        super().__init__()
        self.setWindowTitle("Cold Spray KUKA Control")
        self.resize(1600, 950)
        self.assets = None
        self.scene = None
        self.source = None      # the part as loaded, with the choices made in the Part step
        self.roles: list[str] = []       # per body: "build", "fixture" or "ignore"
        self.part = None        # working part in the cell: ignored bodies left out, placed
        self.cell_key = None    # Part step choices the cell's part was made from
        self.substrate = Substrate()     # None: no plate, the part stands on the faceplate
        self.collisions = []
        self.result = None      # (path, proc, cell, robot)
        self.unreachable = 0
        self.summary_head = self.collision_text = ""
        self.play_timer = QTimer(self, interval=30, timeout=self._tick)

        # Step bar state: the step shown, and steps 3-5 done / out of date for the current result
        self.step = PART
        self._done: set[int] = set()
        self._stale: set[int] = set()
        self._notices: list[QWidget] = []    # "Generate a toolpath first" rows

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

        self.stepbar = StepBar()
        self.stepbar.stepClicked.connect(self.go_to)
        # Simple | Advanced: made before the panels, which register their advanced widgets
        self.mode_toggle = ModeToggle(settings)
        self.stepbar.chip_area.addWidget(self.mode_toggle)
        self.theme_btn = QToolButton()
        self.theme_btn.setToolTip("Light, dark or follow Windows")
        self.theme_btn.setPopupMode(QToolButton.InstantPopup)
        menu = QMenu(self.theme_btn)
        for mode in ("system", "light", "dark"):
            menu.addAction({"system": "Follow Windows", "light": "Light", "dark": "Dark"}[mode],
                           lambda m=mode: self.set_theme(m))
        self.theme_btn.setMenu(menu)
        self.stepbar.chip_area.addWidget(self.theme_btn)

        # Left: one scrolling panel per step (built in order: later panels use earlier widgets)
        self.panels = QStackedWidget()
        self.panels.setFixedWidth(PANEL_WIDTH)
        for build in (self._build_part_panel, self._build_position_panel, self._build_toolpath_panel,
                      self._build_simulate_panel, self._build_export_panel):
            self.panels.addWidget(self._scroll(build()))

        # Right: the 3D views, each with a toolbar row above it (filled by view_toolbar)
        self.views = QStackedWidget()
        self.part_view_bar = QHBoxLayout()
        self.cell_view_bar = QHBoxLayout()
        for bar, plotter in ((self.part_view_bar, self.part_plotter), (self.cell_view_bar, self.plotter)):
            view = QWidget()
            vbox = QVBoxLayout(view)
            vbox.setContentsMargins(0, 0, 0, 0)
            vbox.setSpacing(0)
            vbox.addLayout(bar)
            vbox.addWidget(plotter.interactor, 1)
            self.views.addWidget(view)

        body = QWidget()
        hbox = QHBoxLayout(body)
        hbox.addWidget(self.panels)
        hbox.addWidget(self.views, 1)
        central = QWidget()
        vbox = QVBoxLayout(central)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)
        vbox.addWidget(self.stepbar)
        vbox.addWidget(body, 1)
        self.setCentralWidget(central)
        self.job = JobProgress()          # progress + Cancel for slow jobs
        self.statusBar().addPermanentWidget(self.job)
        self.theme_choice = "system"
        self._refresh_steps()
        # Keyboard: Ctrl+1..5 jump to a step, Ctrl+G generates the toolpath
        self.shortcuts = [QShortcut(QKeySequence(f"Ctrl+{n + 1}"), self, lambda n=n: self.go_to(n))
                          for n in (PART, POSITION, TOOLPATH, SIMULATE, EXPORT)]
        self.shortcuts.append(QShortcut(QKeySequence("Ctrl+G"), self, self.generate))
        self.statusBar().showMessage("Starting...")
        QTimer.singleShot(50, self._startup)

    # --- layout ---------------------------------------------------------------
    @staticmethod
    def _scroll(inner) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidget(inner)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        return scroll

    def _toolpath_notice(self) -> QWidget:
        """'Generate a toolpath first' with a button to step 3, shown while there is no toolpath."""
        box = QWidget()
        row = QHBoxLayout(box)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(wrap_label("Generate a toolpath first"), 1)
        btn = QPushButton("Go to Toolpath")
        btn.clicked.connect(lambda: self.go_to(TOOLPATH))
        row.addWidget(btn)
        self._notices.append(box)
        return box

    # --- steps ----------------------------------------------------------------
    def go_to(self, i: int):
        """Show step `i` (free navigation). Steps 2-5 work in the cell: the part goes in
        as chosen in step 1 (kept as placed if those choices haven't changed)."""
        if i != self.step:
            self._cancel_pick()
        self.step = i
        self.stepbar.set_current(i)
        self.panels.setCurrentIndex(i)
        self.views.setCurrentIndex(0 if i == PART else 1)
        if i != PART:
            self._enter_cell()
        self._refresh_steps()

    def _part_key(self):
        """The Part step choices the cell's part is made from."""
        src = self.source
        return (id(src), tuple(self.roles), src.build_body, src.start_face, src.base_face)

    def _refresh_steps(self):
        """Step bar states from the current state of the work."""
        bar = self.stepbar
        bar.set_state(PART, "done" if self.source is not None else "todo")
        if self.part is None:
            position = "todo"
        else:
            position = "done" if self.cell_key == self._part_key() else "stale"
        bar.set_state(POSITION, position)
        for i in (TOOLPATH, SIMULATE, EXPORT):
            state = "done" if i in self._done else "stale" if i in self._stale else "todo"
            if state == "done" and position == "stale":     # made from older Part choices
                state = "stale"
            bar.set_state(i, state)
        for notice in self._notices:
            notice.setVisible(self.result is None)

    def _mark_done(self, i: int):
        self._done.add(i)
        self._stale.discard(i)
        self._refresh_steps()

    def _invalidate(self, first: int):
        """Steps from `first` on no longer match the settings: done ones become out of date."""
        for i in range(first, EXPORT + 1):
            if i in self._done:
                self._done.discard(i)
                self._stale.add(i)
        self.export_status.clear()
        self._refresh_steps()

    def _reset_steps(self):
        """A new part: steps 3-5 not reached yet."""
        self._done.clear()
        self._stale.clear()
        self._refresh_steps()

    # --- theme ----------------------------------------------------------------
    def set_theme(self, choice: str):
        """"system", "light" or "dark" for the whole window and the 3D views."""
        self.theme_choice = choice
        theme.apply(QApplication.instance(), choice)
        self._apply_view_theme()

    def _apply_view_theme(self):
        mode = style.current_mode()
        self.theme_btn.setIcon(icons.icon("theme", mode))
        self.job.refresh_icons(mode)
        for plotter in (self.plotter, self.part_plotter):
            plotter.set_background(style.view_background(mode))
            plotter.render()
        t = style.tokens(mode)
        self.timeline.set_colours({"accent": t["accent"], "bad": t["bad"], "crash": t["crash"],
                                   "steep": t["steep"], "track": t["soft"], "cursor": t["ink"],
                                   "text": t["muted"]})
        for name, btn in (("step_back", self.controls.prev_btn), ("step_fwd", self.controls.next_btn)):
            btn.setText("")
            btn.setIcon(icons.icon(name, mode))
        view_toolbar.refresh_icons(self, mode)

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
            view_toolbar.install(self)
            self._apply_view_theme()
        finally:
            QApplication.restoreOverrideCursor()

    # --- cell settings shared by the steps ---------------------------------------
    def _work_cell(self):
        """Cell config with the gantry at the chosen E1."""
        cell = self.assets.cell.model_copy(deep=True)
        if cell.gantry:
            cell.gantry.position_mm = self.e1.value()
        return cell

    def _table_axes(self, cell) -> dict:
        names = table_axis_names(cell)
        return dict(zip(names, (self.tilt.value(), self.rotation.value())))

    def _substrate_mesh(self):
        return self.substrate.mesh() if self.substrate else None

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
            out_of_date = "<span style='color:#e37400'>Toolpath out of date: press Generate</span>"
            self.info.setText(f"<b>{self.part.name}</b><br>{out_of_date}")
            self.summary.setText(out_of_date)
        else:
            self.summary.clear()
        self._invalidate(TOOLPATH)
        self.plotter.render()

    # --- SpaceMouse ------------------------------------------------------------
    def _spacemouse_tick(self):
        now = time.time()
        dt, self.sm_last = min(now - self.sm_last, 0.1), now
        pressed = self.spacemouse.poll()
        t, r = self.spacemouse.motion()
        if self.step == PART:                        # Part step: camera only
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
    theme.apply(app)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
