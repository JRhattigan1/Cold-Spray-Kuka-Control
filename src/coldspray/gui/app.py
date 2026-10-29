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
    QApplication, QFrame, QHBoxLayout, QMainWindow, QMenu, QPushButton, QScrollArea, QSplitter, QStackedWidget,
    QToolButton,
    QVBoxLayout, QWidget,
)
from pyvistaqt import QtInteractor

from coldspray.cad.part import Substrate
from coldspray.cell import robroot, table_axis_names
from coldspray.gui import icons, job_menu, plan_jobs, style, theme, undo, view_toolbar, widgets
from coldspray.gui.progress import JobProgress
from coldspray.gui.stepper import EXPORT, PART, POSITION, SIMULATE, TOOLPATH, StepBar
from coldspray.gui.steps import ExportStep, PartStep, PositionStep, SimulateStep, ToolpathStep
from coldspray.gui.widgets import ModeToggle, wrap_label
from coldspray.sim.part_view import PartView
from coldspray.sim.scene import gantry_offset, load_assets, make_gun
from coldspray.sim.spacemouse import SpaceMouse
from coldspray.sim.viewer import Scene, fly

PANEL_WIDTH = 460      # sidebar start width (logical px: Qt 6 scales these by the screen's DPI)
PANEL_MIN = 400        # narrowest the splitter lets the sidebar go before its forms clip
VIEW_MIN = 480         # narrowest 3D view


WIN_MIN = (1100, 680)  # smallest window the sidebar + 3D view still work in
SIDEBAR_KEY = "layout/sidebar_width"


def fit_size(avail_w: int, avail_h: int, want=(1440, 900)) -> tuple[int, int]:
    """Start size: the wanted size, shrunk to 92 % of a smaller screen's available area."""
    return min(want[0], int(avail_w * 0.92)), min(want[1], int(avail_h * 0.92))


class MainWindow(QMainWindow, PartStep, PositionStep, ToolpathStep, SimulateStep, ExportStep):
    def __init__(self, settings=None):
        """settings: QSettings for the saved UI mode (default: widgets.settings(); tests pass their own)."""
        super().__init__()
        self.setWindowTitle("Cold Spray KUKA Control")
        self.ui_settings = settings if settings is not None else widgets.settings()
        self.setMinimumSize(*WIN_MIN)
        screen = QApplication.primaryScreen()
        avail = screen.availableGeometry() if screen else None
        w, h = fit_size(avail.width(), avail.height()) if avail else (1440, 900)
        self.resize(max(w, WIN_MIN[0]), max(h, WIN_MIN[1]))
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

        # Planning (Generate, Auto position) runs on its own worker, so a collision check doesn't hold it up
        self.plan_pool = ThreadPoolExecutor(max_workers=1)
        self.plan_job = None
        self.plan_cache = plan_jobs.new_cache()
        self.plan_timer = QTimer(self, interval=100, timeout=lambda: plan_jobs.poll(self))

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
        self.cell_btn = QToolButton(text="Cell", toolTip="TOOL_DATA / BASE_DATA as on the KUKA pendant")
        self.cell_btn.clicked.connect(self.open_cell_dialog)
        self.stepbar.chip_area.addWidget(self.cell_btn)
        self.cell_dialog = None

        # Left: one scrolling panel per step (built in order: later panels use earlier widgets)
        self.panels = QStackedWidget()
        self.sidebar = QFrame()             # floating glass card around the panels
        self.sidebar.setObjectName("sidebar")
        self.sidebar.setMinimumWidth(PANEL_MIN)
        side = QVBoxLayout(self.sidebar)
        side.setContentsMargins(6, 6, 6, 6)
        side.addWidget(self.panels)
        self.stepbar.setObjectName("glassHeader")
        for build in (self._build_part_panel, self._build_position_panel, self._build_toolpath_panel,
                      self._build_simulate_panel, self._build_export_panel):
            self.panels.addWidget(self._page(build()))
        # The sidebar never gets narrower than its widest panel needs (plus scroll bar and card margins)
        need = max(self.panels.widget(i).scroll.widget().minimumSizeHint().width()
                   for i in range(self.panels.count()))
        bar = self.style().pixelMetric(self.style().PixelMetric.PM_ScrollBarExtent)
        self.sidebar.setMinimumWidth(max(PANEL_MIN, need + bar + 12))

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

        # Sidebar | 3D view share the body through a splitter: drag the gap to widen the panel;
        # on window resize the 3D view takes the change, the sidebar keeps its width.
        self.views.setMinimumWidth(VIEW_MIN)
        body = QWidget()
        hbox = QHBoxLayout(body)
        hbox.setContentsMargins(14, 10, 14, 14)
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.setHandleWidth(14)        # the gap between the cards doubles as the handle
        self.splitter.setChildrenCollapsible(False)
        self.splitter.addWidget(self.sidebar)
        self.splitter.addWidget(self.views)
        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        hbox.addWidget(self.splitter)
        self.splitter.splitterMoved.connect(self._save_split)
        self._split_set = False       # start widths applied on first show, once the splitter has its size
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
        self._glass()
        self._refresh_steps()
        # Keyboard: Ctrl+1..5 jump to a step, Ctrl+G generates the toolpath
        self.shortcuts = [QShortcut(QKeySequence(f"Ctrl+{n + 1}"), self, lambda n=n: self.go_to(n))
                          for n in (PART, POSITION, TOOLPATH, SIMULATE, EXPORT)]
        self.shortcuts.append(QShortcut(QKeySequence("Ctrl+G"), self, self.generate))
        job_menu.install(self)            # Job menu (save / open / recent), Ctrl+S / Ctrl+O
        undo.install(self)                # Position step undo / redo, Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y
        self.statusBar().showMessage("Starting...")
        QTimer.singleShot(50, self._startup)

    def showEvent(self, event):
        super().showEvent(event)
        if not self._split_set:        # sidebar at PANEL_WIDTH (or its minimum), the 3D view gets the rest
            self._split_set = True
            total = max(self.splitter.width(), self.width() - 28)
            try:
                saved = int(self.ui_settings.value(SIDEBAR_KEY, 0) or 0)
            except (TypeError, ValueError):
                saved = 0
            side = max(saved or PANEL_WIDTH, self.sidebar.minimumWidth())
            side = min(side, max(self.sidebar.minimumWidth(), total - VIEW_MIN - self.splitter.handleWidth()))
            self.splitter.setSizes([side, max(VIEW_MIN, total - side - self.splitter.handleWidth())])

    def _save_split(self, *_):
        """Remember the sidebar width the user dragged to (QSettings)."""
        self.ui_settings.setValue(SIDEBAR_KEY, self.splitter.sizes()[0])

    # --- layout ---------------------------------------------------------------
    def _page(self, inner) -> QWidget:
        """One sidebar page: the step's form scrolling, with its "Next: ..." button pinned below."""
        page = QWidget()
        box = QVBoxLayout(page)
        box.setContentsMargins(0, 0, 0, 0)
        box.setSpacing(6)
        page.scroll = self._scroll(inner)
        box.addWidget(page.scroll, 1)
        lay = inner.layout()
        for i in range(lay.count() if lay else 0):
            w = lay.itemAt(i).widget()
            if isinstance(w, QPushButton) and w.text().startswith("Next:"):
                lay.takeAt(i)
                footer = QWidget()
                row = QVBoxLayout(footer)
                row.setContentsMargins(10, 0, 10, 4)     # matches the form's side margins
                row.addWidget(w)
                box.addWidget(footer)
                page.footer = footer
                break
        return page

    @staticmethod
    def _scroll(inner) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidget(inner)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)   # forms reflow to the width, never clip
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

    def open_cell_dialog(self):
        from coldspray.gui.cell_dialog import CellDialog
        if self.cell_dialog is None:
            self.cell_dialog = CellDialog(self)
            # tool.yaml is re-read by each run: a new active TCP makes Position on out of date
            self.cell_dialog.saved.connect(lambda changed: changed and self._invalidate(POSITION))
        self.cell_dialog.show()
        self.cell_dialog.raise_()

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

    def _glass(self):
        """Liquid-glass chrome: translucent header bar, floating rounded sidebar, capsule toggle."""
        t = style.tokens(style.current_mode())
        self.stepbar.setStyleSheet(
            f"#glassHeader {{ background: {t['panel']}d9; border: 1px solid {t['line']};"
            f" border-radius: 18px; margin: 10px 14px 0 14px; }}")
        self.sidebar.setStyleSheet(
            f"#sidebar {{ background: {t['panel']}e6; border: 1px solid {t['line']}; border-radius: 22px; }}"
            " #sidebar QScrollArea, #sidebar QScrollArea > QWidget,"
            " #sidebar QScrollArea > QWidget > QWidget { background: transparent; }")
        self.splitter.setStyleSheet("QSplitter::handle { background: transparent; }")
        self.mode_toggle.setStyleSheet(
            f"#modeToggle {{ background: {t['soft']}; border: 1px solid {t['line']}; border-radius: 14px; }}"
            " #modeToggle QPushButton { border-radius: 12px; border: none; padding: 4px 14px;"
            " background: transparent; }"
            f" #modeToggle QPushButton:checked {{ background: {t['accent']}; color: #ffffff; }}")

    def _apply_view_theme(self):
        mode = style.current_mode()
        self._glass()
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
        if getattr(self, "_closed", False):
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.assets = a = load_assets(log=self._status)
            if getattr(self, "_closed", False):     # closed while loading (_status pumps events)
                return
            if a.tool.gun:
                for key, n in a.tool.gun.nozzles.items():
                    self.nozzle.addItem(f"{n.label} ({n.length:.0f} mm)", key)
                self.nozzle.setCurrentIndex(self.nozzle.findData(a.tool.gun.nozzle))
                widgets.fit_combo(self.nozzle)
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
                self.sm_part.setToolTip(f"{self.spacemouse.name}\n{self.sm_part.toolTip()}")    # device names are long: tooltip, not label
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
        self._closed = True           # a queued _startup must not build on the closed plotters
        self.play_timer.stop()
        self.poll_timer.stop()
        self.sm_timer.stop()
        if self.spacemouse:
            self.spacemouse.close()
        self.check_id += 1            # let a running check finish without effect
        self.plan_timer.stop()
        self.plan_job = None
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.plan_pool.shutdown(wait=False, cancel_futures=True)
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
