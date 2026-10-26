"""JobProgress: a status-bar widget for slow jobs (label, progress bar, Cancel).

    self.job = JobProgress()
    self.statusBar().addPermanentWidget(self.job)
    self.job.start("Auto position...")              # shows it, busy until update(f)
    ... self.job.update(0.4) ...                      # fraction 0..1, or None = busy
    self.job.finish("Auto position done")           # hides it (text goes to the status bar)

Slow jobs take `progress(fraction)` / `cancel() -> bool` callbacks. Pass
`job.set_fraction` / `job.is_cancelled`: both only touch plain Python attributes,
so they are safe to call from a worker thread. Call `job.poll()` from a QTimer on
the GUI thread to move the bar (or `update()` directly when running on the GUI
thread with `QApplication.processEvents()`).
"""

from PySide6.QtCore import Property, Qt, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QProgressBar, QStatusBar, QToolButton, QWidget

from coldspray.gui import icons, style

_STEPS = 1000
_REPAINT = object()      # update() with no argument: plain QWidget repaint


class JobProgress(QWidget):
    cancelRequested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("jobProgress")
        row = QHBoxLayout(self)
        row.setContentsMargins(4, 0, 4, 0)
        row.setSpacing(8)
        self.label = QLabel()
        self.label.setObjectName("jobLabel")
        self.bar = QProgressBar()
        self.bar.setRange(0, _STEPS)
        self.bar.setTextVisible(False)
        self.bar.setFixedWidth(140)
        self.cancel_btn = QToolButton()
        self.cancel_btn.setObjectName("jobCancel")
        self.cancel_btn.setText("Cancel")
        self.cancel_btn.setToolTip("Stop this job (keeps the best result so far)")
        self.cancel_btn.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.cancel_btn.clicked.connect(self.request_cancel)
        row.addWidget(self.label)
        row.addWidget(self.bar, 0, Qt.AlignVCenter)
        row.addWidget(self.cancel_btn)
        self._cancelled = False
        self._fraction = None          # last value from a (possibly worker-thread) set_fraction
        self.running = False
        self.refresh_icons()
        self.hide()

    def refresh_icons(self, mode: str | None = None):
        """Re-tint the Cancel icon (after a theme switch)."""
        self.cancel_btn.setIcon(icons.icon("cancel", mode or style.current_mode()))

    # --- GUI thread -------------------------------------------------------------------
    def start(self, text: str, cancellable: bool = True):
        self._cancelled, self._fraction, self.running = False, None, True
        self.label.setText(text)
        self.cancel_btn.setVisible(cancellable)
        self.cancel_btn.setEnabled(True)
        self.update(None)
        self.show()

    def update(self, fraction=_REPAINT):  # noqa: A003 - extends QWidget.update
        """Bar to `fraction` (0..1), or None for a busy (moving) bar. No argument: repaint."""
        if fraction is _REPAINT:
            pass
        elif fraction is None:
            self.bar.setRange(0, 0)
        else:
            self.bar.setRange(0, _STEPS)
            self.bar.setValue(round(min(max(float(fraction), 0.0), 1.0) * _STEPS))
        super().update()

    def poll(self):
        """Show the fraction last stored by `set_fraction` (call from a QTimer)."""
        if self.running and self._fraction is not None:
            self.update(self._fraction)

    def finish(self, text: str = ""):
        """Hide; `text` goes to the parent status bar for a few seconds, if there is one."""
        self.running = False
        self.hide()
        bar = self.parent()
        while bar is not None and not isinstance(bar, QStatusBar):
            bar = bar.parent()
        if text and bar is not None:
            bar.showMessage(text, 5000)

    def request_cancel(self):
        if not self._cancelled:
            self._cancelled = True
            self.cancel_btn.setEnabled(False)
            self.label.setText(self.label.text().rstrip(".…") + " - cancelling…")
            self.cancelRequested.emit()

    # --- thread-safe callbacks for jobs -------------------------------------------------
    def set_fraction(self, fraction: float):
        """`progress` callback: store only (thread-safe); `poll()` shows it."""
        self._fraction = float(fraction)

    def is_cancelled(self) -> bool:
        """`cancel` callback (thread-safe)."""
        return self._cancelled

    def _get_cancelled(self) -> bool:
        return self._cancelled

    cancelled = Property(bool, _get_cancelled)
