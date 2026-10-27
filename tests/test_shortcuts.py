"""Keyboard shortcuts on the main window: Ctrl+1..5 for the steps, Ctrl+G to generate (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import QApplication

from coldspray.gui.stepper import EXPORT, PART, POSITION, SIMULATE, TOOLPATH


def test_shortcuts(tmp_path):
    app = QApplication.instance() or QApplication([])
    from coldspray.gui.app import MainWindow
    win = MainWindow(QSettings(str(tmp_path / "ui.ini"), QSettings.IniFormat))
    try:
        keys = {s.key().toString() for s in win.findChildren(QShortcut)}
        assert {"Ctrl+1", "Ctrl+2", "Ctrl+3", "Ctrl+4", "Ctrl+5", "Ctrl+G"} <= keys
        for n, step in enumerate((PART, POSITION, TOOLPATH, SIMULATE, EXPORT)):
            win.shortcuts[n].activated.emit()
            assert win.step == step
        assert win.shortcuts[-1].key() == QKeySequence("Ctrl+G")
    finally:
        win.plotter.close()
        win.part_plotter.close()
        win.pool.shutdown(wait=False)
