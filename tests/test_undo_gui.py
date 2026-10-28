"""Position step undo / redo (off screen): move the part, add a constraint, undo twice, redo."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest

from coldspray.models import PROJECT_ROOT

pytestmark = pytest.mark.skipif(not (PROJECT_ROOT / "assets" / "cad" / "cache").exists(),
                                reason="cell CAD cache not available")


def test_undo_redo_placement_and_constraints(tmp_path, monkeypatch):
    from PySide6.QtCore import QSettings
    from PySide6.QtGui import QShortcut
    from PySide6.QtWidgets import QApplication, QMessageBox
    QApplication.instance() or QApplication([])
    from coldspray.gui.app import MainWindow
    from coldspray.gui.stepper import POSITION
    for kind in ("warning", "critical", "information", "question"):
        monkeypatch.setattr(QMessageBox, kind, staticmethod(lambda *a, **kw: None))
    win = MainWindow(QSettings(str(tmp_path / "ui.ini"), QSettings.IniFormat))
    try:
        keys = {s.key().toString() for s in win.findChildren(QShortcut)}
        assert {"Ctrl+Z", "Ctrl+Shift+Z", "Ctrl+Y"} <= keys
        win._startup()
        win._set_part("test_block")
        win.go_to(POSITION)
        u = win.undo
        start = win.part.placement.copy()
        assert u.current is not None and not u.undo_stack

        win.place["X"].setValue(20.0)            # a burst of spin ticks: one undo step
        win.place["X"].setValue(25.0)
        win.place["A"].setValue(30.0)
        u.commit()
        moved = win.part.placement.copy()
        assert not np.allclose(moved, start) and len(u.undo_stack) == 1

        win.constraints.add("centre", "axis:Z", "table_axis")
        u.commit()
        constrained = win.part.placement.copy()
        assert len(win.constraints.rows) == 1 and len(u.undo_stack) == 2
        assert not np.allclose(constrained, moved)

        assert u.undo()                          # the constraint goes, the move stays
        assert win.constraints.rows == [] and np.allclose(win.part.placement, moved)
        assert win.place["X"].value() == pytest.approx(25.0)
        assert u.undo()                          # back to the start
        assert np.allclose(win.part.placement, start) and win.place["X"].value() == pytest.approx(start[0, 3])
        assert not u.undo() and win.result is None

        assert u.redo()                          # the move again
        assert np.allclose(win.part.placement, moved) and win.constraints.rows == []
        assert u.redo()                          # and the constraint row
        assert len(win.constraints.rows) == 1 and win.constraints.rows[0].kind.currentText() == "centre"
        assert np.allclose(win.part.placement, constrained)
        assert not u.redo() and u.undo_btn.isEnabled() and not u.redo_btn.isEnabled()

        win.roll.setValue(15.0)                  # a new edit drops the redo history
        assert u.undo() and win.roll.value() == 0.0 and u.redo_stack
        win.roll.setValue(10.0)
        u.commit()
        assert not u.redo_stack and win.roll.value() == 10.0
    finally:
        win.close()
