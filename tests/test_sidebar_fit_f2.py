"""Toolpath / Simulate / Export panels fit a 420 px sidebar (usable ~388 px)."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtWidgets import QApplication, QComboBox


@pytest.fixture(params=["light", "dark"])
def win(request):
    from coldspray.gui import style
    app = QApplication.instance() or QApplication([])
    style.apply(app, request.param)             # the real app always has the stylesheet
    from coldspray.gui.app import MainWindow
    w = MainWindow()
    w._refresh_steps = lambda: None
    yield w
    w.pool.shutdown(wait=False)
    w.close()


@pytest.mark.parametrize("idx,name", [(2, "toolpath"), (3, "simulate"), (4, "export")])
def test_panel_min_width(win, idx, name):
    inner = win.panels.widget(idx).scroll.widget()
    assert inner.minimumSizeHint().width() <= 402, name


def test_mode_is_visible_combo_with_tooltips(win):
    assert isinstance(win.mode, QComboBox) and not win.mode.isHidden()
    assert win.mode.itemData(0, 3)       # Qt.ToolTipRole
