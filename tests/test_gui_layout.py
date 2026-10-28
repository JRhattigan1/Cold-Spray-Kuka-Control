"""Window layout: sidebar/3D view splitter, sidebar width, start size, short mode labels."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication, QSplitter

from coldspray.gui import app as gui_app
from coldspray.gui.steps.toolpath import SegmentedCapsule, short_label


@pytest.fixture(scope="module")
def win(tmp_path_factory):
    QApplication.instance() or QApplication([])

    w = gui_app.MainWindow(QSettings(str(tmp_path_factory.mktemp("ui") / "ui.ini"), QSettings.IniFormat))
    w._refresh_steps = lambda: None
    w.resize(1600, 1000)
    w.show()
    QApplication.processEvents()
    yield w
    w.close()


def test_short_label():
    assert short_label("Layers (build up on the substrate plate)") == "Layers"
    assert short_label("Revolve (coat the outside, table spinning)") == "Revolve"
    assert short_label("Surface") == "Surface"


def test_fit_size():
    assert gui_app.fit_size(2560, 1400) == (1600, 950)
    w, h = gui_app.fit_size(1280, 680)
    assert w <= 1280 and h <= 680


def test_mode_capsule_short_labels(win):
    cap = win.findChild(SegmentedCapsule)
    assert [b.text() for b in cap.buttons] == ["Layers", "Revolve", "Surface"]
    assert all("(" in b.toolTip() for b in cap.buttons)


def test_splitter_shares_space(win):
    sp = win.splitter
    assert isinstance(sp, QSplitter) and sp.widget(0) is win.sidebar and sp.widget(1) is win.views
    assert win.sidebar.width() >= gui_app.PANEL_MIN
    side0 = win.sidebar.width()
    win.resize(1300, 1000)
    QApplication.processEvents()
    assert win.sidebar.width() == side0          # the 3D view takes the resize
    sp.setSizes([side0 + 120, 10_000])
    QApplication.processEvents()
    assert win.sidebar.width() > side0           # user can widen the panel


def test_panels_fit_sidebar(win):
    """No step panel needs more width than the sidebar gives it (nothing clipped on the right)."""
    for i in range(win.panels.count()):
        scroll = win.panels.widget(i)
        assert scroll.widget().minimumSizeHint().width() <= scroll.viewport().width() + 20
