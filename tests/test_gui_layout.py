"""Window layout: sidebar/3D view splitter, sidebar width, start size, short mode labels."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QSettings, Qt
from PySide6.QtWidgets import QApplication, QPushButton, QSplitter

from coldspray.gui import app as gui_app
from coldspray.gui.steps.toolpath import short_label


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
    assert gui_app.fit_size(2560, 1400) == (1440, 900)
    w, h = gui_app.fit_size(1280, 680)
    assert w <= 1280 and h <= 680


def test_mode_combo_labels(win):
    combo = win.mode
    assert [short_label(combo.itemText(i)) for i in range(combo.count())] == ["Layers", "Revolve", "Surface"]
    assert all(combo.itemData(i, Qt.ToolTipRole) for i in range(combo.count()))


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
        scroll = win.panels.widget(i).scroll
        assert scroll.widget().minimumSizeHint().width() <= scroll.viewport().width() + 20


def test_window_minimum(win):
    assert (win.minimumWidth(), win.minimumHeight()) == gui_app.WIN_MIN


def test_next_buttons_pinned(win):
    """Each step's "Next: ..." button sits in a footer below the scroll area, not inside it."""
    pages = [win.panels.widget(i) for i in range(win.panels.count())]
    pinned = [p for p in pages if hasattr(p, "footer")]
    assert len(pinned) >= 4
    for page in pinned:
        btn = page.footer.findChild(QPushButton)
        assert btn.text().startswith("Next:")
        assert not page.scroll.widget().isAncestorOf(btn)
    assert all(p.scroll.horizontalScrollBarPolicy() == Qt.ScrollBarAlwaysOff for p in pages)


def test_splitter_width_saved(win):
    win.splitter.setSizes([520, 10_000])
    win._save_split()
    assert int(win.ui_settings.value(gui_app.SIDEBAR_KEY)) == win.splitter.sizes()[0]
