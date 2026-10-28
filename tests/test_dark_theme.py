import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import pyvista as pv
import pytest
from PySide6.QtWidgets import QApplication

from coldspray.gui import style
from coldspray.gui.view_bar import glass_button_qss
from coldspray.sim.markers import GHOST, GHOST_DARK, GunMarker
from coldspray.sim.viewer import apply_view_background


@pytest.fixture(scope="module")
def app():
    a = QApplication.instance() or QApplication([])
    yield a
    style.apply(a, "light")


def test_background_follows_theme(app):
    pl = pv.Plotter(off_screen=True)
    style.apply(app, "dark")
    apply_view_background(pl)
    assert pl.background_color.hex_rgba[:7].lower() == style.view_background("dark").lower()


def test_glass_qss_reads_current_mode(app):
    style.apply(app, "dark")
    d = glass_button_qss()
    style.apply(app, "light")
    assert d != glass_button_qss()


def test_marker_recolours(app):
    pl = pv.Plotter(off_screen=True)
    m = GunMarker(pl)
    m.apply_theme("dark")
    assert m.color == GHOST_DARK
    m.apply_theme("light")
    assert m.color == GHOST
