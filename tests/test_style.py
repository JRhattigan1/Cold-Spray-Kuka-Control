import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

from coldspray.gui import icons, style


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize("mode", ["light", "dark"])
def test_apply_sets_palette_and_stylesheet(app, mode):
    assert style.apply(app, mode) == mode
    t = style.tokens(mode)
    assert app.palette().color(QPalette.Window) == QColor(t["bg"])
    assert app.palette().color(QPalette.Base) == QColor(t["panel"])
    assert app.palette().color(QPalette.Highlight) == QColor(t["accent"])
    qss = app.styleSheet()
    assert t["accent"] in qss and t["ink"] in qss
    assert "@accent" not in qss and "@ink" not in qss and "@bg" not in qss
    assert style.current_mode(app) == mode


def test_system_mode_resolves(app):
    assert style.resolve("system") in ("light", "dark")
    assert style.apply(app) in ("light", "dark")
    with pytest.raises(ValueError):
        style.resolve("purple")


def test_view_and_meaning_colours():
    assert style.view_background("light") == "#dfe5ea"
    assert style.view_background("dark") == "#252c33"
    m = style.meaning_colours("dark")
    assert set(m) == {"good", "bad", "warn", "crash", "steep", "accent"} and m["accent"] == "#ff7a33"


def test_no_unreplaced_tokens(app):
    import re
    for mode in ("light", "dark"):
        assert not re.findall(r"@[a-z_]+", style.stylesheet(mode))


@pytest.mark.parametrize("mode", ["light", "dark"])
@pytest.mark.parametrize("name", icons.NAMES)
def test_icons_render(app, mode, name):
    ic = icons.icon(name, mode)
    assert not ic.isNull()
    img = ic.pixmap(24, 24).toImage()
    assert any(img.pixelColor(x, y).alpha() > 0 for x in range(24) for y in range(24))


def test_every_svg_has_a_name():
    files = {p.stem for p in icons.ICON_DIR.glob("*.svg")}
    assert files == set(icons.NAMES)
