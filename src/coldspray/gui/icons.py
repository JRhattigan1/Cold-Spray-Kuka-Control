"""Stroke icons (24x24, 1.75 px lines) tinted for the theme.

    from coldspray.gui import icons
    button.setIcon(icons.icon("fit", mode))          # mode: "light" | "dark" | "system"
    icons.icon("warning", mode, colour="bad")        # a token name or "#rrggbb"

The SVGs in icons/ draw with `currentColor`; Qt SVG doesn't know it, so the text
is loaded and the colour substituted per theme. Results are cached; call
`clear_cache()` after a theme switch only if you hold on to old QIcons (they keep
their old colour - set the icons again with the new mode).
"""

from functools import lru_cache
from pathlib import Path

from PySide6.QtCore import QByteArray, QRectF, Qt
from PySide6.QtGui import QIcon, QPainter, QPixmap
from PySide6.QtSvg import QSvgRenderer

from coldspray.gui import style

ICON_DIR = Path(__file__).with_name("icons")
NAMES = ("open", "part", "position", "toolpath", "simulate", "export", "fit", "iso", "top", "front", "side",
         "bottom", "section", "follow", "layers", "cell", "play", "pause", "step_back", "step_fwd", "warning",
         "check", "cross", "collision", "save", "settings", "spacemouse", "cancel", "theme")
SIZES = (16, 20, 24, 32, 48)


@lru_cache(maxsize=None)
def svg_text(name: str) -> str:
    path = ICON_DIR / f"{name}.svg"
    if not path.exists():
        raise KeyError(f"no icon {name!r}; have {', '.join(NAMES)}")
    return path.read_text(encoding="utf-8")


def _colour(colour: str, mode: str) -> str:
    return colour if colour.startswith("#") else style.tokens(mode)[colour]


def pixmap(name: str, size: int = 24, colour: str = "#000000", scale: float = 1.0) -> QPixmap:
    """`name` drawn in `colour` (#rrggbb) on a transparent `size` px pixmap."""
    renderer = QSvgRenderer(QByteArray(svg_text(name).replace("currentColor", colour).encode()))
    px = round(size * scale)
    pm = QPixmap(px, px)
    pm.fill(Qt.transparent)
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.Antialiasing)
    renderer.render(painter, QRectF(0, 0, px, px))
    painter.end()
    pm.setDevicePixelRatio(scale)
    return pm


@lru_cache(maxsize=None)
def _icon(name: str, normal: str, disabled: str, active: str) -> QIcon:
    ic = QIcon()
    for s in SIZES:
        for scale in (1.0, 2.0):
            ic.addPixmap(pixmap(name, s, normal, scale), QIcon.Normal, QIcon.Off)
            ic.addPixmap(pixmap(name, s, active, scale), QIcon.Normal, QIcon.On)
            ic.addPixmap(pixmap(name, s, active, scale), QIcon.Active, QIcon.On)
            ic.addPixmap(pixmap(name, s, normal, scale), QIcon.Active, QIcon.Off)
            ic.addPixmap(pixmap(name, s, disabled, scale), QIcon.Disabled, QIcon.Off)
            ic.addPixmap(pixmap(name, s, disabled, scale), QIcon.Disabled, QIcon.On)
    return ic


def icon(name: str, mode: str = "system", colour: str = "ink", checked: str = "accent") -> QIcon:
    """QIcon for `name` in theme `mode`. `colour` / `checked` (shown when a checkable
    button is on): token names ("ink", "muted", "accent", "bad", ...) or "#rrggbb"."""
    mode = style.resolve(mode)
    disabled = style._mix(style.tokens(mode)["muted"], style.tokens(mode)["panel"], 0.6)
    return _icon(name, _colour(colour, mode), disabled, _colour(checked, mode))


def clear_cache():
    _icon.cache_clear()
