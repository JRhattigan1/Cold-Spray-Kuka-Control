"""Application look: light / dark palettes, Fusion style and one app stylesheet.

    from coldspray.gui import style
    mode = style.apply(app, "system")          # -> "light" or "dark"
    plotter.set_background(style.view_background(mode))

Colours are tokens (`PALETTES`); orange (`accent`) is kept for the current step,
primary buttons and the toolpath. good / warn / bad / crash / steep keep their
check meanings. Widgets pick up their look from objectNames and Qt properties:

- step pills: QFrame#pill with properties `state` (done|stale|todo) and `current`
  (bool), child QLabels #pillText / #pillMark, separators #pillSep, #brand, #stepBar
- folding sections: QFrame#sectionHeader, QPushButton#sectionToggle, QFrame#sectionBody
- chips: QLabel#chip with property `kind` (good|bad|warn|crash|neutral)
  (also accepted: any QLabel with property `chip` = kind)
- notes: QLabel with property `note` = True (muted, small)
- primary buttons: QPushButton with property `primary` = True
- toggled tool buttons in the view toolbar: QToolButton:checked (accent outline)

After changing a property at run time, re-polish: `style.repolish(widget)`.
"""

from PySide6.QtGui import QColor, QFont, QFontDatabase, QGuiApplication, QPalette
from PySide6.QtWidgets import QApplication, QStyleFactory

MODES = ("system", "light", "dark")

PALETTES = {
    "light": {
        "bg": "#f5f6f8", "panel": "#ffffff", "line": "#d9dee4", "soft": "#eceff3",
        "ink": "#1d2329", "muted": "#5b6672",
        "accent": "#e8590c", "accent_soft": "#fde8dc",
        "good": "#1f8a4c", "warn": "#b7791f", "bad": "#c53030", "crash": "#b8326b", "steep": "#d4a017",
        "view": "#dfe5ea", "view_line": "#9aa7b3",
    },
    "dark": {
        "bg": "#14181c", "panel": "#1b2026", "line": "#2e353d", "soft": "#232a31",
        "ink": "#e6e9ed", "muted": "#9aa5b1",
        "accent": "#ff7a33", "accent_soft": "#3a2418",
        "good": "#4cc27f", "warn": "#e0a23a", "bad": "#f06a6a", "crash": "#e06aa0", "steep": "#e8c04a",
        "view": "#252c33", "view_line": "#5d6a76",
    },
}

SPACE = (4, 8, 12)          # spacing scale, px
CONTROL_H = 26              # height of fields, buttons, combos
CHIP_KINDS = ("good", "bad", "warn", "crash", "neutral")

_BODY_FONTS = ("Barlow", "Segoe UI")
_DISPLAY_FONTS = ("Barlow Semi Condensed", "Barlow", "Segoe UI Semibold", "Segoe UI")
_MONO_FONTS = ("JetBrains Mono", "Consolas", "Courier New")


def resolve(mode: str = "system") -> str:
    """"light" or "dark". "system" follows Windows (Qt >= 6.5), else light."""
    if mode in ("light", "dark"):
        return mode
    if mode != "system":
        raise ValueError(f"mode must be one of {MODES}")
    try:
        from PySide6.QtCore import Qt
        if QGuiApplication.instance() is not None:
            if QGuiApplication.styleHints().colorScheme() == Qt.ColorScheme.Dark:
                return "dark"
    except AttributeError:          # Qt < 6.5: no colorScheme()
        pass
    return "light"


def tokens(mode: str = "system") -> dict:
    """Colour tokens (hex strings) for `mode`."""
    return dict(PALETTES[resolve(mode)])


def view_background(mode: str = "system") -> str:
    """Background for the 3D views (pyvista `plotter.set_background(...)`)."""
    return PALETTES[resolve(mode)]["view"]


def meaning_colours(mode: str = "system") -> dict:
    """Check colours for the viewer: good, bad, warn, crash, steep, accent (toolpath)."""
    t = PALETTES[resolve(mode)]
    return {k: t[k] for k in ("good", "bad", "warn", "crash", "steep", "accent")}


def _first_family(candidates) -> str:
    have = set(QFontDatabase.families())
    return next((f for f in candidates if f in have), candidates[-1])


def fonts() -> dict:
    """Font families in use: body, display (headers), mono (numbers). Needs a QGuiApplication."""
    return {"body": _first_family(_BODY_FONTS), "display": _first_family(_DISPLAY_FONTS),
            "mono": _first_family(_MONO_FONTS)}


def mono_font(point_size: float | None = None) -> QFont:
    font = QFont(fonts()["mono"])
    if point_size:
        font.setPointSizeF(point_size)
    font.setStyleHint(QFont.Monospace)
    return font


def _mix(a: str, b: str, t: float) -> str:
    """`t` of colour a over b (like CSS color-mix(a t, b))."""
    ca, cb = QColor(a), QColor(b)
    return QColor(round(ca.red() * t + cb.red() * (1 - t)), round(ca.green() * t + cb.green() * (1 - t)),
                  round(ca.blue() * t + cb.blue() * (1 - t))).name()


def palette(mode: str = "system") -> QPalette:
    t = PALETTES[resolve(mode)]
    c = {k: QColor(v) for k, v in t.items()}
    pal = QPalette()
    roles = {
        QPalette.Window: c["bg"], QPalette.WindowText: c["ink"], QPalette.Base: c["panel"],
        QPalette.AlternateBase: c["soft"], QPalette.Text: c["ink"], QPalette.Button: c["soft"],
        QPalette.ButtonText: c["ink"], QPalette.ToolTipBase: c["panel"], QPalette.ToolTipText: c["ink"],
        QPalette.PlaceholderText: c["muted"], QPalette.Highlight: c["accent"],
        QPalette.HighlightedText: QColor("#ffffff"), QPalette.Link: c["accent"], QPalette.BrightText: c["bad"],
        QPalette.Light: QColor(_mix(t["panel"], t["line"], 0.5)), QPalette.Midlight: c["soft"],
        QPalette.Mid: c["line"], QPalette.Dark: c["muted"], QPalette.Shadow: QColor("#000000"),
    }
    for role, colour in roles.items():
        pal.setColor(role, colour)
    off = c["muted"]
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        pal.setColor(QPalette.Disabled, role, off)
    pal.setColor(QPalette.Disabled, QPalette.Base, c["soft"])
    pal.setColor(QPalette.Disabled, QPalette.Highlight, c["line"])
    return pal


def stylesheet(mode: str = "system") -> str:
    """The application QSS for `mode`."""
    t = dict(PALETTES[resolve(mode)])
    t.update(fonts())
    t.update(_image_paths(resolve(mode)))
    t["h"] = CONTROL_H
    t["hi"] = CONTROL_H - 2                         # inner height (minus 1 px borders)
    t["accent_hover"] = _mix(t["accent"], "#000000" if resolve(mode) == "light" else "#ffffff", 0.88)
    t["soft_hover"] = _mix(t["line"], t["soft"], 0.5)
    t["field_bg"] = t["bg"] if resolve(mode) == "light" else _mix(t["bg"], t["panel"], 0.5)
    for k in ("good", "bad", "warn", "crash"):
        t[f"{k}_border"] = _mix(t[k], t["line"], 0.45)
    t["chip_bg"] = t["panel"]
    qss = _QSS
    for key in sorted(t, key=len, reverse=True):     # longest first: @accent_soft before @accent
        qss = qss.replace("@" + key, str(t[key]))
    return qss


_QSS = """
* { font-family: "@body"; font-size: 9.5pt; }
QMainWindow, QDialog { background: @bg; color: @ink; }
QWidget { color: @ink; }
QMainWindow::separator { background: @line; width: 1px; height: 1px; }
QSplitter::handle { background: @line; }
QSplitter::handle:horizontal { width: 1px; }
QSplitter::handle:vertical { height: 1px; }

/* panels */
QScrollArea { background: @panel; border: none; }
QScrollArea > QWidget > QWidget { background: @panel; }
QFrame[panel="true"], QWidget#panel, QWidget#stepPanel { background: @panel; }
QGroupBox { border: 1px solid @line; border-radius: 6px; margin-top: 14px; padding: 8px 8px 8px 8px;
            background: @panel; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: @muted;
                   font-family: "@display"; font-weight: 600; }
QLabel { background: transparent; }
QLabel[note="true"] { color: @muted; font-size: 8.5pt; }
QLabel[heading="true"] { font-family: "@display"; font-weight: 600; font-size: 11pt; }
QFrame[frameShape="4"], QFrame[frameShape="5"] { color: @line; }   /* HLine / VLine */

/* step bar */
QWidget#stepBar { background: @soft; border-bottom: 1px solid @line; }
QLabel#brand { font-family: "@display"; font-weight: 700; font-size: 10.5pt; color: @ink; }
QLabel#pillSep { color: @view_line; }
QFrame#pill { border: 1px solid @line; border-radius: 11px; background: @panel; min-height: 22px; max-height: 22px; }
QFrame#pill:hover { border-color: @muted; }
QFrame#pill QLabel#pillText { color: @muted; }
QFrame#pill[state="done"] QLabel#pillText, QFrame#pill[state="stale"] QLabel#pillText { color: @ink; }
QFrame#pill[current="true"] { border: 1px solid @accent; background: @accent_soft; }
QFrame#pill[current="true"] QLabel#pillText { color: @ink; font-weight: 600; }
QFrame#pill QLabel#pillMark { font-weight: 700; }
QFrame#pill[state="done"] QLabel#pillMark { color: @good; }
QFrame#pill[state="stale"] QLabel#pillMark { color: @warn; font-size: 7pt; }
/* alternative: pills as QPushButton[step="true"] with state done|current|stale|todo */
QPushButton[step="true"] { border: 1px solid @line; border-radius: 11px; background: @panel; color: @muted;
                           padding: 0px 10px; min-height: 20px; max-height: 20px; }
QPushButton[step="true"][state="done"], QPushButton[step="true"][state="stale"] { color: @ink; }
QPushButton[step="true"][state="current"] { border-color: @accent; background: @accent_soft; color: @ink;
                                            font-weight: 600; }

/* folding sections */
QFrame#sectionHeader { background: @soft; border: 1px solid @line;
                       border-top-left-radius: 6px; border-top-right-radius: 6px; }
QFrame#sectionHeader[open="false"] { border-radius: 6px; }
QFrame#sectionBody { background: @panel; border: 1px solid @line; border-top: none;
                     border-bottom-left-radius: 6px; border-bottom-right-radius: 6px; }
QPushButton#sectionToggle, QToolButton[sectionHeader="true"] {
    border: none; background: transparent; text-align: left; padding: 2px 2px; min-height: 18px;
    font-family: "@display"; font-weight: 600; font-size: 10pt; color: @ink; }
QPushButton#sectionToggle:hover, QToolButton[sectionHeader="true"]:hover { color: @accent; }

/* chips */
QLabel#chip, QLabel[chip] { border: 1px solid @line; border-radius: 9px; background: @chip_bg;
                            padding: 0px 8px; min-height: 16px; max-height: 16px; font-size: 8pt; color: @ink; }
QLabel#chip[kind="good"], QLabel[chip="good"] { color: @good; border-color: @good_border; }
QLabel#chip[kind="bad"], QLabel[chip="bad"] { color: @bad; border-color: @bad_border; }
QLabel#chip[kind="warn"], QLabel[chip="warn"] { color: @warn; border-color: @warn_border; }
QLabel#chip[kind="crash"], QLabel[chip="crash"] { color: @crash; border-color: @crash_border; }
QLabel#chip[kind="neutral"], QLabel[chip="neutral"] { color: @muted; }

/* buttons */
QPushButton { background: @soft; color: @ink; border: 1px solid @line; border-radius: 5px;
              padding: 0px 10px; min-height: @hipx; }
QPushButton:hover { background: @soft_hover; }
QPushButton:pressed { background: @line; }
QPushButton:checked { border-color: @accent; color: @accent; }
QPushButton:disabled { color: @muted; background: @soft; border-color: @line; }
QPushButton:focus { border-color: @muted; }
QPushButton[primary="true"] { background: @accent; border: 1px solid @accent; color: #ffffff; font-weight: 600; }
QPushButton[primary="true"]:hover { background: @accent_hover; border-color: @accent_hover; }
QPushButton[primary="true"]:pressed { background: @accent; }
QPushButton[primary="true"]:disabled { background: @line; border-color: @line; color: @muted; }
QPushButton:flat { background: transparent; border: none; }
QToolButton { background: transparent; color: @ink; border: 1px solid transparent; border-radius: 4px;
              padding: 3px; }
QToolButton:hover { background: @soft; border-color: @line; }
QToolButton:pressed { background: @line; }
QToolButton:checked { border-color: @accent; color: @accent; background: @panel; }
QToolButton:disabled { color: @muted; }
QToolButton::menu-indicator { image: none; width: 0px; }
QWidget#viewToolbar { background: transparent; }
QWidget#viewToolbar QToolButton { background: @panel; border: 1px solid @line; }
QWidget#viewToolbar QToolButton:hover { background: @soft; }
QWidget#viewToolbar QToolButton:checked { border-color: @accent; color: @accent; }

/* fields: mono, right-aligned */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background: @field_bg; color: @ink; border: 1px solid @line; border-radius: 4px;
    padding: 0px 6px; min-height: @hipx; selection-background-color: @accent; selection-color: #ffffff; }
QLineEdit, QSpinBox, QDoubleSpinBox { font-family: "@mono"; font-size: 9pt; }
QSpinBox, QDoubleSpinBox { qproperty-alignment: 'AlignRight|AlignVCenter'; padding-right: 18px; }
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus { border-color: @accent; }
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled { color: @muted; background: @soft; }
QLineEdit:read-only { color: @muted; }
QSpinBox::up-button, QDoubleSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right;
    width: 16px; border: none; border-left: 1px solid @line; border-top-right-radius: 4px; background: transparent; }
QSpinBox::down-button, QDoubleSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right;
    width: 16px; border: none; border-left: 1px solid @line; border-bottom-right-radius: 4px; background: transparent; }
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover { background: @soft; }
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow { image: url(@img_up); width: 8px; height: 8px; }
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow { image: url(@img_down); width: 8px; height: 8px; }
QSpinBox::up-arrow:disabled, QDoubleSpinBox::up-arrow:disabled,
QSpinBox::down-arrow:disabled, QDoubleSpinBox::down-arrow:disabled { image: none; }
QComboBox { padding-right: 20px; }
QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: center right; width: 18px; border: none; }
QComboBox::down-arrow { image: url(@img_down); width: 10px; height: 10px; }
QComboBox QAbstractItemView { background: @panel; color: @ink; border: 1px solid @line;
    selection-background-color: @accent_soft; selection-color: @ink; outline: none; }

/* check boxes */
QCheckBox, QRadioButton { spacing: 6px; background: transparent; }
QCheckBox::indicator, QRadioButton::indicator { width: 14px; height: 14px; border: 1px solid @muted;
    background: @panel; }
QCheckBox::indicator { border-radius: 3px; }
QRadioButton::indicator { border-radius: 8px; }
QCheckBox::indicator:hover, QRadioButton::indicator:hover { border-color: @accent; }
QCheckBox::indicator:checked { background: @accent; border-color: @accent;
    image: url(@img_check); }
QRadioButton::indicator:checked { background: @panel; border: 4px solid @accent; width: 8px; height: 8px; }
QCheckBox::indicator:disabled, QRadioButton::indicator:disabled { border-color: @line; background: @soft; }

/* lists, tables, tabs */
QListView, QTreeView, QTableView { background: @panel; border: 1px solid @line; border-radius: 4px;
    alternate-background-color: @soft; selection-background-color: @accent_soft; selection-color: @ink; }
QHeaderView::section { background: @soft; color: @muted; border: none; border-bottom: 1px solid @line;
    padding: 3px 6px; font-family: "@display"; font-weight: 600; }
QTabWidget::pane { border: 1px solid @line; background: @panel; border-radius: 4px; }
QTabBar::tab { background: @soft; color: @muted; border: 1px solid @line; padding: 4px 12px;
    border-top-left-radius: 5px; border-top-right-radius: 5px; margin-right: 2px; }
QTabBar::tab:selected { background: @panel; color: @ink; border-bottom-color: @panel; }

/* tooltip, status bar, progress */
QToolTip { background: @panel; color: @ink; border: 1px solid @line; padding: 4px 6px; border-radius: 4px; }
QStatusBar { background: @soft; color: @muted; border-top: 1px solid @line; font-size: 8.5pt; }
QStatusBar::item { border: none; }
QStatusBar QLabel { color: @muted; }
QProgressBar { background: @line; border: none; border-radius: 3px; max-height: 6px; min-height: 6px;
    text-align: center; color: transparent; }
QProgressBar::chunk { background: @accent; border-radius: 3px; }

/* menus */
QMenuBar { background: @soft; color: @ink; border-bottom: 1px solid @line; }
QMenuBar::item { padding: 4px 10px; background: transparent; }
QMenuBar::item:selected { background: @line; border-radius: 4px; }
QMenu { background: @panel; color: @ink; border: 1px solid @line; padding: 4px; }
QMenu::item { padding: 4px 22px 4px 22px; border-radius: 4px; }
QMenu::item:selected { background: @accent_soft; color: @ink; }
QMenu::item:disabled { color: @muted; }
QMenu::separator { height: 1px; background: @line; margin: 4px 6px; }

/* sliders */
QSlider::groove:horizontal { height: 4px; background: @line; border-radius: 2px; }
QSlider::sub-page:horizontal { background: @accent; border-radius: 2px; }
QSlider::handle:horizontal { background: @accent; border: 2px solid @panel; width: 12px; height: 12px;
    margin: -6px 0; border-radius: 8px; }
QSlider::handle:horizontal:hover { background: @accent_hover; }
QSlider::groove:vertical { width: 4px; background: @line; border-radius: 2px; }
QSlider::handle:vertical { background: @accent; border: 2px solid @panel; width: 12px; height: 12px;
    margin: 0 -6px; border-radius: 8px; }

/* scroll bars */
QScrollBar:vertical { background: transparent; width: 10px; margin: 0; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 0; }
QScrollBar::handle { background: @line; border-radius: 4px; min-height: 24px; min-width: 24px; margin: 2px; }
QScrollBar::handle:hover { background: @muted; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
"""


_IMAGES = {
    "check": ('<svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 14 14">'
              '<path d="M3 7.2l2.6 2.6L11 4.4" fill="none" stroke="#ffffff" stroke-width="2"'
              ' stroke-linecap="round" stroke-linejoin="round"/></svg>'),
    "up": ('<svg xmlns="http://www.w3.org/2000/svg" width="8" height="8" viewBox="0 0 8 8">'
           '<path d="M1 5.5L4 2.5l3 3" fill="none" stroke="@muted" stroke-width="1.5"'
           ' stroke-linecap="round" stroke-linejoin="round"/></svg>'),
    "down": ('<svg xmlns="http://www.w3.org/2000/svg" width="8" height="8" viewBox="0 0 8 8">'
             '<path d="M1 2.5L4 5.5l3-3" fill="none" stroke="@muted" stroke-width="1.5"'
             ' stroke-linecap="round" stroke-linejoin="round"/></svg>'),
}


def _image_paths(mode: str) -> dict:
    """Small SVGs the QSS refers to (`image: url(...)` needs files), written to the temp dir."""
    import os
    import tempfile
    t = PALETTES[mode]
    out = {}
    for name, svg in _IMAGES.items():
        svg = svg.replace("@muted", t["muted"])
        path = os.path.join(tempfile.gettempdir(), f"coldspray_qss_{name}_{mode}.svg")
        try:
            with open(path, encoding="utf-8") as f:
                same = f.read() == svg
        except OSError:
            same = False
        if not same:
            with open(path, "w", encoding="utf-8") as f:
                f.write(svg)
        out["img_" + name] = path.replace("\\", "/")
    return out


def apply(app: QApplication | None = None, mode: str = "system") -> str:
    """Style `app` (default: the running QApplication). `mode`: "system" (follow
    Windows), "light" or "dark". Returns the resolved mode ("light" / "dark").
    Call again to switch theme; then reset the 3D views' backgrounds with
    `view_background(mode)`."""
    app = app or QApplication.instance()
    resolved = resolve(mode)
    app.setStyle(QStyleFactory.create("Fusion"))
    app.setPalette(palette(resolved))
    font = QFont(fonts()["body"])
    font.setPointSizeF(9.5)
    app.setFont(font)
    app.setStyleSheet(stylesheet(resolved))
    app.setProperty("coldsprayTheme", resolved)
    return resolved


def current_mode(app: QApplication | None = None) -> str:
    """The mode last applied with `apply` ("light" if none)."""
    app = app or QApplication.instance()
    return (app.property("coldsprayTheme") if app is not None else None) or "light"


def repolish(widget):
    """Re-apply property selectors after `widget.setProperty(...)` (and its children)."""
    from PySide6.QtWidgets import QWidget
    for w in (widget, *widget.findChildren(QWidget)):
        w.style().unpolish(w)
        w.style().polish(w)
        w.update()
