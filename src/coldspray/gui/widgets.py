"""Small shared widgets: a folding Section with a state chip, Chip, and field helpers.

Their look comes from the app stylesheet (gui/style.py), through object names and
Qt properties: chip `kind`, section header `open`, button `primary`, label `note`.
"""

from dataclasses import dataclass
from typing import Callable

import shiboken6
from PySide6.QtCore import QEasingCurve, QPointF, QRectF, QSettings, Qt, QVariantAnimation, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import (
    QButtonGroup, QComboBox, QDoubleSpinBox, QGridLayout, QFrame, QGraphicsDropShadowEffect, QHBoxLayout, QLabel, QLayout, QPushButton, QSizePolicy, QVBoxLayout,
    QWidget,
)

CHIP_KINDS = ("good", "bad", "warn", "crash", "neutral")


def _repolish(widget):
    """Re-apply stylesheet rules after a property change."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)


def _tok() -> dict:
    """Colour tokens for the theme the app is showing now (read from the live palette)."""
    from PySide6.QtGui import QGuiApplication
    from coldspray.gui.style import tokens
    app = QGuiApplication.instance()
    dark = app is not None and app.palette().window().color().lightness() < 128
    return tokens("dark" if dark else "light")


def _rgba(hex_colour: str, alpha: float) -> QColor:
    c = QColor(hex_colour)
    c.setAlphaF(alpha)
    return c


class Chip(QLabel):
    """A capsule status label with a tinted translucent fill. `kind`: good | bad | warn | crash | neutral."""

    def __init__(self, text: str = "", kind: str = "neutral", parent=None):
        super().__init__(text, parent)
        self.setObjectName("chip")
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.setStyleSheet("QLabel#chip { background: transparent; border: none; padding: 2px 10px; "
                           "font-weight: 600; }")
        self.set_kind(kind)

    def set_kind(self, kind: str):
        if kind not in CHIP_KINDS:
            raise ValueError(f"chip kind must be one of {CHIP_KINDS}")
        self.kind = kind
        self.setProperty("kind", kind)
        self.update()

    def set(self, text: str, kind: str = "neutral"):
        self.setText(text)
        self.set_kind(kind)
        self.setVisible(bool(text))

    def paintEvent(self, event):
        t = _tok()
        base = t["muted"] if self.kind == "neutral" else t[self.kind]
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        radius = r.height() / 2
        p.setPen(Qt.NoPen)
        p.setBrush(_rgba(base, 0.16))
        p.drawRoundedRect(r, radius, radius)
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(_rgba(base, 0.45), 1))
        p.drawRoundedRect(r, radius, radius)
        p.setPen(QColor(base))
        p.setFont(self.font())
        p.drawText(self.rect(), Qt.AlignCenter, self.text())
        p.end()


class _Chevron(QWidget):
    """A small chevron that rotates smoothly between folded (right) and open (down)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(16, 16)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._angle = 0.0
        self._anim = QVariantAnimation(self)
        self._anim.setDuration(180)
        self._anim.setEasingCurve(QEasingCurve.OutCubic)
        self._anim.valueChanged.connect(self._set_angle)

    def _set_angle(self, v):
        self._angle = float(v)
        self.update()

    def set_open(self, on: bool, animate: bool):
        target = 90.0 if on else 0.0
        self._anim.stop()
        if animate:
            self._anim.setStartValue(self._angle)
            self._anim.setEndValue(target)
            self._anim.start()
        else:
            self._set_angle(target)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.translate(self.width() / 2, self.height() / 2)
        p.rotate(self._angle)
        p.setPen(QPen(QColor(_tok()["muted"]), 1.8, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        p.drawPolyline([QPointF(-2, -4), QPointF(2, 0), QPointF(-2, 4)])
        p.end()


class Section(QWidget):
    """A folding glass card: header (chevron and title, optional chip on the right) over a body.

    Add content with `add(widget_or_layout)` or straight to `.body` (a QVBoxLayout).
    """

    toggled = Signal(bool)        # open / folded

    def __init__(self, title: str, parent=None, open: bool = True):
        super().__init__(parent)
        self.setObjectName("section")
        self.title = title
        # the card is painted here; header and body stay see-through
        self.setStyleSheet("QFrame#sectionHeader, QFrame#sectionBody { background: transparent; border: none; }"
                           "QPushButton#sectionToggle { background: transparent; border: none; text-align: left; }")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self.header = QFrame()
        self.header.setObjectName("sectionHeader")
        hl = QHBoxLayout(self.header)
        hl.setContentsMargins(12, 6, 10, 6)
        hl.setSpacing(6)
        self.chevron = _Chevron()
        self.toggle_btn = QPushButton()
        self.toggle_btn.setObjectName("sectionToggle")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setFlat(True)
        self.toggle_btn.setCursor(Qt.PointingHandCursor)
        self.toggle_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        font = self.toggle_btn.font()
        font.setBold(True)
        self.toggle_btn.setFont(font)
        self.toggle_btn.setText(title)
        self.toggle_btn.toggled.connect(self.set_open)
        self.chip = Chip()
        self.chip.setVisible(False)
        hl.addWidget(self.chevron)
        hl.addWidget(self.toggle_btn, 1)
        hl.addWidget(self.chip)
        outer.addWidget(self.header)

        self.content = QFrame()
        self.content.setObjectName("sectionBody")
        self.body = QVBoxLayout(self.content)
        self.body.setContentsMargins(12, 4, 12, 12)
        outer.addWidget(self.content)

        self._fold = QVariantAnimation(self)
        self._fold.setDuration(180)
        self._fold.setEasingCurve(QEasingCurve.OutCubic)
        self._fold.valueChanged.connect(lambda v: self.content.setMaximumHeight(int(v)))
        self._fold.finished.connect(self._fold_done)

        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(24)
        shadow.setOffset(0, 4)
        shadow.setColor(QColor(0, 0, 0, 40))
        self.setGraphicsEffect(shadow)

        self._open = None
        self.set_open(open)

    def paintEvent(self, event):
        t = _tok()
        dark = QColor(t["bg"]).lightness() < 128
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        sheen = QLinearGradient(r.topLeft(), r.bottomLeft())
        sheen.setColorAt(0, QColor(255, 255, 255, 22 if dark else 170))
        sheen.setColorAt(1, QColor(255, 255, 255, 8 if dark else 110))
        p.setPen(Qt.NoPen)
        p.setBrush(_rgba(t["panel"], 0.62))
        p.drawRoundedRect(r, 14, 14)
        p.setBrush(sheen)
        p.drawRoundedRect(r, 14, 14)
        edge = QLinearGradient(r.topLeft(), r.bottomLeft())    # hairline: bright on top, faint below
        edge.setColorAt(0, QColor(255, 255, 255, 70 if dark else 230))
        edge.setColorAt(1, _rgba(t["line"], 0.7))
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QBrush(edge), 1))
        p.drawRoundedRect(r, 14, 14)
        p.end()

    def add(self, item):
        """Add a widget or a layout to the body; returns it."""
        if isinstance(item, QLayout):
            self.body.addLayout(item)
        else:
            self.body.addWidget(item)
        return item

    def set_chip(self, text: str, kind: str = "neutral"):
        """Chip on the right of the header (stays visible when folded); empty text hides it."""
        self.chip.set(text, kind)

    def _fold_done(self):
        self.content.setMaximumHeight(16777215)
        self.content.setVisible(bool(self._open))

    def set_open(self, on: bool):
        on = bool(on)
        self.toggle_btn.blockSignals(True)
        self.toggle_btn.setChecked(on)
        self.toggle_btn.blockSignals(False)
        animate = self._open is not None and self.isVisible() and on != self._open
        self.chevron.set_open(on, animate)
        self._fold.stop()
        if animate:
            full = self.content.sizeHint().height()
            start = 0 if on else self.content.height()
            self.content.setMaximumHeight(start)
            self.content.show()
            self._fold.setStartValue(start)
            self._fold.setEndValue(full if on else 0)
            self._fold.start()
        else:
            self.content.setMaximumHeight(16777215)
            self.content.setVisible(on)
        self.header.setProperty("open", on)
        if on != self._open:
            self._open = on
            self.toggled.emit(on)

    def is_open(self) -> bool:
        return bool(self._open)


class _PrimaryButton(QPushButton):
    """Orange gradient capsule."""

    def paintEvent(self, event):
        t = _tok()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        radius = r.height() / 2
        acc = QColor(t["accent"])
        hi, lo = acc.lighter(118), acc.darker(112)
        if not self.isEnabled():
            hi.setAlphaF(0.4)
            lo.setAlphaF(0.4)
        elif self.isDown():
            hi, lo = lo, acc.darker(125)
        elif self.underMouse():
            hi, lo = hi.lighter(108), acc
        g = QLinearGradient(r.topLeft(), r.bottomLeft())
        g.setColorAt(0, hi)
        g.setColorAt(1, lo)
        p.setPen(Qt.NoPen)
        p.setBrush(g)
        p.drawRoundedRect(r, radius, radius)
        gloss = QLinearGradient(r.topLeft(), r.bottomLeft())    # glossy top edge
        gloss.setColorAt(0, QColor(255, 255, 255, 90))
        gloss.setColorAt(0.5, QColor(255, 255, 255, 0))
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QBrush(gloss), 1))
        p.drawRoundedRect(r, radius, radius)
        p.setPen(QColor("#ffffff") if self.isEnabled() else QColor(255, 255, 255, 170))
        p.setFont(self.font())
        p.drawText(self.rect(), Qt.AlignCenter, self.text())
        p.end()


def primary_button(text: str, slot=None) -> QPushButton:
    """A main action button (Next, Generate): orange gradient capsule, `primary` property set."""
    btn = _PrimaryButton(text)
    btn.setProperty("primary", True)
    btn.setAttribute(Qt.WA_Hover, True)
    btn.setCursor(Qt.PointingHandCursor)
    btn.setMinimumHeight(30)
    btn.setStyleSheet('QPushButton[primary="true"] { background: transparent; border: none; padding: 4px 18px; }')
    font = QFont(btn.font())
    font.setBold(True)
    btn.setFont(font)
    if slot is not None:
        btn.clicked.connect(slot)
    return btn


def spin(lo, hi, value, suffix, decimals=1, tip="") -> QDoubleSpinBox:
    box = QDoubleSpinBox()
    box.setDecimals(decimals)
    box.setRange(lo, hi)
    box.setValue(value)
    box.setSuffix(suffix)
    box.setToolTip(tip)
    return box


def wrap_label(text: str = "") -> QLabel:
    label = QLabel(text)
    label.setWordWrap(True)
    return label


# --- Simple / Advanced mode ---------------------------------------------------------
# Widgets most users won't need are registered with `advanced(w)`; Simple mode hides them.
# A registered widget can pass `changed`: while it returns True (the value differs from the
# default) the widget stays visible in Simple mode and its `chip` says "changed", so Simple
# mode never hides a value that changes the output. Call `refresh()` after such values change.

SIMPLE, ADVANCED = "simple", "advanced"
SETTINGS_SCOPE = ("ColdSpray", "KukaControl")    # QSettings organisation, application
MODE_KEY = "ui/mode"


def fit_combo(combo: QComboBox, n: int = 12) -> QComboBox:
    """Let a combo shrink to ~n characters in a narrow sidebar; its popup stays as wide as its items."""
    combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
    combo.setMinimumContentsLength(n)
    combo.setSizePolicy(QSizePolicy.Policy.Expanding, combo.sizePolicy().verticalPolicy())
    view = combo.view()
    fm = view.fontMetrics()
    w = max((fm.horizontalAdvance(combo.itemText(i)) for i in range(combo.count())), default=0)
    if w:
        view.setMinimumWidth(w + 40)
    return combo


class Segmented(QWidget):
    """Segmented capsule row bound to a QComboBox (the combo stays the data API).

    Shows the segments when they fit the available width, otherwise hides itself and shows the combo.
    """

    def __init__(self, combo: QComboBox, parent: QWidget | None = None):
        super().__init__(parent or combo.parentWidget())
        self._combo = combo
        self.setObjectName("Segmented")
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._buttons: list[QPushButton] = []
        for i in range(combo.count()):
            b = QPushButton(combo.itemText(i))
            b.setCheckable(True)
            b.setObjectName("SegmentButton")
            self._group.addButton(b, i)
            lay.addWidget(b)
            self._buttons.append(b)
        self._group.idClicked.connect(self._on_clicked)
        combo.currentIndexChanged.connect(self._sync)
        self._sync(combo.currentIndex())
        if self.parentWidget() is not None:
            self.parentWidget().installEventFilter(self)

    def _on_clicked(self, i: int):
        if self._combo.currentIndex() != i:
            self._combo.setCurrentIndex(i)

    def _sync(self, i: int):
        if 0 <= i < len(self._buttons):
            self._buttons[i].setChecked(True)

    def _natural_width(self) -> int:
        return sum(b.sizeHint().width() for b in self._buttons)

    def _avail_width(self) -> int:
        return self.width() if self.isVisibleTo(self.parentWidget()) else (self.parentWidget().width() if self.parentWidget() else 0)

    def _update_mode(self):
        fits = self._natural_width() <= self._avail_width()
        self.setVisible(fits)
        self._combo.setVisible(not fits)

    def sizeHint(self):
        sh = super().sizeHint()
        sh.setWidth(self._natural_width())
        return sh

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._update_mode()

    def eventFilter(self, obj, ev):
        if obj is self.parentWidget() and ev.type() == ev.Type.Resize:
            self._update_mode()
        return super().eventFilter(obj, ev)


class TileGrid(QWidget):
    """Tiles laid out in `cols` columns (default 2); add() fills row by row."""

    def __init__(self, cols: int = 2, parent: QWidget | None = None):
        super().__init__(parent)
        self._cols = max(1, cols)
        self._n = 0
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setSpacing(6)
        for c in range(self._cols):
            self._grid.setColumnStretch(c, 1)

    def add(self, widget: QWidget) -> QWidget:
        widget.setSizePolicy(QSizePolicy.Policy.Ignored, widget.sizePolicy().verticalPolicy())
        self._grid.addWidget(widget, self._n // self._cols, self._n % self._cols)
        self._n += 1
        return widget

    def count(self) -> int:
        return self._n


@dataclass
class _Entry:
    widget: QWidget
    changed: Callable[[], bool] | None = None
    chip: Chip | None = None


_entries: list[_Entry] = []
_mode = ADVANCED
_listeners: list[Callable[[str], None]] = []


def settings() -> QSettings:
    """The app's persistent settings (Windows registry under SETTINGS_SCOPE)."""
    return QSettings(*SETTINGS_SCOPE)


def mode() -> str:
    return _mode


def is_simple() -> bool:
    return _mode == SIMPLE


def advanced(widget: QWidget, changed: Callable[[], bool] | None = None, chip: Chip | None = None) -> QWidget:
    """Register `widget` as Advanced-only and apply the current mode to it; returns it.

    changed: () -> bool, True when the value differs from its default (keeps it shown in Simple).
    chip: a Chip shown as "changed" when that is why the widget is visible in Simple mode.
    """
    _entries.append(_Entry(widget, changed, chip))
    _apply(_entries[-1])
    return widget


def set_mode(new: str):
    """Switch every registered widget to SIMPLE or ADVANCED and tell the listeners."""
    global _mode
    if new not in (SIMPLE, ADVANCED):
        raise ValueError(f"mode must be {SIMPLE!r} or {ADVANCED!r}")
    _mode = new
    refresh()
    for fn in list(_listeners):
        fn(new)


def on_mode_changed(fn: Callable[[str], None]):
    """Call fn(mode) after every mode switch (e.g. to re-lay out a form)."""
    _listeners.append(fn)


def refresh():
    """Re-apply the mode, e.g. after an advanced value was edited; drops deleted widgets."""
    _entries[:] = [e for e in _entries if shiboken6.isValid(e.widget)]
    for e in _entries:
        _apply(e)


def hidden_count() -> int:
    """How many registered widgets the current mode hides."""
    return sum(1 for e in _entries if shiboken6.isValid(e.widget) and e.widget.isHidden())


def _apply(e: _Entry):
    changed = bool(e.changed and e.changed())
    e.widget.setVisible(_mode == ADVANCED or changed)
    if e.chip is not None and shiboken6.isValid(e.chip):
        e.chip.set("changed" if changed and _mode == SIMPLE else "", "warn")


def _reset_mode_state():
    """Forget all registrations and listeners (tests)."""
    global _mode
    _entries.clear()
    _listeners.clear()
    _mode = ADVANCED


class ModeToggle(QWidget):
    """The `Simple | Advanced` segmented switch for the header. Reads the saved mode on
    creation, applies it with `set_mode`, and saves every change to `store` (QSettings)."""

    def __init__(self, store: QSettings | None = None, parent=None):
        super().__init__(parent)
        self.setObjectName("modeToggle")
        self.store = store if store is not None else settings()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(2)
        self.group = QButtonGroup(self)
        self.buttons: dict[str, QPushButton] = {}
        for key, text, tip in ((SIMPLE, "Simple", "Show the settings most jobs need"),
                               (ADVANCED, "Advanced", "Show every setting")):
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.setToolTip(tip)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setProperty("step", True)          # reuse the step-pill look from the stylesheet
            btn.clicked.connect(lambda _=False, k=key: self.choose(k))
            self.group.addButton(btn)
            self.buttons[key] = btn
            row.addWidget(btn)
        saved = self.store.value(MODE_KEY, SIMPLE)
        self.choose(saved if saved in (SIMPLE, ADVANCED) else SIMPLE, save=False)

    def choose(self, key: str, save: bool = True):
        set_mode(key)
        for k, btn in self.buttons.items():
            btn.setChecked(k == key)
            btn.setProperty("state", "current" if k == key else "todo")
            _repolish(btn)
        if save:
            self.store.setValue(MODE_KEY, key)
            self.store.sync()
