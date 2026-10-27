"""Small shared widgets: a folding Section with a state chip, Chip, and field helpers.

Their look comes from the app stylesheet (gui/style.py), through object names and
Qt properties: chip `kind`, section header `open`, button `primary`, label `note`.
"""

from dataclasses import dataclass
from typing import Callable

import shiboken6
from PySide6.QtCore import QSettings, Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QButtonGroup, QDoubleSpinBox, QFrame, QHBoxLayout, QLabel, QLayout, QPushButton, QSizePolicy, QVBoxLayout,
    QWidget,
)

CHIP_KINDS = ("good", "bad", "warn", "crash", "neutral")


def _repolish(widget):
    """Re-apply stylesheet rules after a property change."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)


class Chip(QLabel):
    """A small rounded status label. `kind`: good | bad | warn | crash | neutral."""

    def __init__(self, text: str = "", kind: str = "neutral", parent=None):
        super().__init__(text, parent)
        self.setObjectName("chip")
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.set_kind(kind)

    def set_kind(self, kind: str):
        if kind not in CHIP_KINDS:
            raise ValueError(f"chip kind must be one of {CHIP_KINDS}")
        self.kind = kind
        self.setProperty("kind", kind)
        _repolish(self)

    def set(self, text: str, kind: str = "neutral"):
        self.setText(text)
        self.set_kind(kind)
        self.setVisible(bool(text))



class Section(QWidget):
    """A folding group: header (▾/▸ and title, optional chip on the right) over a body.

    Add content with `add(widget_or_layout)` or straight to `.body` (a QVBoxLayout).
    """

    toggled = Signal(bool)        # open / folded

    def __init__(self, title: str, parent=None, open: bool = True):
        super().__init__(parent)
        self.setObjectName("section")
        self.title = title
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self.header = QFrame()
        self.header.setObjectName("sectionHeader")
        hl = QHBoxLayout(self.header)
        hl.setContentsMargins(6, 3, 6, 3)
        self.toggle_btn = QPushButton()
        self.toggle_btn.setObjectName("sectionToggle")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setFlat(True)
        self.toggle_btn.setCursor(Qt.PointingHandCursor)
        self.toggle_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        font = self.toggle_btn.font()
        font.setBold(True)
        self.toggle_btn.setFont(font)
        self.toggle_btn.toggled.connect(self.set_open)
        self.chip = Chip()
        self.chip.setVisible(False)
        hl.addWidget(self.toggle_btn, 1)
        hl.addWidget(self.chip)
        outer.addWidget(self.header)

        self.content = QFrame()
        self.content.setObjectName("sectionBody")
        self.body = QVBoxLayout(self.content)
        self.body.setContentsMargins(8, 6, 8, 8)
        outer.addWidget(self.content)

        self._open = None
        self.set_open(open)

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

    def set_open(self, on: bool):
        on = bool(on)
        self.toggle_btn.blockSignals(True)
        self.toggle_btn.setChecked(on)
        self.toggle_btn.blockSignals(False)
        self.toggle_btn.setText(f"{'▾' if on else '▸'}  {self.title}")
        self.content.setVisible(on)
        self.header.setProperty("open", on)
        _repolish(self.header)
        if on != self._open:
            self._open = on
            self.toggled.emit(on)

    def is_open(self) -> bool:
        return bool(self._open)



def primary_button(text: str, slot=None) -> QPushButton:
    """A main action button (Next, Generate): bold, `primary` property set for the theme."""
    btn = QPushButton(text)
    btn.setProperty("primary", True)
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
