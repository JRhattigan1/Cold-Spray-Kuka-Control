"""Small shared widgets: a folding Section with a state chip, Chip, and field helpers.

Their look comes from the app stylesheet (gui/style.py), through object names and
Qt properties: chip `kind`, section header `open`, button `primary`, label `note`.
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QDoubleSpinBox, QFrame, QHBoxLayout, QLabel, QLayout, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
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
