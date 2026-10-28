"""The step bar across the top: 1 Part › 2 Position › 3 Toolpath › 4 Simulate › 5 Export.

Each pill shows its state (done ✓, stale ● = out of date, todo) and whether it is the
current step; clicking a pill emits `stepClicked(index)`. Chips (e.g. the file name)
sit on the right in `chip_area`.
"""

from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QFrame, QGraphicsDropShadowEffect, QHBoxLayout, QLabel, QWidget

from coldspray.gui.style import resolve, tokens

from coldspray.gui.widgets import Chip

STEPS = ("Part", "Position", "Toolpath", "Simulate", "Export")
PART, POSITION, TOOLPATH, SIMULATE, EXPORT = range(len(STEPS))
STATES = ("done", "stale", "todo")
MARKS = {"done": "✓", "stale": "●", "todo": ""}
TIPS = {"done": "done", "stale": "out of date: an earlier step changed", "todo": "not done yet"}


class _Pill(QFrame):
    clicked = Signal()

    def __init__(self, number: int, name: str):
        super().__init__()
        self.setObjectName("pill")
        self.setCursor(Qt.PointingHandCursor)
        self.name = name
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 3, 10, 3)
        layout.setSpacing(6)
        self.text = QLabel(f"{number}  {name}")
        self.text.setObjectName("pillText")
        self.mark = QLabel()
        self.mark.setObjectName("pillMark")
        layout.addWidget(self.text)
        layout.addWidget(self.mark)
        self.state, self.current = "todo", False

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)

    def update_look(self):
        self.mark.setText(MARKS[self.state])
        self.mark.setVisible(bool(MARKS[self.state]))
        self.setToolTip(f"{self.name}: {TIPS[self.state]}")
        self.setProperty("state", self.state)
        self.setProperty("current", self.current)
        if self.current:
            glow = QGraphicsDropShadowEffect(self)
            glow.setBlurRadius(16)
            glow.setOffset(0, 0)
            glow.setColor(QColor(tokens()["accent"]).lighter(105))
            self.setGraphicsEffect(glow)
        else:
            self.setGraphicsEffect(None)
        for w in (self, self.text, self.mark):      # re-apply property selectors
            w.style().unpolish(w)
            w.style().polish(w)


class _Capsule(QWidget):
    """Translucent glass pill behind the steps: soft shadow, hairline highlight border."""

    def __init__(self):
        super().__init__()
        self.setAttribute(Qt.WA_StyledBackground, False)
        # No QGraphicsEffect here: it would render every child pill into a blurred pixmap.

    def paintEvent(self, event):
        dark = resolve() == "dark"
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        radius = r.height() / 2
        p.setPen(Qt.NoPen)                      # painted soft shadow (cheap, keeps text crisp)
        for i, a in enumerate((10, 8, 6)):
            p.setBrush(QColor(0, 0, 0, a))
            p.drawRoundedRect(r.translated(0, 1 + i * 0.5).adjusted(-i * 0.3, 0, i * 0.3, 0), radius, radius)
        fill = QColor(255, 255, 255, 22) if dark else QColor(255, 255, 255, 170)
        p.setBrush(fill)
        p.setPen(QPen(QColor(255, 255, 255, 60 if dark else 220), 1))
        p.drawRoundedRect(r, radius, radius)
        p.setBrush(Qt.NoBrush)                  # faint inner rim for depth
        p.setPen(QPen(QColor(255, 255, 255, 14) if dark else QColor(0, 0, 0, 22), 1))
        p.drawRoundedRect(r.adjusted(1, 1, -1, -1), radius - 1, radius - 1)


def _glass_qss() -> str:
    t = tokens()
    dark = resolve() == "dark"
    idle = "rgba(255,255,255,0.06)" if dark else "rgba(255,255,255,0.55)"
    hover = "rgba(255,255,255,0.14)" if dark else "rgba(255,255,255,0.9)"
    on_accent = "#ffffff"
    return f"""
QWidget#stepBar {{ background: transparent; border: none; }}
QFrame#pill {{ border: 1px solid transparent; border-radius: 12px; background: transparent;
    min-height: 24px; max-height: 24px; }}
QFrame#pill:hover {{ background: {hover}; border-color: transparent; }}
QFrame#pill QLabel#pillText {{ color: {t['muted']}; background: transparent; }}
QFrame#pill[state="done"] QLabel#pillText, QFrame#pill[state="stale"] QLabel#pillText {{ color: {t['ink']}; }}
QFrame#pill[current="true"] {{ background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
    stop:0 {QColor(t['accent']).lighter(112).name()}, stop:1 {t['accent']});
    border: 1px solid {QColor(t['accent']).lighter(135).name()}; }}
QFrame#pill[current="true"] QLabel#pillText {{ color: {on_accent}; font-weight: 600; }}
QFrame#pill[current="true"][state="done"] QLabel#pillMark {{ color: {on_accent}; }}
QFrame#pill[current="true"][state="stale"] QLabel#pillMark {{ color: #ffe3a3; }}
QFrame#pill QLabel#pillMark {{ background: transparent; }}
QLabel#pillSep {{ color: transparent; background: transparent; max-width: 0px; }}
QLabel#brand {{ background: transparent; }}
"""


class StepBar(QWidget):
    stepClicked = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("stepBar")
        self.setAttribute(Qt.WA_StyledBackground, True)
        outer = QHBoxLayout(self)
        outer.setContentsMargins(12, 8, 12, 10)
        outer.setSpacing(10)
        self.brand = QLabel("Cold Spray")
        self.brand.setObjectName("brand")
        outer.addWidget(self.brand)
        self.capsule = _Capsule()
        row = QHBoxLayout(self.capsule)
        row.setContentsMargins(5, 4, 5, 4)
        row.setSpacing(2)
        outer.addWidget(self.capsule)
        outer.addStretch(1)
        self.pills: list[_Pill] = []
        for i, name in enumerate(STEPS):
            if i:
                sep = QLabel("›")
                sep.setObjectName("pillSep")
                row.addWidget(sep)
            pill = _Pill(i + 1, name)
            pill.clicked.connect(lambda i=i: self.stepClicked.emit(i))
            row.addWidget(pill)
            self.pills.append(pill)
        self.chip_area = QHBoxLayout()          # right-aligned chips; others may add their own
        self.chip_area.setSpacing(6)
        self.file_chip = Chip()
        self.file_chip.setVisible(False)
        self.chip_area.addWidget(self.file_chip)
        outer.addLayout(self.chip_area)
        self.setStyleSheet(_glass_qss())
        self.current = 0
        self.set_current(0)

    def changeEvent(self, event):
        if event.type() in (event.Type.PaletteChange, event.Type.ApplicationPaletteChange):
            qss = _glass_qss()
            if qss != self.styleSheet():  # setStyleSheet re-fires PaletteChange
                self.setStyleSheet(qss)
            self.capsule.update()
        super().changeEvent(event)

    def set_current(self, i: int):
        self.current = i
        for k, pill in enumerate(self.pills):
            pill.current = k == i
            pill.update_look()

    def set_state(self, i: int, state: str):
        if state not in STATES:
            raise ValueError(f"step state must be one of {STATES}")
        if self.pills[i].state != state:
            self.pills[i].state = state
            self.pills[i].update_look()

    def state(self, i: int) -> str:
        return self.pills[i].state

    def set_chip_text(self, text: str):
        self.file_chip.set(text)
