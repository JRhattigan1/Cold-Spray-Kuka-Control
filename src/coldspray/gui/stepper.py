"""The step bar across the top: 1 Part › 2 Position › 3 Toolpath › 4 Simulate › 5 Export.

Each pill shows its state (done ✓, stale ● = out of date, todo) and whether it is the
current step; clicking a pill emits `stepClicked(index)`. Chips (e.g. the file name)
sit on the right in `chip_area`.
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QWidget

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
        for w in (self, self.text, self.mark):      # re-apply property selectors
            w.style().unpolish(w)
            w.style().polish(w)


class StepBar(QWidget):
    stepClicked = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("stepBar")
        self.setAttribute(Qt.WA_StyledBackground, True)
        row = QHBoxLayout(self)
        row.setContentsMargins(10, 6, 10, 6)
        row.setSpacing(6)
        self.brand = QLabel("Cold Spray")
        self.brand.setObjectName("brand")
        row.addWidget(self.brand)
        row.addSpacing(10)
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
        row.addStretch(1)
        self.chip_area = QHBoxLayout()          # right-aligned chips; others may add their own
        self.chip_area.setSpacing(6)
        self.file_chip = Chip()
        self.file_chip.setVisible(False)
        self.chip_area.addWidget(self.file_chip)
        row.addLayout(self.chip_area)
        self.current = 0
        self.set_current(0)

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
