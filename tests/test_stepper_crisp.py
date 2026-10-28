"""Done-step labels render crisp: full-ink colour, no nested graphics effects."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QGraphicsOpacityEffect

from coldspray.gui.stepper import StepBar


def test_done_label_crisp():
    QApplication.instance() or QApplication([])
    bar = StepBar()
    bar.set_current(2)
    bar.set_state(0, "done")
    bar.set_state(1, "done")
    bar.show()
    for pill in bar.pills[:2]:
        lab = pill.text
        n, w = 0, lab
        while w is not None:
            eff = w.graphicsEffect()
            assert not isinstance(eff, QGraphicsOpacityEffect)
            n += eff is not None
            w = w.parentWidget()
        assert n <= 1
        assert lab.palette().color(lab.foregroundRole()).alpha() == 255
