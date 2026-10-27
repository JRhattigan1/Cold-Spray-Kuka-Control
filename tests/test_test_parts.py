"""Built-in test parts: the plain square block, and the Part step drop-down (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from coldspray.pipeline import BUILT_IN, BUILT_IN_LABELS, load_part


def test_square_block_is_a_plain_box():
    part = load_part("test_square")
    assert part.n_bodies == 1 and len(part.planar) == 6 and not part.cylinders
    assert np.allclose(np.ptp(part.vertices, axis=0), (50, 50, 10))


def test_every_built_in_part_has_a_label():
    assert list(BUILT_IN_LABELS) == list(BUILT_IN)


def test_drop_down_loads_the_chosen_part():
    from PySide6.QtWidgets import QApplication

    from coldspray.gui.app import MainWindow

    QApplication.instance() or QApplication([])
    w = MainWindow()
    loaded = []
    w._set_part = loaded.append                 # no cell assets off screen: just record the choice
    combo = w.test_parts
    assert combo.itemData(0) is None and [combo.itemData(i) for i in range(1, combo.count())] == list(BUILT_IN)
    w._test_part_chosen(combo.findData("test_square"))
    assert loaded == ["test_square"] and combo.currentIndex() == 0
    w._test_part_chosen(0)                      # the prompt itself loads nothing
    assert loaded == ["test_square"]
    w.close()
