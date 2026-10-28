import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6.QtWidgets import QApplication, QComboBox, QLabel
from coldspray.gui.widgets import TileGrid, fit_combo

_app = QApplication.instance() or QApplication([])


def test_fit_combo_shrinks_and_popup_wide():
    c = QComboBox()
    c.addItems(["Central Injector Kit (CIK) (180 mm) long name here", "b"])
    fit_combo(c, 12)
    assert c.sizeAdjustPolicy() == QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
    assert c.minimumContentsLength() == 12
    assert c.view().minimumWidth() > c.sizeHint().width()


def test_tilegrid_two_cols():
    g = TileGrid(cols=2)
    for _ in range(5):
        g.add(QLabel("x"))
    assert g.count() == 5
    lay = g.layout()
    assert lay.rowCount() == 3 and lay.columnCount() == 2
