import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6.QtWidgets import QApplication, QComboBox, QLabel
from coldspray.gui.widgets import Segmented, TileGrid, fit_combo

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


def test_segmented_swaps_and_syncs():
    from PySide6.QtWidgets import QWidget, QVBoxLayout
    host = QWidget()
    lay = QVBoxLayout(host)
    c = QComboBox()
    c.addItems(["Alpha", "Beta", "Gamma"])
    seg = Segmented(c, host)
    lay.addWidget(c)
    lay.addWidget(seg)
    host.resize(600, 80); host.show(); _app.processEvents()
    assert seg.isVisibleTo(host) and not c.isVisibleTo(host)
    host.resize(60, 80); _app.processEvents()
    assert c.isVisibleTo(host) and not seg.isVisibleTo(host)
    host.resize(600, 80); _app.processEvents()
    assert seg.isVisibleTo(host) and not c.isVisibleTo(host)
    seg._buttons[2].click()
    assert c.currentIndex() == 2
    c.setCurrentIndex(1)
    assert seg._buttons[1].isChecked()
