import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6.QtWidgets import QApplication
from coldspray.gui import timeline

app = QApplication.instance() or QApplication([])


def test_narrow_timeline_paints_with_short_legend():
    t = timeline.Timeline()
    for w in (336, 900):
        t.resize(w, t.height())
        img = t.grab().toImage()
        assert img.width() == w
    assert set(timeline.SHORT_LABELS) == set(timeline.LABELS)


def test_fit_panel_lets_inputs_shrink():
    from PySide6.QtWidgets import QComboBox, QDoubleSpinBox, QVBoxLayout, QWidget
    from coldspray.gui.progress import fit_panel
    w = QWidget()
    QVBoxLayout(w)
    sb = QDoubleSpinBox()
    w.layout().addWidget(sb)
    cb = QComboBox()
    cb.addItem("a very long item text that would force the panel wide")
    w.layout().addWidget(cb)
    fit_panel(w)
    assert sb.minimumWidth() == 0 and cb.minimumSizeHint().width() < 200
