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
