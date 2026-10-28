"""Part and Position panels fit the 420 px sidebar (usable ~388 px)."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from coldspray.gui import app as gui_app


@pytest.mark.parametrize("theme", ["light", "dark"])
def test_part_position_min_width(tmp_path, theme):
    from coldspray.gui import style
    app = QApplication.instance() or QApplication([])
    style.apply(app, theme)                     # the real app always has the stylesheet
    w = gui_app.MainWindow(QSettings(str(tmp_path / "ui.ini"), QSettings.IniFormat))
    w._refresh_steps = lambda: None
    w.show()
    QApplication.processEvents()
    for i in (0, 1):
        scroll = w.panels.widget(i).scroll
        assert scroll.widget().minimumSizeHint().width() <= 402, i
    w.close()
