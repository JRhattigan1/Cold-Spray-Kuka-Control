"""Simple / Advanced on the Simulate and Export steps (off screen, temp QSettings)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from coldspray.gui import widgets
from coldspray.gui.widgets import ADVANCED, SIMPLE


@pytest.fixture
def win(tmp_path):
    QApplication.instance() or QApplication([])
    widgets._reset_mode_state()
    store = QSettings(str(tmp_path / "ui.ini"), QSettings.IniFormat)
    store.setValue(widgets.MODE_KEY, SIMPLE)
    from coldspray.gui.app import MainWindow
    w = MainWindow(store)                       # _startup is deferred by a timer and never runs here
    yield w
    w.plotter.close()
    w.part_plotter.close()
    w.pool.shutdown(wait=False)
    widgets._reset_mode_state()


def test_simple_hides_non_essential_simulate_and_export_controls(win):
    adv = (win.collision_row, win.program_section, win.export_save_note)
    assert widgets.mode() == SIMPLE
    assert all(w.isHidden() for w in adv)
    # essentials stay
    for w in (win.checks_section, win.playback_section, win.robot_section, win.save_krl_btn):
        assert not w.isHidden()
    widgets.set_mode(ADVANCED)
    assert not any(w.isHidden() for w in adv)


def test_collision_check_turned_off_stays_visible_in_simple(win):
    win.check_box.setChecked(False)
    assert not win.collision_row.isHidden()
    assert win.collision_chip.text() == "changed"
    win.check_box.setChecked(True)
    assert win.collision_row.isHidden()
