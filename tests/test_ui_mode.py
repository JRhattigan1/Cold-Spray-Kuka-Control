"""Simple / Advanced mode: registered widgets hide and show, changed ones stay, mode persists (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication, QLabel, QWidget

from coldspray.gui import widgets
from coldspray.gui.widgets import ADVANCED, SIMPLE, Chip, ModeToggle


@pytest.fixture(autouse=True)
def app():
    app = QApplication.instance() or QApplication([])
    widgets._reset_mode_state()
    yield app
    widgets._reset_mode_state()


@pytest.fixture
def store(tmp_path):
    return QSettings(str(tmp_path / "ui.ini"), QSettings.IniFormat)


_parent = None


def label(text):
    """A child label: an unparented, never-shown widget always reports isHidden()."""
    global _parent
    _parent = _parent or QWidget()
    return QLabel(text, _parent)


def test_set_mode_hides_and_shows_registered():
    plain, adv = label("a"), widgets.advanced(label("b"))
    assert not adv.isHidden()                  # default mode is Advanced
    widgets.set_mode(SIMPLE)
    assert adv.isHidden() and not plain.isHidden()
    assert widgets.hidden_count() == 1
    widgets.set_mode(ADVANCED)
    assert not adv.isHidden() and widgets.hidden_count() == 0
    with pytest.raises(ValueError):
        widgets.set_mode("expert")


def test_registering_in_simple_mode_hides_at_once():
    widgets.set_mode(SIMPLE)
    assert widgets.advanced(label("x")).isHidden()


def test_changed_advanced_field_stays_visible_with_chip():
    value = {"v": 0}
    chip = Chip()
    w = widgets.advanced(label("x"), changed=lambda: value["v"] != 0, chip=chip)
    widgets.set_mode(SIMPLE)
    assert w.isHidden() and chip.text() == ""
    value["v"] = 3
    widgets.refresh()
    assert not w.isHidden() and chip.text() == "changed" and chip.kind == "warn"
    widgets.set_mode(ADVANCED)
    assert not w.isHidden() and chip.text() == ""


def test_listeners_called():
    seen = []
    widgets.on_mode_changed(seen.append)
    widgets.set_mode(SIMPLE)
    widgets.set_mode(ADVANCED)
    assert seen == [SIMPLE, ADVANCED]


def test_deleted_widgets_are_dropped():
    w = widgets.advanced(label("x"))
    w.deleteLater()
    QApplication.sendPostedEvents(None, 0)
    from PySide6.QtCore import QEvent
    QApplication.sendPostedEvents(None, QEvent.DeferredDelete)
    widgets.set_mode(SIMPLE)                   # must not touch the dead widget
    assert widgets._entries == []


def test_toggle_defaults_to_simple_and_survives_restart(store):
    adv = widgets.advanced(label("x"))
    toggle = ModeToggle(store)
    assert widgets.mode() == SIMPLE and adv.isHidden()
    assert toggle.buttons[SIMPLE].isChecked()
    toggle.buttons[ADVANCED].click()
    assert widgets.mode() == ADVANCED and not adv.isHidden()
    assert store.value(widgets.MODE_KEY) == ADVANCED

    widgets._reset_mode_state()                 # "restart": fresh registry, new toggle, same store
    adv2 = widgets.advanced(label("y"))
    toggle2 = ModeToggle(QSettings(store.fileName(), QSettings.IniFormat))
    assert widgets.mode() == ADVANCED and toggle2.buttons[ADVANCED].isChecked() and not adv2.isHidden()
    toggle2.buttons[SIMPLE].click()
    assert adv2.isHidden()
