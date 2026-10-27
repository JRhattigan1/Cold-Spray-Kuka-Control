"""Advanced process rows follow the Simple/Advanced mode (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtWidgets import QApplication, QWidget

from coldspray.gui import widgets
from coldspray.gui.steps.toolpath import advanced_row, make_process_box
from coldspray.models import ProcessConfig, field_level, load_process


@pytest.fixture(autouse=True)
def app():
    app = QApplication.instance() or QApplication([])
    widgets._reset_mode_state()
    yield app
    widgets._reset_mode_state()


def test_advanced_row_hides_in_simple_unless_changed():
    proc = load_process()
    name = next(n for n, i in ProcessConfig.model_fields.items()
                if field_level(n) == "advanced" and i.annotation in (int, float))
    parent = QWidget()
    box = make_process_box(name, ProcessConfig.model_fields[name], getattr(proc, name), lambda *_: widgets.refresh())
    row = advanced_row(name, box, getattr(proc, name))
    row.setParent(parent)
    chip = row.findChild(widgets.Chip)
    assert not row.isHidden()
    widgets.set_mode(widgets.SIMPLE)
    assert row.isHidden() and chip.text() == ""
    box.setValue(box.value() + 1)
    assert not row.isHidden() and chip.text() == "changed"
    widgets.set_mode(widgets.ADVANCED)
    assert not row.isHidden() and chip.text() == ""


def test_sync_advanced_opens_section_in_simple_and_skips_text_fields():
    from types import SimpleNamespace
    from PySide6.QtWidgets import QLineEdit
    from coldspray.gui.steps.toolpath import ToolpathStep
    proc = load_process()
    name = next(n for n, i in ProcessConfig.model_fields.items()
                if field_level(n) == "advanced" and i.annotation in (int, float))
    box = make_process_box(name, ProcessConfig.model_fields[name], getattr(proc, name), lambda *_: None)
    text = QLineEdit("not-the-default")
    step = SimpleNamespace(fields={name: box, "turn_plane": text}, process_defaults=proc,
                           advanced_section=widgets.Section("Advanced", open=False))
    widgets.set_mode(widgets.SIMPLE)
    ToolpathStep._sync_advanced(step)
    assert step.advanced_section.isHidden() and not step.advanced_section._open
    box.setValue(box.value() + 1)
    ToolpathStep._sync_advanced(step)
    assert not step.advanced_section.isHidden() and step.advanced_section._open
