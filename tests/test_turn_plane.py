"""turn_plane combo: datum plane + offset, shown only for beyond_plane (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QLineEdit

from coldspray.gui.steps.toolpath import join_plane, plane_row, split_plane
from coldspray.planner.turns import parse_plane

app = QApplication.instance() or QApplication([])


def test_split_join_round_trip_and_parse():
    assert split_plane("") == ("", 0.0)
    assert split_plane("YZ:60") == ("YZ", 60.0)
    assert split_plane("datum:XZ") == ("XZ", 0.0)
    assert join_plane("YZ", 60.0) == "YZ:60" and join_plane("XY", 0) == "XY" and join_plane("", 5) == ""
    assert parse_plane(join_plane("YZ", 60.0)) is not None


def test_combo_writes_turn_plane_string():
    edit, calls = QLineEdit("YZ:60"), []
    row = plane_row(edit, lambda: calls.append(1))
    assert row.combo.currentData() == "YZ" and row.spin.value() == 60
    row.spin.setValue(75)
    assert edit.text() == "YZ:75" and calls
    row.combo.setCurrentIndex(row.combo.findData(""))
    assert edit.text() == "" and not row.spin.isEnabled()
