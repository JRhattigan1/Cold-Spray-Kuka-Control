"""HOME editor: save_home keeps cell.yaml comments; the widget emits the pose and saves."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import shutil
from types import SimpleNamespace

import pytest
from PySide6.QtWidgets import QApplication

from coldspray.config_io import save_home
from coldspray.models import CONFIG_DIR, load_cell


@pytest.fixture
def cell_copy(tmp_path):
    p = tmp_path / "cell.yaml"
    shutil.copyfile(CONFIG_DIR / "cell.yaml", p)
    return p


def _lines(p):
    with open(p, newline="") as f:
        return f.read().splitlines(keepends=True)


def test_save_home_roundtrip_keeps_other_lines(cell_copy):
    before = _lines(cell_copy)
    save_home([10, -80, 95.5, 0, 20, 0], cell_copy)
    after = _lines(cell_copy)
    assert load_cell(cell_copy).home_deg == [10, -80, 95.5, 0, 20, 0]
    assert len(after) == len(before)
    diff = [(a, b) for a, b in zip(before, after) if a != b]
    assert len(diff) == 1 and diff[0][1].startswith("home_deg:")
    save_home(load_cell(CONFIG_DIR / "cell.yaml").home_deg, cell_copy)
    assert _lines(cell_copy) == before


@pytest.fixture
def editor(cell_copy):
    QApplication.instance() or QApplication([])
    from coldspray.gui.home_editor import HomeEditor
    win = SimpleNamespace(assets=SimpleNamespace(cell=load_cell(cell_copy), robot=None),
                          scene=None, checker=None)
    return HomeEditor(win, path=cell_copy), win, cell_copy


def test_editor_builds_and_emits(editor):
    ed, win, _ = editor
    assert ed.joints() == [float(v) for v in win.assets.cell.home_deg]
    if win.assets.cell.gantry is not None:
        assert ed.e1_mm() == win.assets.cell.gantry.position_mm
    got = []
    ed.pose_changed.connect(lambda q, e1: got.append((q, e1)))
    ed.spins[0].setValue(30.0)
    assert got and got[-1][0][0] == 30.0
    assert "checker not ready" in ed.status.text()
    ed.spins[1].setValue(100.0)                   # A2 limit -175..60
    assert "A2" in ed.status.text() and "outside" in ed.status.text()


def test_editor_save_updates_live_cell(editor):
    ed, win, path = editor
    ed.spins[0].setValue(15.0)
    ed.save()
    assert win.assets.cell.home_deg[0] == 15.0
    assert load_cell(path).home_deg[0] == 15.0
    ed.spins[0].setValue(40.0)
    ed.reset()
    assert ed.spins[0].value() == 15.0


def test_editor_uses_checker(editor):
    ed, win, _ = editor
    calls = []
    win.checker = SimpleNamespace(check=lambda q, axes: calls.append((q, axes)) or {("gun", "table base")})
    ed.spins[0].setValue(5.0)
    assert calls and "gun / table base" in ed.status.text()
