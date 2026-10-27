"""Constraints Section of the Position step (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtWidgets import QApplication

from coldspray.cad.part import Substrate
from coldspray.gui.constraints_panel import PART_AXIS, ConstraintsPanel
from coldspray.pipeline import load_part
from coldspray.transforms import frame


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def panel(app):
    tube = load_part("test_tube")
    p = ConstraintsPanel()
    p.set_part(tube, Substrate())
    return p, tube


def bottom(tube):
    return next(i for i, f in tube.planar.items() if np.allclose(f.normal, (0, 0, -1), atol=1e-9))


def test_touch_and_centre_on_tube(panel):
    p, tube = panel
    start = tube.placement @ frame(30, -20, 40, 0, 10, 5)
    p.add("touch", f"face:{bottom(tube)}", "substrate")
    p.add("centre", PART_AXIS, "table_axis")
    t = p.apply(start)
    assert not np.allclose(t, start)
    assert np.allclose(t[:3, :3] @ (0, 0, 1), (0, 0, 1), atol=1e-6)
    assert np.allclose(t[:2, 3], 0, atol=1e-5)
    assert np.isclose(t[2, 3], Substrate().top, atol=1e-4)
    assert p.result.status == "ok" and p.chip.text() == "1 DOF free"
    assert all(r.status.text() == "✓" for r in p.rows)


def test_suppress_and_delete(panel):
    p, tube = panel
    start = tube.placement @ frame(30, -20, 40, 0, 0, 0)
    p.add("touch", f"face:{bottom(tube)}", "substrate")
    centre = p.add("centre", PART_AXIS, "table_axis")
    centre.suppress.setChecked(True)
    t = p.apply(start)
    assert np.allclose(t[:2, 3], start[:2, 3], atol=1e-3)    # centring left out
    assert p.result.dof_left == 3
    centre.delete.click()
    assert len(p.rows) == 1
    p.rows[0].delete.click()
    assert p.rows == [] and np.allclose(p.apply(start), start)


def test_conflict_marks_rows(panel):
    p, tube = panel
    face = f"face:{bottom(tube)}"
    p.add("touch", face, "substrate")
    p.add("touch", face, "faceplate")
    p.apply(tube.placement)
    assert p.result.status != "ok" and p.chip.text() == "conflict"
    assert any(r.status.text() == "✗" for r in p.rows)
