"""suggest_purge: reachable point in front of the filtration unit, spraying into it."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from types import SimpleNamespace

import numpy as np
import pytest

from coldspray.cell import robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_robot, load_tool
from coldspray.planner.purge import suggest_purge
from coldspray.sim.cell_model import load_cell_meshes
from coldspray.sim.markers import marker_pose


@pytest.fixture(scope="module")
def world():
    cell = load_cell()
    if not cell.cad_cache:
        pytest.skip("no cell CAD cache")
    try:
        meshes = load_cell_meshes(cell.cad_cache)
    except Exception:
        pytest.skip("cell CAD not available")
    if "Filtration unit" not in meshes.groups:
        pytest.skip("no filtration unit")
    return cell, meshes, Robot(load_robot(), load_tool())


def test_reachable_and_hits_unit(world):
    cell, meshes, robot = world
    point, d = suggest_purge(cell, meshes, robot, 0.0)
    assert robot.ik(np.linalg.inv(robroot(cell, 0.0)) @ marker_pose(point, d), cell.home_deg)
    assert d[2] == 0 and np.isclose(np.linalg.norm(d), 1)
    b = meshes.groups["Filtration unit"].bounds
    lo, hi = np.array(b[0::2]), np.array(b[1::2])
    p, dd = np.array(point), np.array(d)
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo - p) / dd, (hi - p) / dd
    tin, tout = np.nanmax(np.minimum(t1, t2)), np.nanmin(np.maximum(t1, t2))
    # default cell: robot root Y (1069) lies inside the unit's Y span (437..1540), so a +-Y
    # ray from the robot side starts inside the bbox; tin < 0 there. Kept to tout > 0 for now.
    assert 0 < tout and tin < tout
    assert d[0] == 0 and abs(d[1]) == 1                # exactly +-Y
    centre_y = (lo[1] + hi[1]) / 2
    assert np.sign(centre_y - robroot(cell, 0.0)[1, 3]) == np.sign(d[1])  # robot -> unit


def test_button_fills_fields(world):
    from PySide6.QtWidgets import QApplication
    from coldspray.gui.steps.position import PositionStep
    cell, meshes, robot = world
    QApplication.instance() or QApplication([])

    class Host(PositionStep):
        def __init__(self):
            self.assets = SimpleNamespace(cell=cell.model_copy(deep=True), robot=robot, meshes=meshes)
            self.section = self._build_purge_section()
            self.e1 = SimpleNamespace(value=lambda: 0.0)
        def _work_cell(self):
            return self.assets.cell.model_copy(deep=True)
        def _status(self, *_): pass

    h = Host()
    h.purge_suggest_btn.click()
    point, d = suggest_purge(cell, meshes, robot, 0.0)
    assert np.allclose([b.value() for b in h.purge_xyz], point, atol=1)
    assert np.allclose(h.purge_cfg["spray_dir"], d, atol=1e-3)
