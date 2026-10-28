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
    assert tuple(d) == (0.0, 1.0, 0.0)                 # back of the cell: exactly +Y
    from coldspray.planner.purge import _unit_mesh, trimesh_dist
    tm = _unit_mesh(meshes.groups["Filtration unit"])
    hits, _, _ = tm.ray.intersects_location([point], [d])
    tin = min((hits - np.array(point)) @ np.array(d)) if len(hits) else -1
    assert 0 < tin <= 1500                              # the spray enters the unit's mesh
    assert trimesh_dist(tm, point) >= 150               # point outside, standoff clear
    assert tin >= 150


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
