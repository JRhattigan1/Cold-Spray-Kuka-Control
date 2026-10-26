import numpy as np
import pytest

from coldspray.cell import base_in_robot, robroot
from coldspray.models import load_cell


def test_robroot_matches_cad_placement():
    cell = load_cell()
    root = robroot(cell, e1=0.0)
    assert np.allclose(root[:3, 3], [1845.75, 1069.0, 3196.33])
    assert np.allclose(root[:3, 2], [0, -1, 0])  # A1 axis points along world -Y


def test_gantry_moves_robot_along_x():
    cell = load_cell()
    d = robroot(cell, -1000.0)[:3, 3] - robroot(cell, 0.0)[:3, 3]
    assert np.allclose(d, [-1000, 0, 0])


def test_gantry_limits_enforced():
    with pytest.raises(ValueError):
        robroot(load_cell(), e1=1e6)


def test_base_round_trip():
    cell = load_cell()
    world = robroot(cell) @ base_in_robot(cell)
    assert np.allclose(world[:3, 3], [cell.base.x, cell.base.y, cell.base.z])
