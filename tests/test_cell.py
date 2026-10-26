import numpy as np
import pytest

from coldspray.cell import base_in_robot, base_world, robroot
from coldspray.models import load_cell


def test_robroot_matches_cad_placement():
    cell = load_cell()
    root = robroot(cell, e1=0.0)
    assert np.allclose(root[:3, 3], [1845.75, 1069.0, 3196.33])
    assert np.allclose(root[:3, 2], [0, -1, 0])  # A1 axis points along world -Y


def test_positive_e1_moves_robot_towards_table():
    cell = load_cell()
    d = robroot(cell, 1000.0)[:3, 3] - robroot(cell, 0.0)[:3, 3]
    assert np.allclose(d, [-1000, 0, 0])
    assert cell.base.x < cell.robroot.x  # the turntable is on the -X side


def test_gantry_limits_enforced():
    with pytest.raises(ValueError):
        robroot(load_cell(), e1=1e6)


def test_tilt_90_lines_faceplate_up_with_tailstock():
    cell = load_cell()
    b = base_world(cell, {"E2": 90.0})
    assert np.allclose(b[:3, 2], [1, 0, 0])          # faceplate normal points at the tailstock
    assert np.allclose(b[1:3, 3], [-421.0, 1100.0])  # on the tailstock centre line (y, z)


def test_rotation_turns_faceplate_about_its_centre():
    cell = load_cell()
    b0, b90 = base_world(cell, {"E5": 0.0}), base_world(cell, {"E5": 90.0})
    assert np.allclose(b0[:3, 3], b90[:3, 3])        # centre stays put
    assert np.allclose(b90[:3, 0], [0, 1, 0])        # base X turned to world +Y


def test_turntable_limits_enforced():
    with pytest.raises(ValueError):
        base_world(load_cell(), {"E2": 120.0})


def test_base_round_trip():
    cell = load_cell()
    world = robroot(cell) @ base_in_robot(cell)
    assert np.allclose(world[:3, 3], [cell.base.x, cell.base.y, cell.base.z])
