import numpy as np
import pytest
import pyvista as pv

from coldspray.cell import base_world, robroot
from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.models import load_cell, load_robot
from coldspray.sim.cell_model import CellMeshes
from coldspray.sim.collision import CollisionChecker

SWUNG = [180, -90, 90, 0, 0, 0]  # arm turned to point backwards
# Obstacles overlap the wrist box partly: the check finds surfaces crossing.
OFFSET = np.array([150.0, 0.0, 0.0])


def _links(robot):
    """Robot links as 200 mm boxes at their CAD (home) position."""
    frames = robot.link_frames(HOME_DEG)
    return [pv.Box(bounds=(-100, 100, -100, 100, 0, 200))] + [
        pv.Cube(center=f[:3, 3], x_length=200, y_length=200, z_length=200) for f in frames]


@pytest.fixture(scope="module")
def checker():
    """A wall where the wrist ends up when A1 swings round, nowhere near it at home."""
    robot, cell = Robot(load_robot()), load_cell()
    wrist = robroot(cell) @ robot.link_frames(SWUNG)[4][:, 3]
    wall = pv.Cube(center=wrist[:3] + OFFSET, x_length=400, y_length=400, z_length=400)
    meshes = CellMeshes(static=wall, carriage=pv.PolyData(), robot=_links(robot))
    return CollisionChecker(robot, cell, meshes, reach_mm=1e5)


def test_no_collision_at_home(checker):
    assert checker.check(np.array(HOME_DEG, float), {}) == set()


def test_arm_hits_wall(checker):
    pairs = checker.check(np.array(SWUNG, float), {})
    assert ("A5", "cell") in pairs


def test_check_path_reports_samples(checker):
    joints = np.array([HOME_DEG, [90, -90, 90, 0, 0, 0], SWUNG, [np.nan] * 6], float)
    hits = checker.check_path(joints, lambda i: {})
    assert [h.sample for h in hits] == [2]


def test_contact_with_part_never_ignored():
    """A part placed so the arm touches it in the CAD pose must still be reported."""
    robot, cell = Robot(load_robot()), load_cell()
    wrist = robroot(cell, 0.0) @ robot.link_frames(HOME_DEG)[4][:, 3]
    centre = (np.linalg.inv(base_world(cell)) @ np.r_[wrist[:3] + OFFSET, 1])[:3]
    block = pv.Cube(center=centre, x_length=300, y_length=300, z_length=300).triangulate()
    part = (np.asarray(block.points), block.faces.reshape(-1, 4)[:, 1:])
    meshes = CellMeshes(static=pv.PolyData(), carriage=pv.PolyData(), robot=_links(robot))
    checker = CollisionChecker(robot, cell, meshes, part=part, reach_mm=1e5)
    checker.set_placement(robroot(cell, 0.0))
    assert ("A5", "part") in checker.check(np.array(HOME_DEG, float), {})
