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


def _block_at_wrist(robot, cell, axes=None, size=300):
    """A box (vertices, faces in BASE) overlapping the wrist at home, gantry at 0, table at `axes`."""
    wrist = robroot(cell, 0.0) @ robot.link_frames(HOME_DEG)[4][:, 3]
    centre = (np.linalg.inv(base_world(cell, axes)) @ np.r_[wrist[:3] + OFFSET, 1])[:3]
    block = pv.Cube(center=centre, x_length=size, y_length=size, z_length=size).triangulate()
    return np.asarray(block.points), block.faces.reshape(-1, 4)[:, 1:]


def _bare_meshes(robot):
    return CellMeshes(static=pv.PolyData(), carriage=pv.PolyData(), robot=_links(robot))


@pytest.mark.parametrize("name", ["part", "substrate", "fixture"])
def test_contact_with_workpiece_never_ignored(name):
    """A part, substrate or fixture placed so the arm touches it in the CAD pose must still be reported."""
    robot, cell = Robot(load_robot()), load_cell()
    block = _block_at_wrist(robot, cell)
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), reach_mm=1e5, **{name: block})
    assert not any(name in p for p in checker.allowed)
    checker.set_placement(robroot(cell, 0.0))
    assert ("A5", name) in checker.check(np.array(HOME_DEG, float), {})


def test_part_positional_argument():
    robot, cell = Robot(load_robot()), load_cell()
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), None, _block_at_wrist(robot, cell),
                               HOME_DEG, 1e5)
    checker.set_placement(robroot(cell, 0.0))
    assert ("A5", "part") in checker.check(np.array(HOME_DEG, float), {})


def test_set_workpiece_none_removes_both():
    robot, cell = Robot(load_robot()), load_cell()
    block = _block_at_wrist(robot, cell)
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), part=block, substrate=block,
                               reach_mm=1e5)
    checker.set_placement(robroot(cell, 0.0))
    home = np.array(HOME_DEG, float)
    assert {("A5", "part"), ("A5", "substrate")} <= checker.check(home, {})
    checker.set_workpiece(None, None)
    assert "part" not in checker.env._objs and "substrate" not in checker.env._objs
    assert checker.check(home, {}) == set()


@pytest.mark.parametrize("axes", [{"E5": 180.0}, {"E2": 90.0}])
def test_workpiece_follows_turntable(axes):
    """Bodies placed at the wrist with the table moved are hit only with the table moved."""
    robot, cell = Robot(load_robot()), load_cell()
    block = _block_at_wrist(robot, cell, axes)
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), reach_mm=1e5)
    checker.set_placement(robroot(cell, 0.0))
    checker.set_workpiece(block, block, block)
    home = np.array(HOME_DEG, float)
    assert {("A5", "part"), ("A5", "substrate"), ("A5", "fixture")} <= checker.check(home, axes)
    assert checker.check(home, {}) == set()


def test_batch_fk_matches_robot():
    from coldspray.kinematics.robot import Robot
    from coldspray.models import load_robot, load_tool

    robot = Robot(load_robot(), load_tool())
    q = np.random.default_rng(0).uniform(-120, 120, (7, 6))
    lf, fl = robot.link_frames_batch(q), robot.flange(q)
    for i in range(len(q)):
        assert np.allclose(lf[i], robot.link_frames(q[i]), atol=1e-9)
        assert np.allclose(fl[i], robot.flange(q[i]), atol=1e-9)
