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
    E0 = {cell.gantry.axis: 0.0}
    assert ("A5", name) in checker.check(np.array(HOME_DEG, float), E0)


def test_part_positional_argument():
    robot, cell = Robot(load_robot()), load_cell()
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), None, _block_at_wrist(robot, cell),
                               HOME_DEG, 1e5)
    E0 = {cell.gantry.axis: 0.0}
    assert ("A5", "part") in checker.check(np.array(HOME_DEG, float), E0)


def test_set_workpiece_none_removes_both():
    robot, cell = Robot(load_robot()), load_cell()
    block = _block_at_wrist(robot, cell)
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), part=block, substrate=block,
                               reach_mm=1e5)
    E0 = {cell.gantry.axis: 0.0}
    home = np.array(HOME_DEG, float)
    assert {("A5", "part"), ("A5", "substrate")} <= checker.check(home, E0)
    checker.set_workpiece(None, None)
    assert "part" not in checker.env._objs and "substrate" not in checker.env._objs
    assert checker.check(home, E0) == set()


@pytest.mark.parametrize("axes", [{"E5": 180.0}, {"E2": 90.0}])
def test_workpiece_follows_turntable(axes):
    """Bodies placed at the wrist with the table moved are hit only with the table moved."""
    robot, cell = Robot(load_robot()), load_cell()
    block = _block_at_wrist(robot, cell, axes)
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), reach_mm=1e5)
    E0 = {cell.gantry.axis: 0.0}
    checker.set_workpiece(block, block, block)
    home = np.array(HOME_DEG, float)
    assert {("A5", "part"), ("A5", "substrate"), ("A5", "fixture")} <= checker.check(home, {**E0, **axes})
    assert checker.check(home, E0) == set()


def test_batch_fk_matches_robot():
    from coldspray.kinematics.robot import Robot
    from coldspray.models import load_robot, load_tool

    robot = Robot(load_robot(), load_tool())
    q = np.random.default_rng(0).uniform(-120, 120, (7, 6))
    lf, fl = robot.link_frames_batch(q), robot.flange(q)
    for i in range(len(q)):
        assert np.allclose(lf[i], robot.link_frames(q[i]), atol=1e-9)
        assert np.allclose(fl[i], robot.flange(q[i]), atol=1e-9)


def test_check_without_e1_uses_configured_root_not_stale():
    robot, cell = Robot(load_robot()), load_cell()
    if cell.gantry is None:
        pytest.skip("no gantry")
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), reach_mm=1e5)
    lo, hi = cell.gantry.limits_mm
    other = lo if abs(lo - cell.gantry.position_mm) > 1 else hi
    checker.check(np.array(HOME_DEG, float), {cell.gantry.axis: other})
    assert not np.allclose(checker.world_from_robot, robroot(cell))
    checker.check(np.array(HOME_DEG, float), {})
    assert np.allclose(checker.world_from_robot, robroot(cell))


def test_no_link_vs_cell_allowed_with_bare_meshes():
    robot, cell = Robot(load_robot()), load_cell()
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), reach_mm=1e5)
    assert not any(b == "cell" for _, b in checker.allowed)
    assert isinstance(checker.allowed_text(), str)


def _carriage_checker(robot, cell):
    """A box carriage under the base, touching the base box and A1 in the CAD pose (gantry at 0)."""
    root = robroot(cell, 0.0)
    a2 = root @ robot.link_frames(HOME_DEG)[1][:, 3]
    centre = root[:3, 3] + root[:3, :3] @ np.array([0.0, 0.0, -50.0])
    box = pv.Cube(center=centre, x_length=300, y_length=300, z_length=300)
    # second box where A2 ends up with the arm swung round
    a2_swung = root @ robot.link_frames(SWUNG)[1][:, 3]
    far = pv.Cube(center=a2_swung[:3] + [0.0, 0.0, 150.0], x_length=300, y_length=300, z_length=300)
    meshes = CellMeshes(static=pv.PolyData(), carriage=box.merge(far), robot=_links(robot))
    return CollisionChecker(robot, cell, meshes, reach_mm=1e5), a2


def test_carriage_cad_contacts_allowed_and_reported_elsewhere():
    robot, cell = Robot(load_robot()), load_cell()
    checker, _ = _carriage_checker(robot, cell)
    E0 = {cell.gantry.axis: 0.0}
    assert checker.check(np.array(HOME_DEG, float), E0) == set()
    assert any("carriage" in p for p in checker.allowed)
    pairs = checker.check(np.array(SWUNG, float), E0)
    assert any(p[1] == "carriage" and p[0] in ("A2", "A3") for p in pairs), pairs


def test_carriage_moves_with_e1():
    robot, cell = Robot(load_robot()), load_cell()
    checker, _ = _carriage_checker(robot, cell)
    lo, hi = cell.gantry.limits_mm
    e1 = float(np.clip(500.0, lo, hi))
    checker.check(np.array(HOME_DEG, float), {cell.gantry.axis: e1})
    m = checker.env._objs["carriage"]["obj"].getTranslation()
    assert np.allclose(m, np.asarray(cell.gantry.direction) * e1)


def test_carriage_never_allowed_for_tool():
    from coldspray.sim import collision
    assert {"gun", "nozzle"} <= set(collision.TOOL) and "carriage" in collision.NEVER_ALLOWED_FOR_TOOL


def test_moved_cell_e1_does_not_move_or_break_a_built_checker():
    """S7K/21: a sample without E1 runs at the E1 the checker was built at, not the live
    cell.gantry.position_mm (moved in the GUI while a check runs: used to raise ValueError)."""
    from coldspray.sim.collision import ROBOT_REACH_MM
    robot, cell = Robot(load_robot()), load_cell().model_copy(deep=True)
    if cell.gantry is None:
        pytest.skip("no gantry")
    checker = CollisionChecker(robot, cell, _bare_meshes(robot), reach_mm=ROBOT_REACH_MM + 10)
    root = checker.world_from_robot.copy()
    lo, hi = cell.gantry.limits_mm
    cell.gantry.position_mm = lo if abs(lo - checker.e1) > abs(hi - checker.e1) else hi
    checker.check(np.array(HOME_DEG, float), {})
    assert np.allclose(checker.world_from_robot, root)


def test_part_not_checked_in_layers_mode():
    """S7K/5: Layers builds the part up, so it isn't there to hit; Revolve / Surface coat it."""
    from coldspray.sim.collision import part_for_mode
    part = (np.zeros((3, 3)), np.array([[0, 1, 2]]))
    assert part_for_mode("layers", part) is None
    assert part_for_mode("revolve", part) is part and part_for_mode("surface", part) is part
