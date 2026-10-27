"""Acceptance: collision check of planned, auto-positioned toolpaths against the
cell CAD, the way the Simulate step runs it (gui/steps/toolpath.py generate ->
_start_collision_check): samples every 2 mm / 5 deg, IK seeded at home, checker
with gun, part (build body) and fixture (other bodies), every CHECK_EVERY-th sample.

A real collision found here is marked xfail(strict=True): fixing it is product work."""

import numpy as np
import pytest

from coldspray.cad.part import build_mesh, other_mesh
from coldspray.cell import base_world, robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import PROJECT_ROOT, PurgeConfig, load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.position import auto_position, layer_tilts

CACHE = load_cell().cad_cache
pytestmark = pytest.mark.skipif(not CACHE or not (PROJECT_ROOT / CACHE).with_suffix(".json").exists(),
                                reason="no cell CAD cache")
CHECK_EVERY = 3        # as gui/steps/simulate.py
PURGE = PurgeConfig(point=(-884.56, 400.0, 2000.0))   # the example in cell.yaml


@pytest.fixture(scope="module")
def assets():
    from coldspray.sim.scene import load_assets
    return load_assets(log=lambda _: None)


def plan(part, proc, mode, tilts, purge=None):
    """Auto-positioned toolpath, as tests/test_patterns_e2e.plan_positioned but with a cell purge."""
    cell = load_cell()
    cell.purge = purge
    robot = Robot(load_robot(), load_tool())
    pos = auto_position(lambda t: plan_part(part, proc, mode, tilt_deg=t, cell=cell)[1],
                        robot, cell, tilts, log=lambda _: None)
    assert pos.reach == 1.0
    if cell.gantry and pos.e1_mm is not None:
        cell.gantry.position_mm = pos.e1_mm
    path = plan_part(part, proc, mode, tilt_deg=pos.tilt_deg, cell=cell, roll_deg=pos.roll_deg)[1]
    return path, robot, cell


def collisions(assets, part, path, robot, cell):
    """Collision points of `path`, built like Simulate's _start_collision_check."""
    from coldspray.sim.collision import CollisionChecker
    from coldspray.sim.scene import make_gun

    s = path.sample(2.0, 5.0)
    robot_from_world = np.linalg.inv(robroot(cell))
    targets = [robot_from_world @ base_world(cell, s.axes_at(i)) @ s.poses[i] for i in range(len(s))]
    joints = robot.ik_path(targets, seed_deg=cell.home_deg)
    # Surface: some interpolated samples have no IK (reach is checked at move ends); Simulate skips them
    checker = CollisionChecker(robot, cell, assets.meshes, make_gun(load_tool(), assets.gun_mesh),
                               build_mesh(part), cell.robot_cad_pose_deg, fixture=other_mesh(part))
    hits = checker.check_path(joints, s.axes_at, every=CHECK_EVERY)
    tags = {path.moves[s.move_index[h.sample]].tag for h in hits}
    return hits, tags


def report(hits, tags):
    pairs = sorted({p for h in hits for p in h.pairs})
    return f"{len(hits)} collision points, tags {sorted(tags)}, pairs {pairs}"


@pytest.mark.xfail(strict=True, reason="A2/A3 hit Filtration unit along the whole path (2145 points)")
def test_block_zigzag_no_collision(assets):
    part = load_part("test_block")
    path, robot, cell = plan(part, load_process(), "layers", layer_tilts(load_cell()))
    hits, tags = collisions(assets, part, path, robot, cell)
    assert not hits, report(hits, tags)


@pytest.mark.xfail(strict=True, reason="purge/approach moves: arm, gun and nozzle hit faceplate and Filtration unit")
def test_block_with_purge_no_collision(assets):
    part = load_part("test_block")
    path, robot, cell = plan(part, load_process(), "layers", layer_tilts(load_cell()), purge=PURGE)
    assert any(m.tag == "purge" for m in path.moves)
    hits, tags = collisions(assets, part, path, robot, cell)
    assert not hits, report(hits, tags)


@pytest.mark.xfail(strict=True, reason="A2/A3 hit Filtration unit; wrist, gun, nozzle hit faceplate; nozzle hits fixture")
def test_repair_surface_no_collision(assets):
    from test_surface_e2e import TILTS, repair_patch_faces
    part = load_part("test_repair")
    part.coat_faces = repair_patch_faces(part)
    path, robot, cell = plan(part, load_process(), "surface", TILTS)
    hits, tags = collisions(assets, part, path, robot, cell)
    assert not hits, report(hits, tags)
