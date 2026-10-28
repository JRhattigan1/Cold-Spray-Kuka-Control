"""Acceptance: every infill pattern on test_block plans, auto-positions to full
reach (every move end point, not a sample) and exports KRL.

Move counts are snapshots: a deliberate planner change updates them."""

import numpy as np
import pytest

from coldspray.cell import robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.patterns import PATTERNS
from coldspray.planner.position import auto_position, layer_tilts, world_poses
from coldspray.post.krl import to_krl

MOVES = {"concentric": 1564, "islands": 1444, "oneway": 1300, "spiral": 1564, "zigzag": 910}


def unreachable(path, robot, cell, e1) -> int:
    """Number of move end points with no IK solution."""
    world = world_poses(path, cell)
    q = robot.ik_path(np.linalg.inv(robroot(cell, e1)) @ world, seed_deg=cell.home_deg)
    return int(np.isnan(q[:, 0]).sum())


def plan_positioned(part, proc, mode, tilts, **kw):
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    plan = lambda tilt: plan_part(part, proc, mode, tilt_deg=tilt, cell=cell, **kw)[1]
    pos = auto_position(plan, robot, cell, tilts, log=lambda _: None)
    if cell.gantry and pos.e1_mm is not None:
        cell.gantry.position_mm = pos.e1_mm
    path = plan_part(part, proc, mode, tilt_deg=pos.tilt_deg, cell=cell, roll_deg=pos.roll_deg, **kw)[1]
    return pos, path, robot, cell


@pytest.mark.parametrize("pattern", sorted(PATTERNS))
def test_pattern_end_to_end(pattern):
    proc = load_process().model_copy(update={"infill_pattern": pattern})
    cell = load_cell()
    pos, path, robot, cell = plan_positioned(load_part("test_block"), proc, "layers", layer_tilts(cell))
    assert pos.reach == 1.0
    assert unreachable(path, robot, cell, pos.e1_mm) == 0
    assert len(path.moves) == MOVES[pattern]
    text = to_krl(path, robot, cell, proc)
    assert text.strip()
