"""fix_arm: configuration chosen along the path by dynamic programming."""

import numpy as np
import pytest

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.gun import wrist_side
from coldspray.planner.solve_arm import fix_arm
from coldspray.planner.solver import Context, Issue, robot_targets


@pytest.fixture(scope="module")
def setup():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = 2500.0
    path = plan_part(load_part("test_square"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                     wrist=wrist_side(robot.tool))[1]
    return cell, robot, path


def _quiet(robot, cell, **kw):
    return Context(robot, cell, log=lambda _: None, **kw)


def test_ik_all_contains_ik(setup):
    cell, robot, path = setup
    pose = robot_targets(path.sample(50.0), cell)[3]
    allsol, near = robot.ik_all(pose), robot.ik(pose)
    assert len(allsol) == len(near) and near
    for q in near:
        assert any(np.allclose(q, a) for a in allsol)


def test_no_issues_returns_same_path(setup):
    cell, robot, path = setup
    assert fix_arm(path, [], _quiet(robot, cell)) is path


def test_hint_is_valid_and_continuous_while_spraying(setup):
    cell, robot, path = setup
    out = fix_arm(path, [Issue(0, 0, "collision")], _quiet(robot, cell))
    h = out.joint_hint
    assert out.moves is path.moves and h.shape == (len(path.moves), 6)
    assert not np.isnan(h).any() and all(robot.within_limits(q) for q in h)
    on = np.array([m.gun_on for m in path.moves])
    d = np.abs(np.diff(h, axis=0)).max(axis=1)
    assert (d[on[1:] & on[:-1]] < 90).all()


def test_avoids_colliding_solution(setup):
    cell, robot, path = setup
    base = fix_arm(path, [Issue(0, 0, "collision")], _quiet(robot, cell)).joint_hint
    k = len(path.moves) // 2
    blocked = base[k].copy()

    class Checker:
        def check(self, q, axes):
            return {("A3", "Wall")} if np.allclose(q, blocked, atol=1e-6) else set()

    out = fix_arm(path, [Issue(0, k, "collision")], _quiet(robot, cell, checker=Checker()))
    s = path.sample(5.0)
    last = max(i for i in range(len(s)) if s.move_index[i] == k)
    if len(robot.ik_all(robot_targets(s, cell)[last])) > 1:
        assert not np.allclose(out.joint_hint[k], blocked, atol=1e-6)


def test_ik_path_seeds_from_hint(setup):
    cell, robot, path = setup
    t = robot_targets(path.sample(50.0), cell)[:3]
    first = robot.ik_all(t[0])[-1]
    hint = np.full((3, 6), np.nan)
    hint[0] = first
    assert np.allclose(robot.ik_path(t, hint=hint)[0], first)
