"""fix_angle: tilts the gun on bad stretches only, positions fixed, blended."""

import numpy as np
import pytest

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.gun import wrist_side
from coldspray.planner.solve_angle import fix_angle
from coldspray.planner.solver import Context, SolveOptions, find_issues


@pytest.fixture(scope="module")
def setup():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = 2500.0
    path = plan_part(load_part("test_square"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                     wrist=wrist_side(robot.tool))[1]
    return cell, robot, path


def _ctx(setup, checker, **o):
    cell, robot, _ = setup
    return Context(robot, cell, checker=checker, opts=SolveOptions(**o), log=lambda _: None)


def test_no_issues_returns_same_path(setup):
    ctx = _ctx(setup, None)
    assert fix_angle(setup[2], [], ctx) is setup[2]


def test_angle_offset_fixes_stretch_and_keeps_positions(setup):
    cell, robot, path = setup
    s0 = path.sample(5.0, 5.0)
    j0 = robot.ik_path(__import__("coldspray.planner.solver", fromlist=["x"]).robot_targets(s0, cell),
                       seed_deg=cell.home_deg)
    lo, hi = 20, 30

    class Checker:
        """Collides in samples lo..hi unless joint A6 differs by > 8 degrees from the planned one."""
        def check(self, q, axes):
            i = int(np.argmin(np.abs(j0[:, 0] - q[0]) + np.abs(j0[:, 1] - q[1]) + np.abs(j0[:, 2] - q[2])))
            if lo <= i <= hi and abs(q[5] - j0[i, 5]) < 8.0:
                return {("A6", "Wall")}
            return set()

    ctx = _ctx(setup, Checker(), lean_max_deg=0.0)
    ctx.seed = np.asarray(cell.home_deg, float)   # same IK start, no HOME legs (S9H): the synthetic
    _, _, issues = find_issues(path, ctx)         # checker would also hit the HOME -> first sweep
    assert issues, "test needs a real problem"
    out = fix_angle(path, issues, ctx)
    assert out is not path
    _, _, after = find_issues(out, ctx)
    assert len(after) < len(issues)
    for a, b in zip(path.moves, out.moves):
        assert np.allclose(a.pose[:3, 3], b.pose[:3, 3])
    turns = [np.degrees(np.arccos(np.clip((np.trace(a.pose[:3, :3].T @ b.pose[:3, :3]) - 1) / 2, -1, 1)))
             for a, b in zip(path.moves, out.moves)]
    assert max(turns) <= 30.0 + 1e-6 and max(turns) > 0
    assert turns[0] == 0 or turns[0] < max(turns)      # blended, not applied everywhere


def test_unfixable_returns_same_object(setup):
    from dataclasses import replace
    cell, robot, path = setup
    moves = list(path.moves)
    pose = moves[5].pose.copy()
    pose[:3, 3] += [3000.0, 0.0, 0.0]
    moves[5] = replace(moves[5], pose=pose)
    bad = replace(path, moves=moves)
    ctx = _ctx(setup, None)
    _, _, issues = find_issues(bad, ctx)
    assert issues and fix_angle(bad, issues, ctx) is bad
