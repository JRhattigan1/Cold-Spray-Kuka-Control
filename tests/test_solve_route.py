"""fix_route: lift / via point on non-spray moves only."""

from dataclasses import replace

import numpy as np
import pytest

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.gun import wrist_side
from coldspray.planner.solve_route import fix_route
from coldspray.planner.solver import Context, find_issues, robot_targets


@pytest.fixture(scope="module")
def setup():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = 2500.0
    path = plan_part(load_part("test_square"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                     wrist=wrist_side(robot.tool))[1]
    return cell, robot, path


def _ctx(setup, checker=None):
    cell, robot, _ = setup
    return Context(robot, cell, checker=checker, log=lambda _: None)


def _free_move(path):
    return next(i for i, m in enumerate(path.moves) if i > 0 and not m.gun_on and i + 1 < len(path.moves)
                and not path.moves[i + 1].gun_on)


def test_clean_path_unchanged(setup):
    path = setup[2]
    ctx = _ctx(setup)
    _, _, issues = find_issues(path, ctx)
    assert fix_route(path, issues, ctx) is path


def test_spray_moves_untouched_and_unfixable_returns_same(setup):
    cell, robot, path = setup
    from coldspray.planner.solver import Issue
    k = next(i for i, m in enumerate(path.moves) if m.gun_on)
    ctx = _ctx(setup)
    assert fix_route(path, [Issue(0, k, "collision")], ctx) is path


def _sideways_free_move(path):
    """A free move followed by a free move that travels across (not along the spray axis)."""
    for i in range(1, len(path.moves) - 1):
        a, b, c = path.moves[i - 1], path.moves[i], path.moves[i + 1]
        d = b.pose[:3, 3] - a.pose[:3, 3]
        across = d - (d @ b.pose[:3, 0]) * b.pose[:3, 0]
        if not b.gun_on and not c.gun_on and np.linalg.norm(across) > 20.0:
            return i
    raise AssertionError("no sideways free move")


def test_lifts_a_free_move_out_of_a_zone(setup):
    cell, robot, path = setup
    k = _sideways_free_move(path)
    s, _, _ = find_issues(path, _ctx(setup))
    idx = np.flatnonzero(s.move_index == k)
    centre = robot_targets(s, cell)[idx[len(idx) // 2], :3, 3]       # robot frame, mid-move

    class Zone:            # an obstacle: a small ball round the gun tip there (layers are 0.5 mm apart)
        def check(self, q, axes):
            return {("gun", "zone")} if np.linalg.norm(robot.fk(q)[:3, 3] - centre) < 0.2 else set()

    ctx = _ctx(setup, Zone())
    _, _, issues = find_issues(path, ctx)
    assert issues and k in {i.move for i in issues} and all(not path.moves[i.move].gun_on for i in issues)
    new = fix_route(path, issues, ctx)
    assert new is not path
    _, _, after = find_issues(new, ctx)
    assert not after
    spray_old = [m.pose for m in path.moves if m.gun_on]
    spray_new = [m.pose for m in new.moves if m.gun_on]
    assert len(spray_old) == len(spray_new) and all(np.allclose(p, q) for p, q in zip(spray_old, spray_new))
    assert [m.gun_on for m in new.moves].count(True) == [m.gun_on for m in path.moves].count(True)
