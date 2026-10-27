"""fix_e1 (per-layer gantry), KRL and Simulate honouring E1 per move."""

from dataclasses import replace

import pytest

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.gun import wrist_side
from coldspray.planner.solve_e1 import fix_e1, layer_starts
from coldspray.planner.solver import Context, find_issues
from coldspray.post.krl import to_krl


@pytest.fixture(scope="module")
def setup():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = 2500.0
    path = plan_part(load_part("test_square"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                     wrist=wrist_side(robot.tool))[1]
    return cell, robot, path


def test_nothing_to_fix_returns_same_path(setup):
    cell, robot, path = setup
    assert fix_e1(path, [], Context(robot, cell, log=lambda _: None)) is path


def test_unreachable_layer_gets_another_e1(setup):
    cell, robot, path = setup
    lo, hi = cell.gantry.limits_mm
    cell2 = cell.model_copy(deep=True)
    cell2.gantry.position_mm = lo                      # start at one end: probably short
    ctx = Context(robot, cell2, log=lambda _: None)
    _, _, issues = find_issues(path, ctx)
    if not issues:
        pytest.skip("layer reachable from this E1")
    new = fix_e1(path, issues, ctx)
    assert new is not path
    assert any("E1" in m.axes for m in new.moves)
    assert len(find_issues(new, ctx)[2]) < len(issues)


def test_krl_writes_e1_per_move(setup):
    cell, robot, path = setup
    proc = load_process()
    base = to_krl(path, robot, cell, proc)
    assert "E1 2500.000" in base
    starts = layer_starts(path)
    s = starts[-1] if len(starts) > 1 else 0
    moves = list(path.moves)
    moves[s] = replace(moves[s], kind="PTP", axes={"E1": 2000.0})
    out = to_krl(replace(path, moves=moves), robot, cell, proc)
    assert "E1 2000.000" in out
