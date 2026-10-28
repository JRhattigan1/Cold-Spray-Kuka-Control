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


def test_simulate_uses_cell_e1_before_the_first_e1_like_the_krl(setup):
    """S7K/18: moves before the first move that sets E1 run at the cell's E1 (as the KRL has
    them), not back-filled from the last E1 in the path."""
    from coldspray.cell import axis_defaults
    cell, _, path = setup
    s0 = layer_starts(path)[-1]
    moves = list(path.moves)
    moves[s0] = replace(moves[s0], axes=dict(moves[s0].axes) | {"E1": 2000.0})
    p = replace(path, moves=moves)
    s = p.sample(5.0, 2.0, axis_defaults(cell))
    before = s.move_index < s0
    assert before.any() and (s.axes["E1"][before] == cell.gantry.position_mm).all()
    assert (s.axes["E1"][s.move_index > s0] == 2000.0).all()


def test_fix_e1_scores_only_the_moves_its_e1_governs(setup, monkeypatch):
    """S7K/18: layer k is scored from its carrier (not its first, sprayed, moves) up to the next
    layer with a carrier, so a layer that sprays throughout is scored with the one before it."""
    import coldspray.planner.solve_e1 as se
    from coldspray.planner.solver import Issue
    from coldspray.planner.solve_e1 import e1_carriers
    cell, robot, path = setup
    starts = layer_starts(path)
    if len(starts) < 3:
        pytest.skip("needs three layers")
    ends = starts[1:] + [len(path.moves)]
    moves = [replace(m, gun_on=True) if starts[1] <= i < ends[1] else m for i, m in enumerate(path.moves)]
    p = replace(path, moves=moves)
    car = e1_carriers(p, starts)
    assert car[1] is None and car[0] is not None and car[2] is not None
    spans = []
    monkeypatch.setattr(se, "find_issues", lambda *a: None)
    monkeypatch.setattr(se, "issues_in", lambda p, ctx, lo, hi: spans.append((lo, hi)) or [])
    fix_e1(p, [Issue(0, starts[1], "unreachable")], Context(robot, cell, log=lambda _: None))
    assert spans and set(spans) == {(car[0], car[2] - 1)}
