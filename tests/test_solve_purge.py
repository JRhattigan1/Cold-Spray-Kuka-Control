"""Lever `purge` (S7G/2): move the purge point in X/Y/Z, spray direction kept."""

import numpy as np
import pytest

import coldspray.planner.solve_purge as sp
import coldspray.planner.solver as solver
from coldspray.cell import base_world
from coldspray.kinematics.robot import Robot
from coldspray.models import PurgeConfig, load_cell, load_robot, load_tool
from coldspray.planner.purge import add_purge, move_purge
from coldspray.planner.solver import Context, Issue, solve
from coldspray.toolpath.model import Move, Toolpath, spray_frame


def _path():
    at = lambda x, z: spray_frame((x, 0.0, z), (0, 0, -1), (1, 0, 0))
    moves = []
    for k in range(3):
        z = 30.0 + 2 * k
        moves += [Move(at(60, z + 10), 250, kind="PTP" if k == 0 else "LIN",
                       axes={"E2": 0.0, "E5": 0.0} if k == 0 else {}, tag="approach"),
                  Move(at(60, z), 20, gun_on=True, tag="infill"),
                  Move(at(-50, z), 20, gun_on=True, tag="infill"),
                  Move(at(60, z + 10), 250, tag="approach")]
    return Toolpath("p", moves)


@pytest.fixture(scope="module")
def setup():
    cell = load_cell().model_copy(deep=True)
    pt = base_world(cell)[:3, 3] + [-200.0, 0.0, 30.0]
    cell.purge = PurgeConfig(enabled=True, point=tuple(pt), spray_dir=(0.0, 0.0, -1.0), every_n_layers=1)
    return cell, Robot(load_robot(), load_tool()), add_purge(_path(), cell)


def _world(cell, path, i):
    return (base_world(cell, path.axis_values()[i]) @ path.moves[i].pose)


def test_move_purge_translates_visits_and_keeps_direction(setup):
    cell, _, path = setup
    new_pt = np.asarray(cell.purge.point) + [0.0, 150.0, 100.0]
    p = move_purge(path, cell, new_pt)
    old_v = [i for i, m in enumerate(path.moves) if m.tag == "purge" and m.dwell_s > 0]
    new_v = [i for i, m in enumerate(p.moves) if m.tag == "purge" and m.dwell_s > 0]
    assert len(old_v) == len(new_v) >= 2
    for a, b in zip(old_v, new_v):
        wa, wb = _world(cell, path, a), _world(cell, p, b)
        assert np.allclose(wb[:3, 3], new_pt, atol=1e-6)
        assert np.allclose(wa[:3, :3], wb[:3, :3])
    assert len([m for m in p.moves if m.tag != "purge"]) == len(_path().moves)
    assert cell.purge.point != tuple(new_pt)                 # the cell itself is untouched


def test_no_purge_issues_returns_same_path(setup):
    cell, robot, path = setup
    k = next(i for i, m in enumerate(path.moves) if m.tag == "infill")
    assert sp.fix_purge(path, [Issue(0, k, "unreachable")], Context(robot, cell, log=lambda _: None)) is path


def test_fix_purge_moves_the_point_until_purge_moves_are_clear(setup, monkeypatch):
    """Fake check: a purge pose is bad unless its WORLD x is >= the old point + 150."""
    cell, robot, path = setup
    x_ok = cell.purge.point[0] + 150.0

    def fake(p, ctx):
        bad = [Issue(i, i, "unreachable") for i, m in enumerate(p.moves)
               if m.tag == "purge" and m.dwell_s > 0 and _world(cell, p, i)[0, 3] < x_ok]
        return None, None, bad

    monkeypatch.setattr(sp, "count_issues", lambda p, ctx, limit=None: fake(p, ctx)[2])
    issues = fake(path, None)[2]
    ctx = Context(robot, cell, log=lambda _: None)
    new = sp.fix_purge(path, issues, ctx)
    assert new is not path and not fake(new, None)[2]
    x, y, z = ctx.suggested["purge_point"]
    assert x == pytest.approx(cell.purge.point[0] + 200.0) and y == cell.purge.point[1]
    assert z == cell.purge.point[2]                         # smallest move that works: X only


def test_solve_reports_the_purge_point_and_logs(setup, monkeypatch):
    cell, robot, path = setup
    k = next(i for i, m in enumerate(path.moves) if m.tag == "purge")

    def lever(p, issues, ctx):
        ctx.suggested = {"purge_point": (1.0, 2.0, 3.0)}
        return p.__class__(p.name, list(p.moves[:k]) + list(p.moves[k + 1:]), p.skipped_points)

    calls = iter([[Issue(0, k, "unreachable")], []])
    monkeypatch.setattr(solver, "find_issues", lambda p, ctx: (None, None, next(calls)))
    monkeypatch.setattr(solver, "_levers", lambda: [("purge", lever, lambda o: o.purge)])
    logs = []
    _, report = solve(path, Context(robot, cell, log=logs.append))
    assert report.suggested == {"purge_point": (1.0, 2.0, 3.0)}
    assert any("moving the purge point" in s for s in logs)


def test_purge_lever_is_registered():
    names = [n for n, _, _ in solver._levers()]
    assert "purge" in names and names.index("gantry") < names.index("e1")


def test_suggested_purge_point_row():
    from coldspray.gui.checks import solve_checks
    from coldspray.planner.solver import SolveReport
    rows = solve_checks(SolveReport(fixed=["purge: 2 of 2"], suggested={"purge_point": (10.0, 20.4, 30.6)}))
    assert any(r.title == "Suggested purge point = (10, 20, 31) mm" for r in rows)
