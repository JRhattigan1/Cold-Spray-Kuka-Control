"""Solver framework: finding problem points and running levers (fake levers, fake checker)."""

from dataclasses import replace

import numpy as np
import pytest

import coldspray.planner.solver as solver
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.gun import wrist_side
from coldspray.planner.solver import Context, SolveOptions, find_issues, solve


@pytest.fixture(scope="module")
def setup():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = 2500.0
    path = plan_part(load_part("test_square"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                     wrist=wrist_side(robot.tool))[1]
    return cell, robot, path


def _far(path, k):
    """Copy with move k moved 3 m away: unreachable."""
    moves = list(path.moves)
    pose = moves[k].pose.copy()
    pose[:3, 3] += [3000.0, 0.0, 0.0]
    moves[k] = replace(moves[k], pose=pose)
    return replace(path, moves=moves)


def test_reachable_path_has_no_issues(setup):
    cell, robot, path = setup
    _, joints, issues = find_issues(path, Context(robot, cell, log=lambda _: None))
    assert not issues and not np.isnan(joints).any()


def test_unreachable_move_is_reported(setup):
    cell, robot, path = setup
    _, _, issues = find_issues(_far(path, 5), Context(robot, cell, log=lambda _: None))
    assert issues and {i.kind for i in issues} == {"unreachable"} and 5 in {i.move for i in issues}


def test_collisions_come_from_the_checker(setup):
    cell, robot, path = setup

    class Checker:
        def check(self, q, axes):
            return {("A3", "Wall")} if q[0] > -1000 else set()      # every sample

    _, _, issues = find_issues(path, Context(robot, cell, checker=Checker(), log=lambda _: None))
    assert issues and all(i.kind == "collision" and i.detail == "A3/Wall" for i in issues)


def test_solve_keeps_a_lever_only_if_it_helps(setup, monkeypatch):
    cell, robot, path = setup
    bad = _far(path, 5)
    calls = []

    def useless(p, issues, ctx):
        calls.append("useless")
        return _far(p, 6)                   # makes it worse: must be dropped

    def fixer(p, issues, ctx):
        calls.append("fixer")
        return path                         # the good path

    monkeypatch.setattr(solver, "_levers", lambda: [("useless", useless, lambda o: True),
                                                    ("fixer", fixer, lambda o: True)])
    out, report = solve(bad, Context(robot, cell, log=lambda _: None))
    assert calls == ["useless", "fixer"] and out is path and report.ok
    assert len(report.fixed) == 1 and report.fixed[0].startswith("fixer:")


def test_unfixable_is_reported_and_path_unchanged(setup, monkeypatch):
    cell, robot, path = setup
    bad = _far(path, 5)
    monkeypatch.setattr(solver, "_levers", lambda: [("noop", lambda p, i, c: p, lambda o: True)])
    out, report = solve(bad, Context(robot, cell, log=lambda _: None))
    assert out is bad and not report.ok and not report.fixed


def test_disabled_lever_is_skipped(setup, monkeypatch):
    cell, robot, path = setup
    monkeypatch.setattr(solver, "_levers", lambda: [("e1", lambda p, i, c: path, lambda o: o.e1_per_layer)])
    out, report = solve(_far(path, 5), Context(robot, cell, opts=SolveOptions(e1_per_layer=False),
                                               log=lambda _: None))
    assert not report.ok and not report.fixed
