"""Review findings 1, 5, 6: program joints as Simulate, E1 never on a spray move, plan key vs robot side."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from coldspray.cad.part import Substrate
from coldspray.gui import plan_jobs
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.gun import wrist_side
from coldspray.planner.solve_e1 import _with_e1, e1_carriers, fix_e1, layer_starts
from coldspray.planner.solver import Context, find_issues, robot_targets
from coldspray.post.joints import ANGLE_STEP_DEG, STEP_MM, program_joints


@pytest.fixture(scope="module")
def setup():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    if cell.gantry is not None:
        cell.gantry.position_mm = 2500.0
    path = plan_part(load_part("test_square"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                     wrist=wrist_side(robot.tool))[1]
    return cell, robot, path


# --- 1: program joints ---------------------------------------------------------

def test_program_joints_match_simulate_samples(setup):
    cell, robot, path = setup
    q = program_joints(path, robot, cell)
    assert q.shape == (len(path.moves), 6)
    s = path.sample(STEP_MM, ANGLE_STEP_DEG)
    ref = robot.ik_path(robot_targets(s, cell), seed_deg=cell.home_deg, hint=path.hint_for(s.move_index))
    last = len(s) - 1 - np.argmax(s.move_index[::-1] == len(path.moves) - 1)
    np.testing.assert_allclose(q[-1], ref[last])


def test_hinted_ptp_takes_the_hint_configuration(setup):
    cell, robot, path = setup
    s = path.sample(STEP_MM, ANGLE_STEP_DEG)
    targets = robot_targets(s, cell)
    plain = program_joints(path, robot, cell)
    ptps = [i for i, m in enumerate(path.moves) if m.kind == "PTP" and not np.isnan(plain[i, 0])]
    assert ptps, "test path has no reachable PTP"
    k = ptps[len(ptps) // 2]
    end = int(np.flatnonzero(s.move_index == k)[-1])
    sols = robot._candidates(targets[end][None])[0]
    sols = sols[~np.isnan(sols[:, 0])]
    other = [c for c in sols if np.max(np.abs(c - plain[k])) > 20.0]
    if not other:
        pytest.skip("only one configuration reaches this PTP")
    hint = np.full((len(path.moves), 6), np.nan)
    hint[k] = other[0]
    q = program_joints(replace(path, joint_hint=hint), robot, cell)
    np.testing.assert_allclose(q[k], other[0], atol=1e-9)


# --- 5: E1 only on gun-off moves -------------------------------------------------

def test_e1_goes_on_first_gun_off_move_of_layer(setup):
    cell, _, path = setup
    starts = layer_starts(path)
    moves = list(path.moves)
    k = len(starts) - 1
    end = len(moves)
    moves[starts[k]] = replace(moves[starts[k]], gun_on=True)          # layer opens spraying
    sprayed = replace(path, moves=moves)
    carriers = e1_carriers(sprayed, starts)
    c = carriers[k]
    if c is None:
        pytest.skip("layer sprays throughout")
    assert c > starts[k] and not sprayed.moves[c].gun_on and c < end
    out = _with_e1(sprayed, carriers, [1000.0] * len(starts), "E1")
    assert "E1" not in out.moves[starts[k]].axes and out.moves[c].axes["E1"] == 1000.0


def test_all_spray_layer_gets_no_e1():
    p = SimpleNamespace(moves=[SimpleNamespace(gun_on=True, axes={}) for _ in range(3)])
    assert e1_carriers(p, [0]) == [None]


def test_fix_e1_never_changes_e1_on_a_spray_move(setup):
    cell, robot, path = setup
    if cell.gantry is None:
        pytest.skip("no gantry")
    cell2 = cell.model_copy(deep=True)
    cell2.gantry.position_mm = cell.gantry.limits_mm[0]
    moves = [replace(m, gun_on=True) if i in layer_starts(path) else m for i, m in enumerate(path.moves)]
    p = replace(path, moves=moves)
    ctx = Context(robot, cell2, log=lambda _: None)
    _, _, issues = find_issues(p, ctx)
    if not issues:
        pytest.skip("reachable from this E1")
    new = fix_e1(p, issues, ctx)
    for m in new.moves:
        if "E1" in m.axes:
            assert not m.gun_on


# --- 6: plan key and the robot's side -------------------------------------------

def _part():
    return SimpleNamespace(name="p", placement=np.eye(4), coat_faces={1}, start_face=None, base_face=None,
                           build_body=0, fixed_in_cell=False)


def _kw(cell):
    return dict(mode="layers", tilt_deg=10.0, rotation_deg=0.0, coats=1, cell=cell, substrate=Substrate(),
                roll_deg=0.0, wrist=np.array([0.0, 0.0, 1.0]))


def _flip_side(cell):
    """Same cell with the robot on the other side of the base (gantry run past it)."""
    c = cell.model_copy(deep=True)
    c.robroot.x, c.robroot.y = 2 * c.base.x - c.robroot.x, 2 * c.base.y - c.robroot.y
    return c


def _path(sprays):
    return SimpleNamespace(moves=[SimpleNamespace(pose=np.eye(4) if s == "h" else
                                                  np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1.0]]),
                                                  axes={}) for s in sprays],
                           axis_values=lambda: [{}] * len(sprays))


def test_key_includes_robot_side():
    cell = load_cell()
    a = plan_jobs.plan_key(1, _part(), load_process(), _kw(cell))
    b = plan_jobs.plan_key(1, _part(), load_process(), _kw(_flip_side(cell)))
    assert a[:-1] != b[:-1] or a != b
    assert a[-1] != b[-1]


def _hits(path):
    cell, proc = load_cell(), load_process()
    cache, calls = plan_jobs.new_cache(), []
    k1 = plan_jobs.plan_key(1, _part(), proc, _kw(cell))
    k2 = plan_jobs.plan_key(1, _part(), proc, _kw(cell))
    k2 = plan_jobs.PlanKey((*k1[:-1], ("other",)))
    k2.cell = k1.cell
    plan_jobs.cached_plan(cache, k1, lambda: calls.append(1) or ("s", path))
    plan_jobs.cached_plan(cache, k2, lambda: calls.append(2) or ("s", path))
    return calls


def test_side_free_plan_reused_across_sides():
    cell = load_cell()
    p = _path("hh")
    if not plan_jobs._side_free(p, cell):
        pytest.skip("base frame makes the test spray vertical")
    assert _hits(p) == [1]


def test_vertical_plan_replanned_for_another_side():
    cell = load_cell()
    p = _path("hv")
    if plan_jobs._side_free(p, cell):
        pytest.skip("base frame makes the test spray horizontal")
    assert _hits(p) == [1, 2]


def test_e1_change_on_same_side_still_hits():
    cell = load_cell()
    if cell.gantry is None:
        pytest.skip("no gantry")
    proc = load_process()
    c2 = cell.model_copy(deep=True)
    c2.gantry.position_mm = (c2.gantry.position_mm or 0.0) + 1.0
    k1 = plan_jobs.plan_key(1, _part(), proc, _kw(cell))
    k2 = plan_jobs.plan_key(1, _part(), proc, _kw(c2))
    assert k1[:-1] == k2[:-1]
