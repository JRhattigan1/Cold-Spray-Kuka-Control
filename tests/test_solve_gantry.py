"""fix_gantry: one whole-program E1, set on the first (gun-off PTP) move and suggested (S7G/1)."""

import pytest

import coldspray.planner.solver as solver
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.gun import wrist_side
from coldspray.planner.solve_gantry import fix_gantry, with_gantry
from coldspray.planner.solver import Context, find_issues, solve


@pytest.fixture(scope="module")
def setup():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = 2500.0
    path = plan_part(load_part("test_square"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                     wrist=wrist_side(robot.tool))[1]
    return cell, robot, path


def _far_cell(cell):
    c = cell.model_copy(deep=True)
    c.gantry.position_mm = c.gantry.limits_mm[0]        # at one end: the part is out of reach
    return c


def test_nothing_to_fix_returns_same_path(setup):
    cell, robot, path = setup
    assert fix_gantry(path, [], Context(robot, cell, log=lambda _: None)) is path


def test_with_gantry_sets_e1_on_a_gun_off_ptp_first_move(setup):
    cell, _, path = setup
    p = with_gantry(path, "E1", 1234.0)
    assert p.moves[0].kind == "PTP" and not p.moves[0].gun_on and p.moves[0].axes["E1"] == 1234.0
    assert all("E1" not in m.axes for m in p.moves[1:])


def test_far_gantry_gets_a_whole_program_e1(setup):
    cell, robot, path = setup
    c = _far_cell(cell)
    ctx = Context(robot, c, log=lambda _: None)
    _, _, issues = find_issues(path, ctx)
    if not issues:
        pytest.skip("reachable from the gantry end")
    new = fix_gantry(path, issues, ctx)
    e1 = ctx.suggested["gantry_e1"]
    lo, hi = c.gantry.limits_mm
    assert lo <= e1 <= hi and new.moves[0].axes["E1"] == e1
    assert len(find_issues(new, ctx)[2]) < len(issues)


def test_solve_reports_the_suggested_gantry_e1(setup, monkeypatch):
    cell, robot, path = setup
    c = _far_cell(cell)
    logs = []
    levers = [x for x in solver._levers() if x[0] == "gantry"]
    assert levers
    monkeypatch.setattr(solver, "_levers", lambda: levers)
    out, report = solve(path, Context(robot, c, log=logs.append))
    if not report.fixed:
        pytest.skip("nothing to fix from the gantry end")
    assert "gantry_e1" in report.suggested
    assert any("one gantry E1" in s for s in logs)


def test_gantry_is_tried_before_e1_per_layer():
    names = [n for n, _, _ in solver._levers()]
    assert names.index("gantry") < names.index("e1")


def test_suggestion_rows_and_apply_sets_the_cell_e1(monkeypatch):
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from coldspray.gui.checks import solve_checks
    from coldspray.planner.solver import SolveReport
    app = QApplication.instance() or QApplication([])
    rep = SolveReport(fixed=["gantry: 3 of 3 problem points fixed"], suggested={"gantry_e1": 2100.0})
    assert any(r.title == "Suggested gantry E1 = 2100 mm" for r in solve_checks(rep))
    from coldspray.gui.app import MainWindow
    w = MainWindow()
    w._refresh_steps = lambda: None
    try:
        if not w.e1.isEnabled():
            pytest.skip("no gantry in the cell")
        w.e1.setRange(0.0, 5500.0)                 # set from the cell when assets load
        path = object()
        w.result, w.solve_report, w._solve_path = (path, None, object(), object()), rep, path
        w._apply_suggested()
        assert w.e1.value() == 2100.0               # _work_cell takes E1 from this box
    finally:
        w.pool.shutdown(wait=False)
        w.close()
