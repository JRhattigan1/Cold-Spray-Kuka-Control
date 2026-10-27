"""Solve problems button in Simulate: runs on the worker thread, applies the result (fake solve)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import time

import pytest
from PySide6.QtWidgets import QApplication

from coldspray.gui.checks import solve_checks
from coldspray.planner.solver import Issue, SolveReport


@pytest.fixture
def win():
    app = QApplication.instance() or QApplication([])
    from coldspray.gui.app import MainWindow
    w = MainWindow()
    w._refresh_steps = lambda: None
    yield w
    w.pool.shutdown(wait=False)
    w.close()


def _wait(w):
    t0 = time.time()
    while w.solve_future is not None and time.time() - t0 < 5:
        time.sleep(0.05)
        w._poll_check()
    assert w.solve_future is None


def test_solve_runs_off_thread_and_replaces_path(win, monkeypatch):
    import threading
    import coldspray.planner.solver as solver
    old, new = object(), object()
    seen = {}

    def fake(path, ctx):
        seen["thread"], seen["path"] = threading.current_thread(), path
        return new, SolveReport(fixed=["arm: 2 of 2 problem points fixed"])

    monkeypatch.setattr(solver, "solve", fake)
    win.result = (old, None, object(), object())
    shown = []
    win._show_solved = shown.append
    win._update_info = lambda: None
    win._solve()
    _wait(win)
    assert seen["path"] is old and seen["thread"] is not threading.main_thread()
    assert shown == [new] and win.solve_report.fixed


def test_unfixed_blocks_export_and_is_listed(win, monkeypatch):
    import coldspray.planner.solver as solver
    path = object()
    rep = SolveReport(unfixed=[Issue(7, 1, "collision", "A3/Wall")])
    monkeypatch.setattr(solver, "solve", lambda p, ctx: (p, rep))
    win.result = (path, None, object(), object())
    win._update_info = lambda: None
    win._solve()
    _wait(win)
    assert win.solve_report is rep and win._report() is rep
    rows = solve_checks(rep)
    assert rows[-1].state == "bad" and rows[-1].sample == 7 and "A3/Wall" in rows[-1].detail
    win.result = (object(), None, object(), object())     # a new Generate: the report no longer applies
    assert win._report() is None


def test_cancel_keeps_the_old_path(win, monkeypatch):
    import threading
    import coldspray.planner.solver as solver
    go, old = threading.Event(), object()

    def fake(path, ctx):
        go.wait(5)
        ctx.log("Solver: arm: 1 of 1 problem points fixed")    # raises once Cancel is pressed
        return object(), SolveReport(fixed=["arm"])

    monkeypatch.setattr(solver, "solve", fake)
    win.result = (old, None, object(), object())
    shown = []
    win._show_solved = shown.append
    win._update_info = lambda: None
    win._solve()
    assert win.job.cancel_btn.isVisibleTo(win.job)            # Cancel offered
    win._poll_check()                                         # still running: nothing applied
    win.job.request_cancel()
    go.set()
    _wait(win)
    assert shown == [] and win.result[0] is old and win._report() is None


def test_button_disabled_while_busy_and_log_shown(win, monkeypatch):
    import threading
    import coldspray.planner.solver as solver
    go, logged = threading.Event(), threading.Event()

    def fake(path, ctx):
        ctx.log("Solver: gun angle: 3 of 5 problem points fixed")
        logged.set()
        go.wait(5)
        return path, SolveReport()

    monkeypatch.setattr(solver, "solve", fake)
    win.result = (object(), None, object(), object())
    win._update_info = lambda: None
    win._solve()
    assert not win.solve_btn.isEnabled()
    win._solve()                                              # a second click while running does nothing
    logged.wait(5)
    win._poll_check()
    assert "gun angle" in win.job.label.text()
    go.set()
    _wait(win)


def test_unfixed_rows_reach_simulate_and_export_checks():
    from coldspray.gui.checks import open_items
    rep = SolveReport(fixed=["arm: 1 of 2 problem points fixed"],
                      unfixed=[Issue(3, 0, "unreachable")])
    rows = solve_checks(rep)
    assert [r.state for r in rows] == ["ok", "bad"]
    assert len(open_items(rows)) == 1
    assert solve_checks(SolveReport())[0].state == "ok"
