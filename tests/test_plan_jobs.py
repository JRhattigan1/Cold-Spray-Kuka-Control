"""gui/plan_jobs: plan cache keys and the worker job (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import numpy as np
import pytest
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from coldspray.cad.part import Substrate
from coldspray.gui import plan_jobs
from coldspray.gui.progress import JobProgress
from coldspray.models import load_cell, load_process


def _part(**kw):
    return SimpleNamespace(name="p", placement=np.eye(4), coat_faces={1}, start_face=None, base_face=None,
                           build_body=0, fixed_in_cell=False, **kw)


def _kw(**over):
    kw = dict(mode="layers", tilt_deg=10.0, rotation_deg=0.0, coats=1, cell=load_cell(), substrate=Substrate(),
              roll_deg=0.0, wrist=np.array([0.0, 0.0, 1.0]))
    kw.update(over)
    return kw


def test_key_ignores_e1_but_not_tilt_process_or_placement():
    part, proc = _part(), load_process()
    k = plan_jobs.plan_key(1, part, proc, _kw())
    cell = load_cell()
    if cell.gantry:
        cell.gantry.position_mm += 500.0
        assert plan_jobs.plan_key(1, part, proc, _kw(cell=cell)) == k
    assert plan_jobs.plan_key(1, part, proc, _kw(tilt_deg=20.0)) != k
    assert plan_jobs.plan_key(2, part, proc, _kw()) != k
    other = proc.model_copy(update={"traverse_speed_mm_s": proc.traverse_speed_mm_s + 1})
    assert plan_jobs.plan_key(1, part, other, _kw()) != k
    moved = _part()
    moved.placement = moved.placement.copy()
    moved.placement[0, 3] = 5.0
    assert plan_jobs.plan_key(1, moved, proc, _kw()) != k


def test_snapshot_is_independent_of_later_drags():
    part = _part()
    snap = plan_jobs.snapshot(part)
    part.placement[0, 3] = 99.0
    part.coat_faces.add(7)
    assert snap.placement[0, 3] == 0.0 and snap.coat_faces == {1}


def test_cache_reuses_and_bounds():
    cache, calls = plan_jobs.new_cache(), []
    for k in [1, 1, 2] + list(range(3, 3 + plan_jobs.CACHE_SIZE)):
        plan_jobs.cached_plan(cache, k, lambda k=k: calls.append(k) or k)
    assert calls.count(1) == 1 and len(cache) == plan_jobs.CACHE_SIZE


class Win:
    def __init__(self):
        self.job = JobProgress()
        self.check_id = 0
        self.plan_pool = ThreadPoolExecutor(max_workers=1)
        self.plan_job = None
        self.plan_timer = QTimer(interval=10, timeout=lambda: plan_jobs.poll(self))


@pytest.fixture
def win():
    QApplication.instance() or QApplication([])
    w = Win()
    yield w
    w.plan_pool.shutdown(wait=True)


def test_runs_on_worker_and_hands_back_on_gui_thread(win):
    import threading
    got = []
    plan_jobs.run(win, "Planning...", lambda: threading.get_ident(),
                  lambda out, err: got.append((out, err, threading.get_ident())))
    assert plan_jobs.busy(win)
    plan_jobs.wait(win, 10)
    (worker, err, gui), = got
    assert err is None and worker != gui == threading.get_ident()


def test_stale_result_is_dropped(win):
    got = []
    plan_jobs.run(win, "Planning...", lambda: time.sleep(0.2) or 1, lambda *a: got.append(a))
    win.check_id += 1                        # the set-up changed meanwhile
    plan_jobs.wait(win, 10)
    assert got == [] and not win.job.running


def test_cancel_abandons_a_job_that_cannot_stop(win):
    got = []
    plan_jobs.run(win, "Planning...", lambda: time.sleep(0.5) or 1, lambda *a: got.append(a),
                  abandon_on_cancel=True)
    win.job.request_cancel()
    t0 = time.time()
    plan_jobs.wait(win, 10)
    assert got == [] and time.time() - t0 < 0.4


def test_errors_reach_done(win):
    got = []
    plan_jobs.run(win, "x", lambda: 1 / 0, lambda out, err: got.append(err))
    plan_jobs.wait(win, 10)
    assert isinstance(got[0], ZeroDivisionError)


def test_without_a_pool_it_runs_inline():
    QApplication.instance() or QApplication([])
    w = SimpleNamespace(job=JobProgress(), check_id=0)
    got = []
    plan_jobs.run(w, "x", lambda: 5, lambda out, err: got.append(out))
    assert got == [5]
