"""Progress and cancel callbacks of the slow jobs (auto position, collision check) and JobProgress."""
import os
import time

import numpy as np
import pytest

from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.position import auto_position, layer_tilts


@pytest.fixture(scope="module")
def layers_job():
    cell, proc = load_cell(), load_process()
    robot = Robot(load_robot(), load_tool())
    part = load_part("test_block")
    plan = lambda tilt: plan_part(part, proc, "layers", tilt_deg=tilt, cell=cell)[1]
    return plan, robot, cell


def _monotonic_0_to_1(values):
    return values[0] == 0.0 and values[-1] == 1.0 and all(b >= a for a, b in zip(values, values[1:])) \
        and all(0.0 <= v <= 1.0 for v in values)


def test_auto_position_cancel_immediately(layers_job):
    _, robot, cell = layers_job

    def plan(tilt):
        raise AssertionError("should not plan after cancel")

    t0 = time.perf_counter()
    pos = auto_position(plan, robot, cell, layer_tilts(cell), log=lambda _: None, cancel=lambda: True)
    assert time.perf_counter() - t0 < 1.0
    assert pos.reach < 1.0


def test_auto_position_cancel_midway_keeps_best(layers_job):
    plan, robot, cell = layers_job
    calls = []

    def cancel():
        calls.append(1)
        return len(calls) > 3          # stop inside the first grid
    pos = auto_position(plan, robot, cell, layer_tilts(cell), log=lambda _: None, cancel=cancel)
    assert pos.reach < 1.0


def test_auto_position_progress_monotonic(layers_job):
    plan, robot, cell = layers_job
    seen = []
    pos = auto_position(plan, robot, cell, layer_tilts(cell), log=lambda _: None, progress=seen.append)
    assert pos.reach == 1.0
    assert len(seen) > 3 and _monotonic_0_to_1(seen)


class _Checker:
    """CollisionChecker.check_path with a fake per-sample check (no meshes needed)."""
    from coldspray.sim.collision import CollisionChecker
    check_path = CollisionChecker.check_path

    def __init__(self):
        from coldspray.models import load_cell
        self.checked = []
        self.cell = load_cell()

    def _precompute(self, joints):                 # check_path's batch FK: not needed by the fake check
        return [None] * len(joints), None

    def _table(self, axes):
        return None

    def check(self, q, axes, _pre=None, _table=None):
        self.checked.append(q)
        return {("A5", "cell")} if q[0] > 50 else set()


def test_check_path_cancel_immediately():
    c = _Checker()
    joints = np.tile(np.array(HOME_DEG, float), (1000, 1))
    assert c.check_path(joints, lambda i: {}, cancel=lambda: True) == []
    assert c.checked == []


def test_check_path_progress_monotonic_and_same_hits():
    joints = np.tile(np.array(HOME_DEG, float), (500, 1))
    joints[100:120, 0] = 90.0
    joints[300, :] = np.nan
    seen = []
    plain = _Checker().check_path(joints, lambda i: {}, every=3)
    tracked = _Checker().check_path(joints, lambda i: {}, every=3, progress=seen.append, cancel=lambda: False)
    assert [h.sample for h in plain] == [h.sample for h in tracked] and plain
    assert _monotonic_0_to_1(seen)


def test_job_progress_widget():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMainWindow
    from coldspray.gui.progress import JobProgress
    app = QApplication.instance() or QApplication([])
    win = QMainWindow()
    job = JobProgress()
    win.statusBar().addPermanentWidget(job)
    assert job.isHidden()
    got = []
    job.cancelRequested.connect(lambda: got.append(1))
    job.start("Working")
    assert not job.isHidden() and job.bar.maximum() == 0          # busy
    job.set_fraction(0.25)
    job.poll()
    assert job.bar.maximum() > 0 and job.bar.value() == job.bar.maximum() // 4
    job.update()                                                   # plain repaint keeps the value
    assert job.bar.value() == job.bar.maximum() // 4
    assert not job.cancelled and not job.is_cancelled()
    job.cancel_btn.click()
    assert job.cancelled and job.is_cancelled() and got == [1]
    job.finish("done")
    assert job.isHidden()
    job.start("Again", cancellable=False)
    assert not job.cancelled and job.cancel_btn.isHidden()
