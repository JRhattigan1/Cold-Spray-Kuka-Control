"""Predicted coating thickness and process estimates (sim/deposit.py)."""

import numpy as np
import pytest

from coldspray.models import load_process
from coldspray.sim.deposit import estimate, refine, thickness
from coldspray.toolpath.model import Move, Toolpath, spray_frame
from coldspray.toolpath.raster import plate_raster


def _plate(w, l, cell=1.0):
    xs, ys = np.arange(0, w + 1e-9, cell), np.arange(0, l + 1e-9, cell)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    v = np.column_stack([X.ravel(), Y.ravel(), np.zeros(X.size)])
    nx, ny = len(xs), len(ys)
    i = np.arange(nx - 1)[:, None] * ny + np.arange(ny - 1)[None, :]
    i = i.ravel()
    f = np.vstack([np.column_stack([i, i + ny, i + ny + 1]), np.column_stack([i, i + ny + 1, i + 1])])
    return v, f


def _proc(**kw):
    return load_process().model_copy(update={"spray_settle_s": 0.0, **kw})


def _interior(dep, lo, hi):
    c = dep.vertices[dep.faces].mean(axis=1)
    m = (c[:, 0] > lo[0]) & (c[:, 0] < hi[0]) & (c[:, 1] > lo[1]) & (c[:, 1] < hi[1])
    return dep.thickness[m]


def test_flat_raster_is_uniform_layer_thickness():
    proc = _proc()
    v, f = _plate(60, 60)
    dep = thickness(v, f, plate_raster(proc, 60, 60), proc)
    t = _interior(dep, (10, 10), (50, 50))
    assert abs(t.mean() - proc.layer_thickness_mm) < 0.03 * proc.layer_thickness_mm
    assert t.std() < 0.03 * t.mean()
    st = dep.stats()
    assert st["max"] > 0 and st["min"] < st["mean"] <= st["max"]


def test_bunched_passes_are_thicker():
    proc = _proc()
    v, f = _plate(60, 60)
    normal = _interior(thickness(v, f, plate_raster(proc, 60, 60), proc), (10, 10), (50, 50)).mean()
    tight = proc.model_copy(update={"step_over_mm": proc.step_over_mm / 2})
    # same calibration (proc), passes half as far apart -> about twice as thick
    bunched = _interior(thickness(v, f, plate_raster(tight, 60, 60), proc), (10, 10), (50, 50)).mean()
    assert 1.9 < bunched / normal < 2.1


def test_tilted_spray_conserves_mass():
    proc = _proc()
    v, f = _plate(60, 60)
    path = plate_raster(proc, 60, 60)
    r = np.eye(4)
    c, s = np.cos(np.radians(30)), np.sin(np.radians(30))
    r[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]       # tilt the spray 30 deg about BASE X
    tilted = Toolpath(path.name, [Move(m.pose.copy(), m.speed, m.gun_on, m.kind) for m in path.moves])
    for m in tilted.moves:
        m.pose[:3, :3] = r[:3, :3] @ m.pose[:3, :3]
    # the tilted tracks land 17 mm further along +Y: compare well inside both
    a = _interior(thickness(v, f, path, proc), (25, 25), (45, 45)).mean()
    b = _interior(thickness(v, f, tilted, proc), (25, 25), (45, 45)).mean()
    assert abs(b / a - 1) < 0.03          # steady layer: same mass per area
    one = Toolpath("one", [Move(spray_frame((-20, 30, 30), (0, 0, -1), (1, 0, 0)), 100.0),
                           Move(spray_frame((80, 30, 30), (0, 0, -1), (1, 0, 0)), 100.0, gun_on=True)])
    tilt1 = Toolpath("one", [Move(m.pose.copy(), m.speed, m.gun_on) for m in one.moves])
    for m in tilt1.moves:
        m.pose[:3, :3] = r[:3, :3] @ m.pose[:3, :3]
    peak = thickness(v, f, one, proc).thickness.max()
    assert abs(thickness(v, f, tilt1, proc).thickness.max() / peak - c) < 0.05   # single track: lower


def test_refine_caps_edges():
    v = np.array([[0, 0, 0], [100, 0, 0], [0, 10, 0]], float)
    rv, rf = refine(v, [[0, 1, 2]], 2.0)
    e = np.linalg.norm(rv[rf][:, [1, 2, 0]] - rv[rf], axis=2)
    assert e.max() <= 2.0
    p = rv[rf]
    area = 0.5 * np.linalg.norm(np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1).sum()
    assert np.isclose(area, 500.0)


def test_estimate_matches_hand_count():
    proc = _proc(spray_settle_s=2.0, powder_feed_g_min=30.0)
    at = lambda x: spray_frame((x, 0, 30), (0, 0, -1), (1, 0, 0))
    path = Toolpath("t", [
        Move(at(0), 100.0),                                   # start
        Move(at(100), 50.0),                                  # 2 s, spray off
        Move(at(200), 25.0, gun_on=True),                     # settle 2 + 4 s spray
        Move(at(300), 50.0, gun_on=True, dwell_s=3.0),        # 2 s + 3 s dwell, spray on
        Move(at(400), 100.0),                                 # 1 s off
    ])
    e = estimate(path, proc)
    assert np.isclose(e.cycle_s, 2 + 2 + 4 + 2 + 3 + 1)
    assert np.isclose(e.spray_s, 2 + 4 + 2 + 3)
    assert np.isclose(e.powder_g, 11 / 60 * 30)
    from coldspray.gui.steps.toolpath import estimate_time_s
    assert np.isclose(e.cycle_s, estimate_time_s(path, proc))


def test_tube_is_fast():
    import time

    from coldspray.cad.part import build_mesh
    from coldspray.models import load_cell
    from coldspray.pipeline import load_part, plan_part

    part = load_part("test_tube")
    proc = load_process()
    _, path = plan_part(part, proc, "revolve", cell=load_cell())
    t0 = time.time()
    dep = thickness(*build_mesh(part), path, proc)
    assert time.time() - t0 < 15
    assert dep.thickness.max() > 0


@pytest.mark.slow
def test_simulate_thickness_toggle_and_estimates(tmp_path, monkeypatch):
    """GUI (off screen): Thickness colours the part with stats; estimates show in Simulate and Export."""
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import QSettings
    from PySide6.QtWidgets import QApplication, QMessageBox
    QApplication.instance() or QApplication([])
    from coldspray.gui import plan_jobs
    from coldspray.gui.app import MainWindow
    from coldspray.gui.stepper import POSITION, SIMULATE, TOOLPATH
    for kind in ("warning", "critical", "information", "question"):
        monkeypatch.setattr(QMessageBox, kind, staticmethod(lambda *a, **k: None))
    w = MainWindow(QSettings(str(tmp_path / "ui.ini"), QSettings.IniFormat))
    w._startup()
    try:
        w._set_part("test_tube")
        w.go_to(POSITION)
        w.go_to(TOOLPATH)
        w.check_box.setChecked(False)
        w.generate()
        plan_jobs.wait(w)
        assert w.result is not None
        w.go_to(SIMULATE)
        assert "powder" in w.info.text() and "powder" in w.export_estimate.text()
        assert w.thickness_box.isEnabled()
        w.thickness_box.setChecked(True)
        assert w.scene.thickness_actor is not None and "mean" in w.thickness_info.text()
        assert w.scene.THICKNESS_BAR in w.plotter.scalar_bars
        w.thickness_box.setChecked(False)
        assert w.scene.thickness_actor is None and w.scene.THICKNESS_BAR not in w.plotter.scalar_bars
    finally:
        w.pool.shutdown(wait=False)
        w.close()
