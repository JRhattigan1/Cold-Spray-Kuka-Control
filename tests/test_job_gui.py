"""Job save / open in the GUI (off screen): save every step's settings, open them in a fresh
window, and Generate gives the identical KRL."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest

from coldspray.models import PROJECT_ROOT

pytestmark = pytest.mark.skipif(not (PROJECT_ROOT / "assets" / "cad" / "cache").exists(),
                                reason="cell CAD cache not available")


def _window(tmp_path, monkeypatch, name):
    from PySide6.QtCore import QSettings
    from PySide6.QtWidgets import QApplication, QMessageBox
    QApplication.instance() or QApplication([])
    from coldspray.gui.app import MainWindow
    dialogs = []
    for kind in ("warning", "critical", "information", "question"):
        monkeypatch.setattr(QMessageBox, kind, staticmethod(lambda *a, k=kind, **kw: dialogs.append((k, a[1:]))))
    w = MainWindow(QSettings(str(tmp_path / f"{name}.ini"), QSettings.IniFormat))
    w._startup()
    w.dialogs = dialogs
    return w


def _krl(win):
    from coldspray.gui import plan_jobs
    from coldspray.post.krl import to_krl
    win.check_box.setChecked(False)
    win.generate()
    plan_jobs.wait(win)
    assert win.result is not None, win.dialogs
    path, proc, cell, robot = win.result
    return to_krl(path, robot, cell, proc, win._part_base())


@pytest.mark.slow
def test_save_open_same_settings_and_krl(tmp_path, monkeypatch):
    from coldspray.gui import job_menu
    from coldspray.gui.stepper import PART, POSITION
    from coldspray.transforms import frame

    a = _window(tmp_path, monkeypatch, "a")
    b = None
    try:
        a._set_part("test_block")
        face = int(next(iter(a.source.planar)))
        a._set_coat({face})
        a._set_krl_base(frame(5, 6, 0, 0, 0, 0), face)
        a.go_to(POSITION)
        a.sub_fields["thickness"].setValue(12.0)
        a.place["X"].setValue(20.0)
        a.place["A"].setValue(30.0)
        a.roll.setValue(10.0)
        a.e1.setValue(a.e1.value() + 50.0)
        a.purge_on.setChecked(True)
        a.purge_dwell.setValue(7.0)
        a.fields["step_over_mm"].setValue(a.fields["step_over_mm"].value() + 0.5)
        box = a.fields["infill_pattern"]
        box.setCurrentIndex(box.findData("oneway"))
        a.preflight_check.setChecked(True)
        a.spray_output_check.setChecked(False)
        a.split_check.setChecked(True)
        path = job_menu.save_job(a, tmp_path / "job.csjob")
        assert path.exists() and job_menu.recent(a)[0] == str(path.resolve())
        krl_a = _krl(a)

        b = _window(tmp_path, monkeypatch, "b")
        assert job_menu.open_job(b, path), b.dialogs
        assert b.source.coat_faces == {face} and np.allclose(b.part.krl_base, a.part.krl_base)
        assert np.array_equal(b.part.placement, a.part.placement)
        for name in ("roll", "e1", "tilt", "rotation"):
            assert getattr(b, name).value() == getattr(a, name).value(), name
        assert b.sub_fields["thickness"].value() == 12.0 and b.substrate.thickness == 12.0
        assert b.read_process() == a.read_process()
        assert b.mode.currentData() == a.mode.currentData()
        assert b.purge_cfg == a.purge_cfg and b.preflight_check.isChecked()
        assert not b.spray_output_check.isChecked() and b.split_check.isChecked()
        assert job_menu.collect(b) == job_menu.collect(a)
        assert _krl(b) == krl_a
        assert not [d for d in a.dialogs + b.dialogs if d[0] == "warning"], a.dialogs + b.dialogs
    finally:
        a.close()
        if b is not None:
            b.close()


def test_job_menu_and_shortcuts(tmp_path):
    from PySide6.QtCore import QSettings
    from PySide6.QtGui import QShortcut
    from PySide6.QtWidgets import QApplication
    QApplication.instance() or QApplication([])
    from coldspray.gui.app import MainWindow
    win = MainWindow(QSettings(str(tmp_path / "ui.ini"), QSettings.IniFormat))
    try:
        keys = {s.key().toString() for s in win.findChildren(QShortcut)}
        assert {"Ctrl+S", "Ctrl+O"} <= keys
        texts = [a.text() for a in win.job_menu.actions()]
        assert texts[0].startswith("Save job") and texts[1].startswith("Open job")
        win.ui_settings.setValue("jobs/recent", [str(tmp_path / "x.csjob")])
        from coldspray.gui import job_menu
        job_menu._fill_menu(win)
        assert win.job_menu.actions()[-1].text() == "x.csjob"
    finally:
        win.plotter.close()
        win.part_plotter.close()
        win.pool.shutdown(wait=False)
