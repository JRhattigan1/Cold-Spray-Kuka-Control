"""End-to-end offscreen smoke run of the GUI: part -> coat faces -> position -> generate -> simulate -> export."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from coldspray.models import PROJECT_ROOT

pytestmark = pytest.mark.skipif(not (PROJECT_ROOT / "assets" / "cad" / "cache").exists(),
                                reason="cell CAD cache not available")


@pytest.fixture
def win(tmp_path, monkeypatch):
    from PySide6.QtCore import QSettings
    from PySide6.QtWidgets import QApplication, QMessageBox
    QApplication.instance() or QApplication([])
    from coldspray.gui import plan_jobs
    from coldspray.gui.app import MainWindow
    dialogs = []
    for kind in ("warning", "critical", "information", "question"):
        monkeypatch.setattr(QMessageBox, kind, staticmethod(lambda *a, k=kind, **kw: dialogs.append((k, a[1:]))))
    w = MainWindow(QSettings(str(tmp_path / "ui.ini"), QSettings.IniFormat))
    w._startup()
    w.dialogs = dialogs
    w.wait = lambda: plan_jobs.wait(w)      # Generate / Auto position plan on a worker
    yield w
    w.close()


@pytest.mark.slow
def test_coat_change_reaches_cell_part(win):
    """Coat faces picked after the part went into the cell must reach the cell part (Surface mode)."""
    from coldspray.gui.stepper import POSITION
    win._set_part("test_repair")
    win.go_to(POSITION)
    f = int(win.source.face_ids[0])
    win.go_to(0)
    win._set_coat({f})
    win.go_to(POSITION)
    assert win.part.coat_faces == {f}


@pytest.mark.slow
def test_layers_generate_and_export(win, monkeypatch):
    from coldspray.gui.stepper import EXPORT, POSITION, SIMULATE, TOOLPATH
    win._set_part("test_block")
    win.go_to(POSITION)
    win.purge_on.setChecked(True)
    import coldspray.gui.steps.position as pos_mod
    import coldspray.gui.steps.toolpath as tp_mod
    plans = []
    for mod in (pos_mod, tp_mod):
        real = mod.plan_part
        monkeypatch.setattr(mod, "plan_part", lambda *a, real=real, **k: plans.append(1) or real(*a, **k))
    win._auto_position()
    win.wait()
    win.go_to(TOOLPATH)
    n = len(plans)
    win.generate()                       # same settings as Auto position's pick: plan reused
    win.wait()
    assert win.result is not None and len(plans) == n, win.dialogs
    for pat in ("zigzag", "oneway", "concentric", "spiral", "islands"):
        box = win.fields["infill_pattern"]
        box.setCurrentIndex(box.findData(pat))
        win.generate()
        win.wait()
        assert win.result is not None, (pat, win.dialogs)
    win.fields["contour_tilt_deg"].setValue(20)
    win.generate()
    win.wait()
    assert win.result is not None, win.dialogs
    win.go_to(SIMULATE)
    win.go_to(EXPORT)
    assert not [d for d in win.dialogs if d[0] != "question"], win.dialogs


@pytest.mark.slow
def test_fixed_in_cell_generates_off_the_table(win):
    """Mounted "Fixed in the cell": the table locks at 0, the plate goes, and a part moved
    well off the faceplate still generates a toolpath."""
    from coldspray.gui.stepper import POSITION, TOOLPATH
    win._set_part("test_block")
    win.go_to(POSITION)
    win.tilt.setValue(30.0)
    win.mount.setCurrentIndex(win.mount.findData(True))
    assert win.tilt.value() == 0.0 and not win.tilt.isEnabled()
    assert win.substrate is None and win.part.fixed_in_cell and win.source.fixed_in_cell
    win.place["X"].setValue(900.0)                    # past the 500 mm faceplate
    win.place["Z"].setValue(-200.0)                   # below its surface
    assert "faceplate" not in win.face_label.text()
    win.go_to(TOOLPATH)
    win.generate()
    win.wait()
    assert win.result is not None, win.dialogs
