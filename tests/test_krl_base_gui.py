"""KRL base picker in the Part step (off screen): face + edge, 3 planes, vertex hover / click,
spin and flip, and the panel still fits the sidebar."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication


@pytest.fixture(scope="module")
def win(tmp_path_factory):
    from coldspray.gui import app as gui_app, style
    app = QApplication.instance() or QApplication([])
    style.apply(app, "light")
    w = gui_app.MainWindow(QSettings(str(tmp_path_factory.mktemp("ui") / "ui.ini"), QSettings.IniFormat))
    w._refresh_steps = lambda: None
    w._set_part("test_block")
    yield w
    w.close()


def _tri(part, normal):
    """(a triangle of, the face id of) the flat face facing most along `normal`."""
    fid = max(part.planar, key=lambda k: part.planar[k].normal @ np.asarray(normal, float))
    return int(np.flatnonzero(part.face_ids == fid)[0]), fid


def _pick(win, method):
    win.krl_btn.setChecked(False)
    win.krl_method.setCurrentIndex(win.krl_method.findData(method))
    win.krl_btn.setChecked(True)
    assert win.krl_btn.isChecked()


def test_methods_spin_flip(win):
    part = win.source
    lo, hi = part.vertices.min(0), part.vertices.max(0)
    corner = np.array([lo[0], lo[1], hi[2]])
    assert win.krl_method.currentData() == "face"                       # the default

    # face + edge: as before
    tri, top = _tri(part, [0, 0, 1])
    _pick(win, "face")
    win.part_view.pick_triangle(tri, corner + [5.0, 0.5, 0.0])
    assert part.krl_base_face == top and np.allclose(part.krl_base[:3, 3], corner)
    assert not win.krl_btn.isChecked()

    # 3 planes: top, front, left; a parallel pick is rejected and starts again
    _pick(win, "planes")
    vp = win.part_view
    vp.pick_triangle(_tri(part, [0, 0, 1])[0])
    assert "2nd" in vp.hint.GetText(2) and "krl0" in vp.highlights
    vp.pick_triangle(_tri(part, [0, 0, -1])[0])
    vp.pick_triangle(_tri(part, [-1, 0, 0])[0])
    assert "1st" in vp.hint.GetText(2) and not any(k.startswith("krl") for k in vp.highlights)
    for n in ([0, 0, 1], [0, -1, 0], [-1, 0, 0]):
        vp.pick_triangle(_tri(part, n)[0])
    assert part.krl_base_face is None and "3 planes" in win.krl_label.text()
    assert np.allclose(part.krl_base[:3, 3], corner) and np.allclose(part.krl_base[:3, 0], [1, 0, 0])

    # vertex: hover previews (snapped), click commits
    _pick(win, "vertex")
    tri, _ = _tri(part, [0, 0, 1])
    vp.hover_triangle(tri, corner + [0.3, 1.0, 0.0])
    assert len(vp.preview_actors) == 4
    vp.hover_triangle(None)
    assert vp.preview_actors == []
    vp.pick_triangle(tri, corner + [1.0, 0.3, 0.0])
    t = part.krl_base
    assert np.allclose(t[:3, 3], corner) and np.allclose(t[:3, 0], [1, 0, 0]) and np.allclose(t[:3, 2], [0, 0, 1])
    assert "vertex" in win.krl_label.text() and vp.preview_actors == []

    # spin and flip
    raw = t.copy()
    win._turn_krl(90.0)
    assert np.allclose(part.krl_base[:3, 0], [0, 1, 0]) and np.allclose(part.krl_base[:3, 3], corner)
    assert "turned 90" in win.krl_label.text()
    win.krl_flip.setChecked(True)
    assert np.allclose(part.krl_base[:3, 2], [0, 0, -1]) and np.allclose(part.krl_base[:3, 0], [0, 1, 0])
    win._turn_krl(-90.0)
    win.krl_flip.setChecked(False)
    assert np.allclose(part.krl_base, raw)
    win._set_krl_base(None, None)
    assert part.krl_base is None


def test_panel_fits(win):
    win.show()
    QApplication.processEvents()
    assert win.panels.widget(0).scroll.widget().minimumSizeHint().width() <= 398
