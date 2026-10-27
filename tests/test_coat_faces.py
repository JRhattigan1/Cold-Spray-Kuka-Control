"""Coat-face selection: Part.coat_faces, tangent growth, PartView picking (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pyvista as pv

from coldspray.cad.part import grow_tangent
from coldspray.pipeline import load_part
from coldspray.sim.part_view import PartView


def face_triangles(part, f):
    return np.flatnonzero(part.face_ids == f)


def radial_faces(part):
    """Non-planar faces with their mean distance from the Z axis."""
    out = {}
    for f in set(part.face_ids.tolist()) - set(part.planar):
        v = part.vertices[np.unique(part.faces[part.face_ids == f])]
        out[f] = np.hypot(v[:, 0] - v[:, 0].mean(), v[:, 1] - v[:, 1].mean()).mean()
    return out


def test_tube_outer_grows_to_itself():
    tube = load_part("test_tube")
    r = radial_faces(tube)
    outer = max(r, key=r.get)
    grown = grow_tangent(tube, {outer})
    assert all(f in r and abs(r[f] - r[outer]) < 1 for f in grown)   # outer cylinder only (maybe split in halves)
    assert not grown & set(tube.planar)            # not onto the end faces


def test_block_top_does_not_grow_onto_sides():
    block = load_part("test_block")
    top = next(i for i, f in block.planar.items() if np.allclose(f.normal, (0, 0, 1)))
    assert grow_tangent(block, {top}) == {top}
    assert grow_tangent(block, {top}, 91) == set(block.face_ids.tolist())   # edges are joined


def test_without_keeps_coat_faces():
    part = load_part("test_repair")
    part.coat_faces = {int(part.face_ids[0])}
    assert part.without([]).coat_faces == part.coat_faces
    assert part.without([]).coat_faces is not part.coat_faces


def test_pick_triangle_coat_mode():
    part = load_part("test_block")
    pl = pv.Plotter(off_screen=True, window_size=(300, 200))
    view = PartView(pl)
    view.set_part(part, ["build"])
    picked = []
    a, b = 0, int(np.flatnonzero(part.face_ids != part.face_ids[0])[0])
    view._pick_cb, view._keep = (lambda f, body: picked.append(f)), True   # as start_pick(..., keep=True)
    view.pick_triangle(a)
    view.pick_triangle(b)
    assert picked == [int(part.face_ids[a]), int(part.face_ids[b])]   # keeps picking
    view.highlight_coat({1, 2})
    assert view.coat_actor is not None
    view.highlight_coat(set())
    assert view.coat_actor is None
    pl.close()


def test_part_step_coat_ui():
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from coldspray.gui.app import MainWindow
    w = MainWindow()
    w._refresh_steps = lambda: None                 # no cell assets loaded off screen
    part = w.source = load_part("test_tube")
    w.part_view.set_part(part, ["build"])
    w._coat_picked(0, 0)
    assert part.coat_faces == {0} and "1 face" in w.coat_label.text()
    w._coat_picked(3, 0)
    w._coat_picked(3, 0)                            # a second click toggles it off
    assert part.coat_faces == {0}
    w._grow_coat()
    assert part.coat_faces == {0} and w.part_view.coat_actor is not None
    w.coat_btn.setChecked(True)
    w.start_btn.setChecked(True)                    # picking another face kind stops coat picking
    assert not w.coat_btn.isChecked()
    w._cancel_pick()
    w._set_coat(set())
    assert w.part_view.coat_actor is None and "No coat faces" in w.coat_label.text()
    w.close()


def test_grow_tangent_stays_in_body():
    from coldspray.cad.part import Part
    # two coplanar quads sharing an edge, faces 0 and 1, in different bodies
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [2, 0, 0], [2, 1, 0]], float)
    f = np.array([[0, 1, 2], [0, 2, 3], [1, 4, 5], [1, 5, 2]])
    ids = np.array([0, 0, 1, 1])
    one = Part("p", v, f, ids, {})
    assert grow_tangent(one, {0}) == {0, 1}
    two = Part("p", v, f, ids, {}, body_ids=np.array([0, 0, 1, 1]))
    assert grow_tangent(two, {0}) == {0}
