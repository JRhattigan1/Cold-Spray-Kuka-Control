"""Constraints Section of the Position step (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtWidgets import QApplication

from coldspray.cad.part import Substrate
from coldspray.gui.constraints_panel import PART_AXIS, ConstraintsPanel
from coldspray.pipeline import load_part
from coldspray.transforms import frame


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def panel(app):
    tube = load_part("test_tube")
    p = ConstraintsPanel()
    p.set_part(tube, Substrate())
    return p, tube


def bottom(tube):
    return next(i for i, f in tube.planar.items() if np.allclose(f.normal, (0, 0, -1), atol=1e-9))


def test_touch_and_centre_on_tube(panel):
    p, tube = panel
    start = tube.placement @ frame(30, -20, 40, 0, 10, 5)
    p.add("touch", f"face:{bottom(tube)}", "substrate")
    p.add("centre", PART_AXIS, "table_axis")
    t = p.apply(start)
    assert not np.allclose(t, start)
    assert np.allclose(t[:3, :3] @ (0, 0, 1), (0, 0, 1), atol=1e-6)
    assert np.allclose(t[:2, 3], 0, atol=1e-5)
    assert np.isclose(t[2, 3], Substrate().top, atol=1e-4)
    assert p.result.status == "ok" and p.chip.text() == "1 DOF free"
    assert all(r.status.text() == "✓" for r in p.rows)


def test_suppress_and_delete(panel):
    p, tube = panel
    start = tube.placement @ frame(30, -20, 40, 0, 0, 0)
    p.add("touch", f"face:{bottom(tube)}", "substrate")
    centre = p.add("centre", PART_AXIS, "table_axis")
    centre.suppress.setChecked(True)
    t = p.apply(start)
    assert np.allclose(t[:2, 3], start[:2, 3], atol=1e-3)    # centring left out
    assert p.result.dof_left == 3
    centre.delete.click()
    assert len(p.rows) == 1
    p.rows[0].delete.click()
    assert p.rows == [] and np.allclose(p.apply(start), start)


def test_conflict_marks_rows(panel):
    p, tube = panel
    face = f"face:{bottom(tube)}"
    p.add("touch", face, "substrate")
    p.add("touch", face, "faceplate")
    p.apply(tube.placement)
    assert p.result.status != "ok" and p.chip.text() == "conflict"
    assert any(r.status.text() == "✗" for r in p.rows)


def test_centre_between_row(app):
    repair = load_part("test_repair")
    p = ConstraintsPanel()
    p.set_part(repair, Substrate())
    ids = [i for i, f in repair.planar.items() if abs(f.normal[0]) > 0.99]
    lo = min(ids, key=lambda i: repair.planar[i].point[0])
    hi = max(ids, key=lambda i: repair.planar[i].point[0])
    row = p.add("centre_between", f"face:{lo}", "datum:YZ", face2=f"face:{hi}")
    assert row.face2.isVisibleTo(row) and row.face2.currentText() == f"face:{hi}"
    t = p.apply(repair.placement @ frame(17, 0, 0, 0, 0, 0))
    assert p.result.status == "ok" and row.status.text() == "✓"
    xl = (t @ np.append(repair.planar[lo].point, 1))[0]
    xr = (t @ np.append(repair.planar[hi].point, 1))[0]
    assert (xl + xr) / 2 == pytest.approx(0, abs=1e-5)


def test_fix_row_locks_current_pose(panel):
    p, tube = panel
    here = tube.placement @ frame(5, 5, 5, 0, 0, 10)
    tube.placement = here
    row = p.add("fix")
    assert not row.face.isVisibleTo(row) and not row.ref.isVisibleTo(row)
    t = p.apply(tube.placement @ frame(30, -20, 40, 0, 10, 5))
    assert np.allclose(t, here, atol=1e-5)
    assert p.result.dof_left == 0 and p.chip.text() == "0 DOF free"
    tube.placement = here @ frame(1, 0, 0, 0, 0, 0)
    row.lock.click()
    assert np.allclose(p.apply(here), tube.placement, atol=1e-5)


def test_menu_has_new_rows(panel):
    p, _ = panel
    names = [a.text() for a in p.add_btn.menu().actions()]
    assert "centre_between" in names and "fix" in names


class FakeScene:
    """Stands in for sim.viewer.Scene: records pick mode and highlighted faces."""

    def __init__(self):
        self.pick, self.lit, self.pl = None, [], type("Pl", (), {"render": lambda self: None})()

    def start_face_pick(self, callback, on_cancel=None):
        self.pick = (callback, on_cancel)
        return True

    def cancel_face_pick(self, notify=False):
        pick, self.pick = self.pick, None
        if notify and pick and pick[1]:
            pick[1]()

    def highlight_part_faces(self, ids):
        self.lit = list(ids)


def test_pick_face_graphically(app):
    repair = load_part("test_repair")
    p = ConstraintsPanel()
    p.set_part(repair, Substrate())
    scene = FakeScene()
    p.set_scene(scene)
    msgs = []
    p.message.connect(msgs.append)
    ids = [i for i, f in repair.planar.items() if abs(f.normal[0]) > 0.99]
    row = p.add("centre_between", f"face:{ids[0]}", "datum:YZ", face2=f"face:{ids[0]}")
    assert scene.lit == [ids[0], ids[0]]                       # selected row's faces lit
    assert row.face.label.text() == f"F{ids[0]}"
    row.face2.pick.click()                                     # Pick toggle on
    assert scene.pick is not None and row.face2.pick.isChecked()
    bad = next(int(i) for i in range(int(repair.face_ids.max()) + 2)
               if i not in repair.planar and i not in (repair.cylinders or {}))
    scene.pick[0](bad)                                         # neither flat nor a cylinder: rejected
    assert "neither flat nor cylindrical" in msgs[-1] and scene.pick is not None
    scene.pick[0](ids[1])
    assert scene.pick is None and not row.face2.pick.isChecked()
    assert row.face2.currentText() == f"face:{ids[1]}" and scene.lit == [ids[0], ids[1]]
    row.face.pick.click()
    row.face.pick.click()                                      # clicking Pick again cancels
    assert scene.pick is None and row.face.currentText() == f"face:{ids[0]}"
    row.face.pick.click()
    scene.cancel_face_pick(notify=True)                        # Esc in the view
    assert p.picking is None and not row.face.pick.isChecked()
    p.add("touch", f"face:{ids[1]}", "substrate")
    assert scene.lit == [ids[1]]
    row.selected.emit()
    assert scene.lit == [ids[0], ids[1]]


def test_scene_face_pick_and_highlight():
    import pyvista as pv
    from coldspray.cell import robroot
    from coldspray.kinematics.robot import Robot
    from coldspray.models import load_cell, load_robot, load_tool
    from coldspray.sim.viewer import Scene

    cell = load_cell()
    pl = pv.Plotter(off_screen=True, window_size=(300, 200))
    scene = Scene(pl, Robot(load_robot(), load_tool()), robroot(cell))
    tube = load_part("test_tube")
    scene.enter_setup(cell)
    scene.set_part(tube)
    face = bottom(tube)
    scene.highlight_part_faces([face])
    build = np.asarray(scene.part_mesh.cell_data["build"])
    assert np.all(build[tube.face_ids == face] == 2) and np.all(build[tube.face_ids != face] < 2)
    scene.highlight_part_faces([])
    assert np.asarray(scene.part_mesh.cell_data["build"]).max() <= 1

    class Iren:                                                # the interactor's observer API
        def __init__(self):
            self.obs = {}

        def add_observer(self, event, cb):
            self.obs[len(self.obs)] = (event, cb)
            return len(self.obs) - 1

        def remove_observer(self, k):
            self.obs.pop(k)

    pl.iren = Iren()
    got, cancelled = [], []
    assert scene.start_face_pick(got.append, on_cancel=lambda: cancelled.append(1))
    scene.report_face(face)
    assert got == [face] and scene.picking_faces()
    key = next(cb for ev, cb in pl.iren.obs.values() if ev == "KeyPressEvent")
    key(type("E", (), {"GetKeySym": lambda self: "Escape"})(), "KeyPressEvent")
    assert cancelled == [1] and not scene.picking_faces() and pl.iren.obs == {}
