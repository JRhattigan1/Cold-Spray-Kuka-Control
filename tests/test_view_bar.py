"""View toolbar: camera presets, section plane, follow, layer filter (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
import pyvista as pv

from coldspray.cad.part import placed_vertices
from coldspray.cell import robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.sim.part_view import PartView
from coldspray.sim.viewer import Scene


@pytest.fixture(scope="module")
def planned():
    cell, proc = load_cell(), load_process()
    part = load_part("test_block")
    _, path = plan_part(part, proc, "layers", cell=cell)
    return cell, proc, part, path


def make_scene(cell):
    pl = pv.Plotter(off_screen=True, window_size=(400, 300))
    return pl, Scene(pl, Robot(load_robot(), load_tool()), robroot(cell))


def preview_scene(planned, **kw):
    cell, proc, part, path = planned
    pl, scene = make_scene(cell)
    scene.set_part(part)
    scene.set_path(path, cell, part=(placed_vertices(part), part.faces), step=5.0, home=cell.home_deg, **kw)
    return pl, scene


def n_lines(scene) -> int:
    return sum(a.mapper.GetInput().GetNumberOfLines() for a in scene.line_actors)


def test_scene_presets_change_camera():
    cell = load_cell()
    pl, scene = make_scene(cell)
    scene.enter_setup(cell)
    seen = []
    for name in ("fit", "part", "iso", "top", "front", "side"):
        scene.view_preset(name)
        seen.append(tuple(np.round(np.r_[pl.camera.position, pl.camera.focal_point], 1)))
    assert len(set(seen)) == len(seen)
    scene.view_preset("top")
    fwd = np.subtract(pl.camera.focal_point, pl.camera.position)
    assert fwd[2] < -0.99 * np.linalg.norm(fwd)              # looking straight down
    scene.view_preset("front")
    assert np.allclose(pl.camera.up, (0, 0, 1))
    with pytest.raises(ValueError):
        scene.view_preset("nope")
    pl.close()


def test_scene_section_clips_part(planned):
    cell, _, part, _ = planned
    pl, scene = make_scene(cell)
    scene.set_part(part)
    scene.enter_setup(cell)
    scene.set_section(True, "x")
    assert scene.part_actor.mapper.GetNumberOfClippingPlanes() == 1
    scene.set_section(True, "y")                       # new axis: still one plane
    assert scene.part_actor.mapper.GetNumberOfClippingPlanes() == 1
    assert abs(scene.section_plane.GetNormal()[1]) == 1.0
    scene.set_part(part)                               # new actors get the plane too
    assert scene.part_actor.mapper.GetNumberOfClippingPlanes() == 1
    scene.set_section(False)
    assert scene.part_actor.mapper.GetNumberOfClippingPlanes() == 0
    assert scene.section_plane is None
    pl.close()


def test_layer_filter(planned):
    cell, proc, part, path = planned
    pl, scene = make_scene(cell)
    scene.set_path(path, cell, step=5.0, home=cell.home_deg)
    assert scene.n_layers == 0 and not scene.set_layer_filter("only", 0)   # no layer_mm: disabled
    pl.close()

    pl, scene = preview_scene(planned, layer_mm=proc.layer_thickness_mm)
    assert scene.n_layers == round(5 / proc.layer_thickness_mm)   # 5 mm block
    total = n_lines(scene)
    assert scene.set_layer_filter("only", 0)
    one = n_lines(scene)
    assert scene.set_layer_filter("upto", 2)
    three = n_lines(scene)
    assert 0 < one < three < total
    assert scene.set_layer_filter("all")
    assert n_lines(scene) == total
    assert all(a in scene.path_actors for a in scene.line_actors)
    pl.close()


def test_follow_moves_camera_with_tool(planned):
    pl, scene = preview_scene(planned)
    scene.view_preset("iso")
    scene.set_follow(True)
    offset = np.subtract(pl.camera.position, pl.camera.focal_point)
    scene.show_point(scene.n_points - 1)
    tcp = (scene.base_frames[-1] @ scene.samples.poses[-1])[:3, 3]
    assert np.allclose(pl.camera.focal_point, tcp)
    assert np.allclose(np.subtract(pl.camera.position, pl.camera.focal_point), offset)
    scene.set_follow(False)
    scene.show_point(0)
    assert np.allclose(pl.camera.focal_point, tcp)
    pl.close()


def test_part_view_presets_and_section():
    part = load_part("test_repair")
    pl = pv.Plotter(off_screen=True, window_size=(400, 300))
    view = PartView(pl)
    view.set_part(part, ["fixture", "build"])
    seen = set()
    for name in ("iso", "top", "front", "side", "bottom"):
        view.view_preset(name)
        seen.add(tuple(np.round(pl.camera.position, 1)))
    assert len(seen) == 5
    view.highlight("start", int(part.face_ids[0]))
    view.set_section(True, "z")
    assert all(a.mapper.GetNumberOfClippingPlanes() == 1 for a in view.actors + list(view.highlights.values()))
    view.set_section(False)
    assert all(a.mapper.GetNumberOfClippingPlanes() == 0 for a in view.actors)
    pl.close()


# --- the Qt toolbar ------------------------------------------------------------
class FakeTarget:
    def __init__(self):
        self.calls = []
        self.section_plane, self.section_axis = None, "x"
        self.follow, self.mode, self.n_points, self.n_layers = False, "preview", 10, 4
        self.layer_filter = ("all", None)
        self.groups = {"Booth walls": False, "Hoppers": True}

    def view_preset(self, name):
        self.calls.append(("preset", name))

    def set_section(self, on, axis=None):
        self.section_axis = axis or self.section_axis
        self.calls.append(("section", on, self.section_axis))
        self.section_plane = object() if on else None

    def set_follow(self, on):
        self.calls.append(("follow", on))

    def set_layer_filter(self, mode, n=None):
        self.calls.append(("layers", mode, n))
        return True

    def cell_groups(self):
        return list(self.groups.items())

    def set_group_visible(self, label, on):
        self.calls.append(("group", label, on))


@pytest.fixture(scope="module")
def qapp():
    from PySide6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def test_view_bar_calls_through(qapp):
    from coldspray.gui.view_bar import ViewBar

    t, renders, toggled = FakeTarget(), [], []
    bar = ViewBar(t, lambda: renders.append(1), kind="cell")
    bar.groupToggled.connect(lambda label, on: toggled.append((label, on)))
    for name in ("fit", "part", "iso", "top", "front", "side"):
        bar.buttons[name].click()
    assert [c[1] for c in t.calls] == ["fit", "part", "iso", "top", "front", "side"]
    assert "bottom" not in bar.buttons and bar.findChild(object, "view_iso") is bar.buttons["iso"]

    t.calls.clear()
    bar.buttons["section"].click()
    bar.axis_actions["z"].trigger()
    bar.buttons["section"].click()
    assert t.calls == [("section", True, "x"), ("section", True, "z"), ("section", False, "z")]

    t.calls.clear()
    bar.buttons["follow"].click()
    bar.set_layer(2)
    bar.layer_actions["only"].trigger()
    assert t.calls == [("follow", True), ("layers", "only", 2)]

    walls = next(a for a in bar.cell_menu.actions() if a.text() == "Booth walls")
    assert not walls.isChecked()
    walls.trigger()
    assert ("group", "Booth walls", True) in t.calls and toggled == [("Booth walls", True)]
    assert len(renders) >= 10

    t.n_layers = 0
    bar.refresh()
    assert not bar.buttons["layers"].isEnabled()


def test_view_bar_on_real_views(qapp, planned):
    from coldspray.gui.view_bar import ViewBar

    cell, proc, part, path = planned
    pl, scene = preview_scene(planned, layer_mm=proc.layer_thickness_mm)
    bar = ViewBar(scene, pl.render, kind="cell")
    assert bar.buttons["layers"].isEnabled() and bar.layer_spin.maximum() == scene.n_layers
    total = n_lines(scene)
    bar.set_layer(0)
    bar.set_layer_mode("only")
    assert n_lines(scene) < total
    bar.buttons["section"].click()
    assert scene.section_plane is not None
    pl.close()

    pl = pv.Plotter(off_screen=True, window_size=(400, 300))
    view = PartView(pl)
    view.set_part(part, ["build"])
    bar = ViewBar(view, pl.render, kind="part")
    assert "bottom" in bar.buttons and "follow" not in bar.buttons and "part" not in bar.buttons
    bar.buttons["top"].click()
    bar.buttons["section"].click()
    assert view.actors[0].mapper.GetNumberOfClippingPlanes() == 1
    pl.close()
