"""Toolpath point markers (collision, unreachable, steep gun) can be hidden (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pyvista as pv

from coldspray.cell import robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.sim.collision import Collision
from coldspray.sim.viewer import Scene


def _scene_with_path():
    cell = load_cell()
    pl = pv.Plotter(off_screen=True, window_size=(300, 200))
    scene = Scene(pl, Robot(load_robot(), load_tool()), robroot(cell))
    path = plan_part(load_part("test_block"), load_process(), "layers", tilt_deg=90.0, cell=cell)[1]
    scene.set_path(path, cell, step=10.0)
    return scene, pl


def test_hidden_collision_markers_stay_hidden_on_new_hits():
    scene, _ = _scene_with_path()
    scene.set_collisions([Collision(3, {("A2", "Wall")}), Collision(5, {("A3", "Wall")})])
    assert scene.markers["collision"].GetVisibility()
    scene.set_marker_visible("collision", False)
    assert not scene.markers["collision"].GetVisibility()
    scene.set_collisions([Collision(7, {("A2", "Wall")})])          # a new check result
    assert not scene.markers["collision"].GetVisibility()
    scene.set_marker_visible("collision", True)
    assert scene.markers["collision"].GetVisibility()
    scene.set_collisions([])
    assert scene.markers["collision"] is None


def test_marker_menu_toggles_kinds():
    from PySide6.QtWidgets import QApplication

    from coldspray.gui.view_toolbar import marker_button

    QApplication.instance() or QApplication([])
    scene, pl = _scene_with_path()
    scene.set_collisions([Collision(3, {("A2", "Wall")})])
    button = marker_button(scene, pl)
    assert all(a.isChecked() for a in button.actions_by_kind.values())
    button.actions_by_kind["collision"].setChecked(False)
    assert not scene.marker_shown["collision"] and not scene.markers["collision"].GetVisibility()
    assert scene.marker_shown["unreachable"] and scene.marker_shown["steep"]
