"""Datum planes: pure plane lookup and the cell-view actors (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
import pyvista as pv

from coldspray.cell import base_world, robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_robot, load_tool
from coldspray.sim.datums import Datums, datum_matrix, datum_plane
from coldspray.sim.viewer import Scene


def test_datum_plane_lookup():
    p, n = datum_plane("datum:XZ", 25.0)
    assert np.allclose(n, (0, 1, 0)) and np.allclose(p, (0, 25, 0))
    assert np.allclose(datum_plane("XY")[1], (0, 0, 1))
    assert np.allclose(datum_plane("YZ", -5)[0], (-5, 0, 0))
    assert np.allclose(datum_matrix("XY", 10)[:3, 3], (0, 0, 10))
    with pytest.raises(KeyError):
        datum_plane("datum:AB")


def test_datums_toggle_offset_name():
    pl = pv.Plotter(off_screen=True, window_size=(200, 150))
    d = Datums(pl)
    assert not any(d.visible(k) for k in ("XY", "XZ", "YZ"))
    d.set_visible("datum:XZ", True)
    assert d.visible("XZ") and d.actors["XZ"].GetPickable() and not d.visible("XY")
    d.set_offset("XZ", 40)
    p, n = d.plane("datum:XZ")
    assert np.allclose(p, (0, 40, 0)) and np.allclose(n, (0, 1, 0))
    assert np.allclose(np.asarray(d.actors["XZ"].user_matrix)[:3, 3], (0, 40, 0))
    assert d.name_of(d.actors["YZ"]) == "datum:YZ" and d.name_of(object()) is None


def test_scene_datums_follow_tilt():
    cell = load_cell()
    pl = pv.Plotter(off_screen=True, window_size=(400, 300))
    scene = Scene(pl, Robot(load_robot(), load_tool()), robroot(cell))
    tilt = cell.turntable.tilt.axis
    axes = {tilt: 30.0}
    scene.enter_setup(cell, axes)
    frame = base_world(cell, axes)
    for k in ("XY", "XZ", "YZ"):
        scene.datums.set_visible(k, True)
        assert scene.datums.actors[k].GetVisibility()
        assert np.allclose(np.asarray(scene.datums.actors[k].user_matrix), frame @ datum_matrix(k))
        assert scene.pick_name(scene.datums.actors[k]) == f"datum:{k}"
        scene.datums.set_visible(k, False)
        assert not scene.datums.actors[k].GetVisibility()
    # the XY normal tilts 30 deg away from vertical in WORLD
    nz = (frame[:3, :3] @ scene.datums.plane("XY")[1])[2]
    assert np.isclose(nz, np.cos(np.radians(30)), atol=1e-6)
    assert scene.pick_name(scene.part_actor) is None


def test_datum_menu_toggles_planes():
    from PySide6.QtWidgets import QApplication

    from coldspray.gui.view_toolbar import datum_button

    QApplication.instance() or QApplication([])
    cell = load_cell()
    pl = pv.Plotter(off_screen=True, window_size=(200, 150))
    scene = Scene(pl, Robot(load_robot(), load_tool()), robroot(cell))
    button = datum_button(scene, pl)
    act, off = button.actions_by_plane["YZ"]
    act.setChecked(True)
    assert scene.datums.visible("YZ") and not scene.datums.visible("XY")
    off.setValue(12.5)
    assert np.allclose(scene.datums.plane("datum:YZ")[0], (12.5, 0, 0))
    act.setChecked(False)
    assert not scene.datums.visible("YZ")
