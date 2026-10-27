"""Purge point ghost gun marker (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pyvista as pv

from coldspray.sim.markers import DIRECTION_PRESETS, GunMarker, marker_pose, preset_for


def test_marker_pose_puts_tool_x_along_spray():
    for d in list(DIRECTION_PRESETS.values()) + [(0, 0, -1)]:
        pose = marker_pose((100, 200, 300), d)
        assert np.allclose(pose[:3, 0], np.asarray(d) / np.linalg.norm(d))
        assert np.allclose(pose[:3, 3], (100, 200, 300))
        assert np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-9)


def test_preset_for_round_trips():
    for name, d in DIRECTION_PRESETS.items():
        assert preset_for(np.asarray(d) * 3) == name
    assert preset_for((1, 1, 0)) == "Custom"


def test_gun_marker_pose_and_visibility():
    pl = pv.Plotter(off_screen=True, window_size=(200, 150))
    m = GunMarker(pl)
    pose = marker_pose((500, -200, 900), (0, -1, 0))
    m.set_pose(pose)
    for a in m.actors:
        assert np.allclose(np.asarray(a.user_matrix), pose)
    m.set_visible(False)
    assert not m.visible
    m.set_visible(True)
    assert m.visible
    pl.close()
