"""Position step's warm-up / purge Section drives the settings and the ghost gun (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from types import SimpleNamespace

import numpy as np
import pytest
import pyvista as pv
from PySide6.QtWidgets import QApplication

from coldspray.cell import robroot
from coldspray.gui.steps.position import PositionStep
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_robot, load_tool
from coldspray.sim.markers import DIRECTION_PRESETS, marker_pose
from coldspray.sim.viewer import Scene


class Host(PositionStep):
    """Just enough of MainWindow for the purge Section."""

    def __init__(self):
        cell = load_cell()
        robot = Robot(load_robot(), load_tool())
        self.assets = SimpleNamespace(cell=cell, robot=robot)
        self.pl = pv.Plotter(off_screen=True, window_size=(200, 150))
        self.scene = Scene(self.pl, robot, robroot(cell))
        self.section = self._build_purge_section()

    def _work_cell(self):
        return self.assets.cell.model_copy(deep=True)


@pytest.fixture
def host():
    QApplication.instance() or QApplication([])
    h = Host()
    yield h
    h.pl.close()


def settings(h):
    cfg = getattr(h.assets.cell, "purge", None)
    return h.purge_cfg if cfg is None else cfg.model_dump()


def test_values_update_settings_and_marker(host):
    host.purge_on.setChecked(True)
    for box, v in zip(host.purge_xyz, (100.0, -300.0, 1800.0)):
        box.setValue(v)
    host.purge_dir.setCurrentText("+X")
    host.purge_dwell.setValue(8)
    host.purge_every.setValue(3)
    host.purge_approach.setValue(150)
    host.purge_end.setChecked(False)
    c = settings(host)
    assert c["enabled"] and np.allclose(c["point"], (100, -300, 1800))
    assert np.allclose(c["spray_dir"], DIRECTION_PRESETS["+X"])
    assert c["dwell_s"] == 8 and c["every_n_layers"] == 3 and c["at_end"] is False and c["approach_mm"] == 150
    m = host.scene.purge_marker
    assert m.visible
    assert np.allclose(m.pose, marker_pose((100, -300, 1800), (1, 0, 0)))
    assert host.purge_section.chip.text() in ("reachable", "out of reach")


def test_disabled_hides_marker_and_far_point_is_out_of_reach(host):
    host.purge_on.setChecked(True)
    host.purge_xyz[2].setValue(19000)
    assert host.purge_section.chip.text() == "out of reach"
    host.purge_on.setChecked(False)
    assert not host.scene.purge_marker.visible
    assert host.purge_section.chip.text() == "off"


def test_custom_direction_boxes(host):
    host.purge_on.setChecked(True)
    host.purge_dir.setCurrentText("Custom")
    assert not host.purge_custom[0].isHidden()
    for box, v in zip(host.purge_custom, (0.0, 0.6, -0.8)):
        box.setValue(v)
    assert np.allclose(settings(host)["spray_dir"], (0.0, 0.6, -0.8))
    host.purge_dir.setCurrentText("+X")
    assert host.purge_custom[0].isHidden()
    assert np.allclose(settings(host)["spray_dir"], DIRECTION_PRESETS["+X"])


def test_save_button_writes_cell_yaml(host, tmp_path, monkeypatch):
    import shutil
    import coldspray.config_io as cio
    from coldspray.models import CONFIG_DIR
    shutil.copyfile(CONFIG_DIR / "cell.yaml", tmp_path / "cell.yaml")
    monkeypatch.setattr(cio, "CONFIG_DIR", tmp_path)
    msgs = []
    host._status = msgs.append
    host.purge_on.setChecked(True)
    host.purge_dwell.setValue(9)
    host.purge_save_btn.click()
    saved = load_cell(tmp_path / "cell.yaml").purge
    assert saved.enabled and saved.dwell_s == 9
    assert msgs and "saved" in msgs[0]


def test_move_gun_here(host, monkeypatch):
    msgs, poses = [], []
    host._status = msgs.append
    monkeypatch.setattr(host.scene, "set_robot_pose", poses.append)
    host.purge_cfg["point"] = (1e5, 0.0, 0.0)
    host.purge_goto_btn.click()
    assert not poses and "out of reach" in msgs[-1]
    host.purge_cfg["point"] = host.PURGE_DEFAULTS["point"]
    host.purge_goto_btn.click()
    assert len(poses) == 1
