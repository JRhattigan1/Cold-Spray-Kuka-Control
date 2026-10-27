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
from coldspray.gui import widgets
from coldspray.models import PurgeConfig, load_cell, load_robot, load_tool
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
    return h.assets.cell.purge.model_dump()


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
    host.purge_cfg["point"] = PurgeConfig(point=(-884.56, 400.0, 2000.0)).point
    host.purge_goto_btn.click()
    assert len(poses) == 1


def test_no_f2_workarounds_left():
    """F2 shims are gone now PurgeConfig is on main (P6)."""
    import inspect
    import coldspray.gui.steps.position as pos
    src = inspect.getsource(pos)
    assert "model_fields" not in src and "PURGE_DEFAULTS" not in src and "hasattr(" not in src


class PanelHost(PositionStep):
    """Enough of MainWindow to build the whole Position panel."""

    def __init__(self):
        from coldspray.cad.part import Substrate
        self.substrate = Substrate()
        self.part = None
        self.scene = None
        self.panel = self._build_position_panel()

    def _enter_setup(self, *_):
        pass


@pytest.fixture
def panel():
    QApplication.instance() or QApplication([])
    widgets._reset_mode_state()
    h = PanelHost()
    h.panel.show()
    yield h
    h.panel.close()
    widgets._reset_mode_state()


def test_simple_mode_shows_only_the_essentials(panel):
    h = panel
    widgets.set_mode(widgets.SIMPLE)
    for k in "XYA":
        assert h.place[k].isVisible(), k
    for k in "ZBC":
        assert not h.place[k].isVisible(), k
    assert h.keep_on_plate.isVisible() and h.auto_btn.isVisible() and h.purge_on.isVisible()
    for w in (h.e1, h.tilt, h.roll, h.nozzle, h.snap_btn, h.purge_dwell, h.purge_xyz[0],
              h.purge_save_btn, h.sub_check, h.constraints, *h.group_checks.values()):
        assert not w.isVisible(), w
    widgets.set_mode(widgets.ADVANCED)
    for w in (h.place["Z"], h.e1, h.snap_btn, h.purge_dwell, h.sub_check, h.constraints):
        assert w.isVisible(), w


def test_simple_mode_never_hides_non_default_values(panel):
    h = panel
    widgets.set_mode(widgets.SIMPLE)
    h.tilt.setMaximum(90)
    h.tilt.setValue(30)
    h.e1.setMaximum(5000)
    h.e1.setValue(100)
    h.roll.setValue(20)
    h.place["Z"].setValue(5)
    h.sub_fields["length"].setValue(h.sub_fields["length"].value() + 10)
    for w in (h.tilt, h.e1, h.roll, h.place["Z"], h.sub_fields["length"]):
        assert w.isVisible(), w
    assert not h.place["B"].isVisible() and not h.rotation.isVisible()
    h.tilt.setValue(0)
    assert not h.tilt.isVisible()
    h.sub_check.setChecked(False)
    assert h.sub_check.isVisible()
    h.purge_on.setChecked(True)
    h.purge_dwell.setValue(9)
    assert h.purge_dwell.isVisible()
