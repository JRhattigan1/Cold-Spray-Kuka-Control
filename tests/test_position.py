import numpy as np
import pytest

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.position import auto_position, gun_down_deg, layer_tilts, world_poses
from coldspray.toolpath.model import Move, Toolpath
from coldspray.transforms import frame


def test_gun_down_angle():
    down = frame(b=90)       # tool X -> -Z
    level = frame()          # tool X -> +X
    up = frame(b=-30)        # tool X tilted 30 deg up
    assert np.allclose(gun_down_deg([down, level, up]), [90, 0, -30])


def test_roll_keeps_spray_point_and_direction():
    pose = frame(10, 20, 30, 15, 70, 5)
    p = Toolpath("t", [Move(pose, 50.0, True, axes={"E2": 10.0})]).rolled(45)
    m = p.moves[0]
    assert np.allclose(m.pose[:3, 3], pose[:3, 3])
    assert np.allclose(m.pose[:3, 0], pose[:3, 0])           # spray axis unchanged
    assert not np.allclose(m.pose[:3, 1], pose[:3, 1])
    assert (m.speed, m.gun_on, m.axes) == (50.0, True, {"E2": 10.0})


@pytest.mark.parametrize("limit, expected", [(45, [90, 75, 60, 45]), (20, [90, 75]), (0, [90])])
def test_layer_tilts_respect_gun_limit(limit, expected):
    cell = load_cell()
    cell.gun_max_down_deg = limit
    assert layer_tilts(cell) == expected


def test_auto_position_levels_the_gun_for_layers():
    cell, proc = load_cell(), load_process()
    robot = Robot(load_robot(), load_tool())
    part = load_part("test_block")
    plan = lambda tilt: plan_part(part, proc, "layers", tilt_deg=tilt, cell=cell)[1]
    pos = auto_position(plan, robot, cell, layer_tilts(cell), log=lambda _: None)
    assert pos.reach == 1.0
    assert pos.tilt_deg == 90.0
    path = plan_part(part, proc, "layers", tilt_deg=pos.tilt_deg, cell=cell, roll_deg=pos.roll_deg)[1]
    assert gun_down_deg(world_poses(path, cell)).max() < 1e-6


def test_layer_tilts_for_a_sideways_build():
    cell = load_cell()
    assert layer_tilts(cell, gun_dir=(1.0, 0.0, 0.0))[0] == 0.0     # already level with the table flat
    assert layer_tilts(cell, gun_dir=(0.0, 0.0, -1.0)) == [90, 75, 60, 45, 30]   # 70 deg limit allows 30 deg tilt


def test_auto_position_names_an_unreachable_purge_point():
    from coldspray.cell import base_world
    from coldspray.models import PurgeConfig
    cell, proc = load_cell(), load_process()
    robot = Robot(load_robot(), load_tool())
    far = tuple(float(x) for x in base_world(cell)[:3, 3] + [0.0, 20000.0, 300.0])
    cell.purge = PurgeConfig(point=far, spray_dir=(0.0, 0.0, -1.0))
    part = load_part("test_block")
    plan = lambda tilt: plan_part(part, proc, "layers", tilt_deg=tilt, cell=cell)[1]
    logs = []
    pos = auto_position(plan, robot, cell, [90.0], e1_step=5000.0, log=logs.append)
    assert pos.reach < 1.0
    assert "purge point" in pos.note.lower()
    assert f"{far[1]:.0f}" in pos.note
    assert any(pos.note in line for line in logs)
