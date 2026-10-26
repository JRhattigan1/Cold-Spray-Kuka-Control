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
