"""Gun held at a fixed angle: wrist above the gun, spray direction untouched."""

import numpy as np

from coldspray.cell import base_world
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.gun import level_gun, wrist_side
from coldspray.planner.position import auto_position
from coldspray.toolpath.model import Move, Toolpath, spray_frame


def _robot():
    return Robot(load_robot(), load_tool())


def _wrist_up(path, cell, wrist):
    """WORLD z of the wrist direction at every pose, and the most it could be (wrist square to the spray)."""
    out, best = [], []
    for m, axes in zip(path.moves, path.axis_values()):
        r = base_world(cell, axes)[:3, :3] @ m.pose[:3, :3]
        out.append((r @ wrist)[2])
        best.append(np.sqrt(max(0.0, 1.0 - r[2, 0] ** 2)))
    return np.array(out), np.array(best)


def test_wrist_side_is_tool_y():
    assert np.allclose(wrist_side(_robot().tool), [0.0, 1.0, 0.0])


def test_level_gun_puts_wrist_above_horizontal_spray():
    cell, wrist = load_cell(), [0.0, 1.0, 0.0]
    poses = [spray_frame((x, 0.0, 30.0), (0, 0, -1), d) for x, d in ((0, (1, 0, 0)), (10, (0, 1, 0)))]
    path = Toolpath("t", [Move(p, 50.0, True, axes={"E2": 90.0, "E5": 30.0}) for p in poses])
    out = level_gun(path, cell, wrist)
    up, best = _wrist_up(out, cell, wrist)
    assert np.allclose(up, best, atol=1e-6) and np.all(best > 0.99)       # wrist straight up
    for a, b in zip(path.moves, out.moves):                                # spray point and axis kept
        assert np.allclose(a.pose[:3, 3], b.pose[:3, 3]) and np.allclose(a.pose[:3, 0], b.pose[:3, 0])


def test_gun_angle_turns_from_wrist_up():
    cell, wrist = load_cell(), [0.0, 1.0, 0.0]
    path = Toolpath("t", [Move(spray_frame((0, 0, 30), (0, 0, -1), (1, 0, 0)), 50.0, axes={"E2": 90.0})])
    up, best = _wrist_up(level_gun(path, cell, wrist, angle_deg=60.0), cell, wrist)
    assert np.isclose(up[0], best[0] * np.cos(np.radians(60)), atol=1e-6)


def test_near_vertical_spray_keeps_planner_roll():
    cell, wrist = load_cell(), [0.0, 1.0, 0.0]
    pose = spray_frame((0, 0, 30), (0, 0, -1), (1, 0, 0))
    path = Toolpath("t", [Move(pose, 50.0, axes={"E2": 0.0})])                 # table flat: gun points down
    assert np.allclose(level_gun(path, cell, wrist).moves[0].pose, pose)


def test_plan_part_with_wrist_levels_layers():
    cell, proc, robot = load_cell(), load_process(), _robot()
    wrist = wrist_side(robot.tool)
    part = load_part("test_block")
    path = plan_part(part, proc, "layers", tilt_deg=90.0, cell=cell, wrist=wrist)[1]
    up, best = _wrist_up(path, cell, wrist)
    assert np.allclose(up, best, atol=1e-6)


def test_auto_position_keeps_the_gun_angle():
    cell, proc, robot = load_cell(), load_process(), _robot()
    wrist, part = wrist_side(robot.tool), load_part("test_block")

    def plan(tilt):
        return plan_part(part, proc, "layers", tilt_deg=tilt, cell=cell, wrist=wrist)[1]

    pos = auto_position(plan, robot, cell, [90.0], rolls=(0.0,), log=lambda _: None)
    assert pos.roll_deg == 0.0
