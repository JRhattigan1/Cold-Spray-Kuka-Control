"""S9H: the KRL's HOME legs are in the solver's find_issues, HOME -> first is moved as the KRL
issues it (LIN when move 0 is a LIN), and the drawn path follows PTP moves in joint space."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from coldspray.cell import axis_defaults
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.solver import Context, find_issues, home_first_leg, joint_sweep, robot_targets


def _plan():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = 2000.0
    _, path = plan_part(load_part("test_block"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                        roll_deg=-135.0)
    return cell, robot, path


class _LegChecker:
    """Collides only away from the path: on joints the path itself never has."""
    def __init__(self, path_joints):
        self.seen = {np.asarray(q, float).tobytes() for q in path_joints}

    def check(self, q, axes):
        return set() if np.asarray(q, float).tobytes() in self.seen else {("A3", "Wall")}


def test_find_issues_reports_home_legs():
    cell, robot, path = _plan()
    ctx = Context(robot, cell, log=lambda _: None)
    s, joints, base = find_issues(path, ctx)
    assert not [i for i in base if i.kind == "collision"]
    ctx2 = Context(robot, cell, _LegChecker(joints), log=lambda _: None)
    _, _, issues = find_issues(path, ctx2)
    legs = [i for i in issues if i.kind == "collision"]
    assert [i.sample for i in legs] == [0, len(s) - 1]
    assert legs[0].detail.startswith("HOME -> first") and legs[1].detail.startswith("last -> HOME")
    ctx3 = Context(robot, cell, _LegChecker(joints), seed=joints[0], log=lambda _: None)
    assert not [i for i in find_issues(path, ctx3)[2] if i.kind == "collision"]   # seeded: no HOME legs


def test_home_first_leg_lin_is_a_straight_tcp_line():
    cell, robot, path = _plan()
    if path.moves[0].kind == "PTP":
        path = replace(path, moves=[replace(path.moves[0], kind="LIN")] + path.moves[1:])
    s = path.sample(5.0, 5.0, axis_defaults(cell))
    target0 = robot_targets(s, cell)[0]
    q0 = robot.ik(target0, cell.home_deg)[0]
    qs = home_first_leg(robot, cell.home_deg, target0, q0, ptp=False)
    assert len(qs) > 2 and not np.isnan(qs).any()
    p = np.array([robot.fk(q)[:3, 3] for q in qs])
    a, b = robot.fk(np.asarray(cell.home_deg, float))[:3, 3], target0[:3, 3]
    d = (b - a) / np.linalg.norm(b - a)
    off = (p - a) - np.outer((p - a) @ d, d)
    assert np.abs(off).max() < 1e-3                                     # on the LIN, not a joint line
    assert np.allclose(home_first_leg(robot, cell.home_deg, target0, q0, ptp=True),
                       joint_sweep(cell.home_deg, q0))


def test_drawn_path_follows_ptp_in_joint_space():
    from coldspray.sim.viewer import Scene
    cell, robot, path = _plan()
    k = len(path.moves) // 2
    path = replace(path, moves=[replace(m, kind="PTP", gun_on=False) if i == k else m
                                for i, m in enumerate(path.moves)])
    s, joints, _ = find_issues(path, Context(robot, cell, log=lambda _: None))
    from coldspray.cell import base_worlds, robroot
    bases = base_worlds(cell, [s.axes_at(i) for i in range(len(s))])
    roots = [robroot(cell, 2000.0)] * len(s)
    drawn = []

    class Pl:
        def add_mesh(self, mesh, **kw):
            drawn.append(np.array(mesh.points))
            return SimpleNamespace(SetPickable=lambda _: None)

        def remove_actor(self, a):
            pass

    fake = SimpleNamespace(line_actors=[], path_actors=[], samples=s, layer_filter=("all", None), n_layers=0,
                           joints=joints, root_frames=roots, base_frames=bases, robot=robot, pl=Pl())
    Scene._draw_lines(fake)
    pts = drawn[0]
    sel = np.flatnonzero(s.move_index == k)
    planned = s.poses[sel, :3, 3]
    assert np.allclose(pts[sel[-1]], planned[-1], atol=1e-3)             # end point kept
    assert np.abs(pts[sel[:-1]] - planned[:-1]).max() > 0.05             # middle off the straight line (short move: small bulge)
    other = np.flatnonzero((s.move_index != k) & ~np.isnan(joints[:, 0]))
    assert np.allclose(pts[other], s.poses[other, :3, 3])                # LIN samples as planned
