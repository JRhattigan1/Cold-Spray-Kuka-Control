"""S7K/19+3: PTP moves (kind PTP, or an E1 change) are simulated and collision-checked in joint
space, as the robot moves them, including HOME -> first point and last point -> HOME."""
from dataclasses import replace

import numpy as np

from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.solver import Context, find_issues, joint_sweep, ptp_in_joint_space, ptp_moves
from coldspray.post.joints import program_joints


def test_ptp_samples_on_a_joint_line_ends_kept():
    joints = np.array([[0.0] * 6, [5.0] * 6, [99.0] * 6, [-7.0] * 6, [30.0] * 6, [31.0] * 6])
    idx = np.array([0, 1, 2, 2, 2, 3])
    out = ptp_in_joint_space([False, False, True, False], idx, joints)
    assert np.allclose(out[[0, 1, 4, 5]], joints[[0, 1, 4, 5]])
    assert np.allclose(out[2:5, 0], [5 + 25 / 3, 5 + 50 / 3, 30])
    nan = joints.copy()
    nan[4] = np.nan                                     # unknown end: left as is
    assert np.isnan(ptp_in_joint_space([False, False, True, False], idx, nan)[4]).all()


def test_joint_sweep_steps_and_ends():
    q = joint_sweep([0] * 6, [12, 0, 0, 0, 0, -3])
    assert len(q) == 2 and np.allclose(q[:, 0], [4, 8]) and np.allclose(q[:, 5], [-1, -2])
    assert len(joint_sweep([0] * 6, [np.nan] * 6)) == 0


def _plan():
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = 2000.0
    _, path = plan_part(load_part("test_block"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                        roll_deg=-135.0)
    return cell, robot, path


def test_find_issues_moves_ptp_in_joint_space_and_keeps_krl_end_joints():
    cell, robot, path = _plan()
    k = len(path.moves) // 2
    path = replace(path, moves=[replace(m, kind="PTP", gun_on=False) if i == k else m
                                for i, m in enumerate(path.moves)])
    assert ptp_moves(path, cell)[k]
    s, joints, _ = find_issues(path, Context(robot, cell, log=lambda _: None))
    sel = np.flatnonzero(s.move_index == k)
    assert len(sel) > 2
    q0, q1 = joints[sel[0] - 1], joints[sel[-1]]
    t = (np.arange(1, len(sel) + 1) / len(sel))[:, None]
    assert np.allclose(joints[sel], (1 - t) * q0 + t * q1)
    assert np.allclose(program_joints(path, robot, cell)[k], q1)       # what the KRL commands


def test_e1_change_is_a_ptp():
    cell, _, path = _plan()
    k = len(path.moves) // 2
    p = replace(path, moves=[replace(m, axes=dict(m.axes) | {"E1": 1500.0}) if i == k else m
                             for i, m in enumerate(path.moves)])
    ptp = ptp_moves(p, cell)
    assert ptp[k] and not ptp[k + 1]


class _Checker:
    from coldspray.sim.collision import CollisionChecker
    check_path = CollisionChecker.check_path
    _home_sweeps = CollisionChecker._home_sweeps

    def __init__(self):
        self.cell, self.checked = load_cell(), []

    def _precompute(self, joints):
        return [None] * len(joints), None

    def _table(self, axes):
        return None

    def check(self, q, axes, _pre=None, _table=None):
        self.checked.append(np.array(q))
        return {("A5", "cell")} if q[0] > 50 else set()


def test_check_path_checks_home_sweeps():
    home = np.array(HOME_DEG, float)
    joints = np.tile(home, (5, 1))
    joints[:, 0] = home[0] + 100                         # whole path far from home on A1
    c = _Checker()
    plain = c.check_path(joints, lambda i: {})
    n_plain = len(c.checked)
    c2 = _Checker()
    hits = c2.check_path(joints, lambda i: {}, home=home)
    assert len(c2.checked) > n_plain                     # the two sweeps were checked too
    assert [h.sample for h in hits] == [h.sample for h in plain]       # merged into the end samples
