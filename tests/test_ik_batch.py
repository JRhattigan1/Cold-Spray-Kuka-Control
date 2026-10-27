"""Vectorised IK (Robot.ik_path / _candidates) matches the per-pose OPW solver."""
import numpy as np
import pytest

from coldspray.kinematics import opw
from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.models import load_robot, load_tool


@pytest.fixture(scope="module")
def robot():
    return Robot(load_robot(), load_tool())


def _ref_in_limits(robot, pose):
    target = np.asarray(pose) @ robot._to_opw
    sols = np.degrees(np.asarray(opw.inverse(robot.params, target), dtype=float)).reshape(-1, 6)
    cands = np.repeat(sols, 3, axis=0)
    cands[:, 5] += np.tile([-360.0, 0.0, 360.0], len(sols))
    ok = np.all((cands >= robot.limits[:, 0]) & (cands <= robot.limits[:, 1]), axis=1)
    return cands[ok]


def _ref_ik_path(robot, poses, seed, hint=None):
    out = np.full((len(poses), 6), np.nan)
    seed = np.asarray(seed, float)
    for i, pose in enumerate(poses):
        if hint is not None and not np.isnan(hint[i, 0]):
            seed = np.asarray(hint[i], float)
        c = _ref_in_limits(robot, pose)
        if len(c):
            d = np.max(np.abs(c - seed), axis=1)
            out[i] = seed = c[np.argsort(d, kind="stable")][0]
    return out


def _poses(robot, n=300, seed=0):
    rng = np.random.default_rng(seed)
    q = rng.uniform(robot.limits[:, 0], robot.limits[:, 1], (n, 6))
    poses = np.array([robot.fk(x) for x in q])
    poses[::7, :3, 3] += 5000.0              # unreachable
    poses[::11, :3, 3] *= 0.01               # near the base: some branches fail
    return poses


def test_candidates_match_scalar_solver(robot):
    poses = _poses(robot)
    batch = robot._candidates(poses)
    for pose, c in zip(poses, batch):
        ref = _ref_in_limits(robot, pose)
        got = c[~np.isnan(c[:, 0])]
        assert got.shape == ref.shape
        assert np.allclose(got, ref, atol=1e-6)
        assert np.allclose(np.array(robot.ik_all(pose)).reshape(-1, 6), ref, atol=1e-6)


def test_ik_path_matches_loop(robot):
    rng = np.random.default_rng(1)
    q = np.cumsum(rng.normal(0, 3, (400, 6)), axis=0) + [0, -60, 90, 0, 40, 0]
    poses = np.array([robot.fk(x) for x in q])
    poses[50:60, :3, 3] += 5000.0
    hint = np.full((len(poses), 6), np.nan)
    hint[200] = q[200]
    for h in (None, hint):
        got = robot.ik_path(list(poses), seed_deg=HOME_DEG, hint=h)
        ref = _ref_ik_path(robot, poses, HOME_DEG, h)
        assert np.array_equal(np.isnan(got), np.isnan(ref))
        assert np.allclose(got[~np.isnan(got)], ref[~np.isnan(ref)], atol=1e-6)


def test_ik_path_empty(robot):
    assert robot.ik_path([]).shape == (0, 6)
