import numpy as np
import pytest

from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.models import load_robot, load_tool
from coldspray.transforms import frame, xyzabc


@pytest.fixture(scope="module")
def robot():
    return Robot(load_robot())


def test_home_flange_position(robot):
    # Arm horizontal at home: x = a1 + c3 + c4, z = c1 + c2 - a2
    x, y, z, a, b, c = xyzabc(robot.flange(HOME_DEG))
    assert np.allclose([x, y, z], [175 + 1035 + 185, 0, 575 + 1290 + 50])
    assert np.allclose([a, b, c], 0, atol=1e-9)


def test_a5_tilts_flange_down(robot):
    # Positive A5 at home tips the flange downwards: KUKA shows B = A5
    _, _, _, a, b, c = xyzabc(robot.flange([0, -90, 90, 0, 60, 0]))
    assert np.allclose([a, b, c], [0, 60, 0], atol=1e-9)


def test_a1_positive_turns_towards_minus_y(robot):
    # KUKA A1 axis points down, so +A1 swings the arm clockwise seen from above
    _, y, *_ = xyzabc(robot.flange([30, -90, 90, 0, 0, 0]))
    assert y < 0


def test_xyzabc_round_trip():
    v = [100, -200, 300, 30, -20, 10]
    assert np.allclose(xyzabc(frame(*v)), v)


@pytest.mark.parametrize("seed", range(30))
def test_ik_recovers_reachable_joints(robot, seed):
    rng = np.random.default_rng(seed)
    lo, hi = robot.limits[:, 0], robot.limits[:, 1]
    q = rng.uniform(lo, hi)
    q[4] = rng.uniform(15, 120) * rng.choice([-1, 1])
    pose = robot.fk(q)
    sols = robot.ik(pose, seed_deg=q)
    assert sols
    assert np.allclose(sols[0], q, atol=1e-6)
    for s in sols:
        assert np.allclose(robot.fk(s), pose, atol=1e-6)


def test_ik_with_tool(robot):
    r = Robot(robot.cfg, load_tool())
    q = np.array([20, -80, 100, 10, 45, -30])
    sols = r.ik(r.fk(q), seed_deg=q)
    assert np.allclose(sols[0], q, atol=1e-6)


def test_ik_out_of_reach(robot):
    assert robot.ik(frame(4000, 0, 1000)) == []
