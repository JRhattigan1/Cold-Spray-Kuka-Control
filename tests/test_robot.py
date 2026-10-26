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


@pytest.mark.parametrize("q", [[0, 0, 0, 0, 0, 0], [30, -60, 100, 45, -30, 120], [-100, -120, 20, -170, 90, -300]])
def test_link_frames_end_at_flange(robot, q):
    t6 = robot.link_frames(q)[-1]
    flange_opw = t6.copy()
    flange_opw[:3, 3] += robot.params.c4 * t6[:3, 2]
    assert np.allclose(flange_opw @ robot._opw_to_kuka, robot.flange(q), atol=1e-9)


def test_link_frames_follow_their_joint(robot):
    # Moving only A5 leaves A1..A4 where they were
    a, b = robot.link_frames([10, -80, 95, 20, 0, 0]), robot.link_frames([10, -80, 95, 20, 70, 0])
    for k in range(4):
        assert np.allclose(a[k], b[k])
    assert not np.allclose(a[4], b[4])


def test_ik_out_of_reach(robot):
    assert robot.ik(frame(4000, 0, 1000)) == []
