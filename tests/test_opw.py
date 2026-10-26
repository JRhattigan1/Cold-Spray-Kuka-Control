import numpy as np
import pytest

from coldspray.kinematics.opw import OPWParams, forward, inverse

# Published OPW parameters for a KUKA KR6 R700 sixx (opw_kinematics project).
# Used only to exercise the maths; the KR 50 R2500 values live in config.
KR6 = OPWParams(
    a1=25.0, a2=-35.0, b=0.0, c1=400.0, c2=315.0, c3=365.0, c4=80.0,
    offsets=(0.0, -np.pi / 2, 0.0, 0.0, 0.0, 0.0),
    signs=(-1, 1, 1, -1, 1, -1),
)


def _close_angles(a, b, tol=1e-6):
    d = (np.asarray(a) - np.asarray(b) + np.pi) % (2 * np.pi) - np.pi
    return np.all(np.abs(d) < tol)


def test_forward_is_rigid_transform():
    pose = forward(KR6, [0.3, -0.5, 0.8, 0.1, 0.7, -0.4])
    rot = pose[:3, :3]
    assert np.allclose(rot @ rot.T, np.eye(3))
    assert np.isclose(np.linalg.det(rot), 1.0)


@pytest.mark.parametrize("seed", range(50))
def test_inverse_recovers_joints(seed):
    rng = np.random.default_rng(seed)
    q = rng.uniform(-2.5, 2.5, 6)
    q[4] = rng.uniform(0.2, 2.5) * rng.choice([-1, 1])  # stay off the wrist singularity
    pose = forward(KR6, q)

    sols = inverse(KR6, pose)
    assert sols, "no IK solution for a reachable pose"
    assert any(_close_angles(s, q) for s in sols)
    for s in sols:
        assert np.allclose(forward(KR6, s), pose, atol=1e-6)


def test_unreachable_pose_has_no_solutions():
    pose = np.eye(4)
    pose[:3, 3] = [5000.0, 0.0, 400.0]
    assert inverse(KR6, pose) == []
