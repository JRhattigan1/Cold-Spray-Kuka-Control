"""OPW analytic kinematics for 6-axis arms with an offset spherical wrist.

Based on Brandstötter, Angerer & Hofbaur (2014), "An Analytical Solution of the
Inverse Kinematics Problem of Industrial Serial Manipulators with an
Ortho-parallel Basis and a Spherical Wrist".

Lengths are in millimetres, angles in radians. Poses are 4x4 homogeneous
matrices of the flange in the robot base frame, using the OPW convention
(flange Z axis points out of the flange). Mapping to KUKA's flange frame
(X out of the flange) is handled in robot.py.

Both directions work on many poses at once (numpy), which is how whole paths
are solved; the single-pose calls are the same code with one row.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OPWParams:
    a1: float
    a2: float
    b: float
    c1: float
    c2: float
    c3: float
    c4: float
    offsets: tuple[float, ...] = (0.0,) * 6
    signs: tuple[int, ...] = (1,) * 6


def forward(p: OPWParams, joints) -> np.ndarray:
    """Flange pose (4x4) for joint angles (6,), or poses (n, 4, 4) for (n, 6)."""
    joints = np.asarray(joints, dtype=float)
    q = np.atleast_2d(joints) * p.signs + p.offsets
    n = len(q)
    s1, s2, s3, s4, s5, s6 = np.sin(q).T
    c1, c2, c3, c4, c5, c6 = np.cos(q).T

    psi3 = np.arctan2(p.a2, p.c3)
    k = np.hypot(p.a2, p.c3)
    cx1 = p.c2 * s2 + k * np.sin(q[:, 1] + q[:, 2] + psi3) + p.a1
    cy1 = p.b
    cz1 = p.c2 * c2 + k * np.cos(q[:, 1] + q[:, 2] + psi3)
    wrist = np.stack([cx1 * c1 - cy1 * s1, cx1 * s1 + cy1 * c1, cz1 + p.c1], axis=1)

    zero = np.zeros(n)
    r_0c = np.stack([c1 * c2 * c3 - c1 * s2 * s3, -s1, c1 * c2 * s3 + c1 * s2 * c3,
                     s1 * c2 * c3 - s1 * s2 * s3, c1, s1 * c2 * s3 + s1 * s2 * c3,
                     -s2 * c3 - c2 * s3, zero, -s2 * s3 + c2 * c3], axis=1).reshape(n, 3, 3)
    r_ce = np.stack([c4 * c5 * c6 - s4 * s6, -c4 * c5 * s6 - s4 * c6, c4 * s5,
                     s4 * c5 * c6 + c4 * s6, -s4 * c5 * s6 + c4 * c6, s4 * s5,
                     -s5 * c6, s5 * s6, c5], axis=1).reshape(n, 3, 3)
    rot = r_0c @ r_ce

    pose = np.zeros((n, 4, 4))
    pose[:, :3, :3] = rot
    pose[:, :3, 3] = wrist + p.c4 * rot[:, :, 2]
    pose[:, 3, 3] = 1.0
    return pose[0] if joints.ndim == 1 else pose


def _acos_or_nan(x):
    with np.errstate(invalid="ignore"):
        return np.where((x >= -1.0) & (x <= 1.0), np.arccos(np.clip(x, -1.0, 1.0)), np.nan)


def inverse_all(p: OPWParams, poses) -> np.ndarray:
    """Joint solutions for (n, 4, 4) flange poses: (n, 8, 6) radians, the 8
    branches in a fixed order, NaN rows for unreachable branches. Angles are
    wrapped to [-pi, pi]; joint limits are not applied here."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 4, 4)
    rot = poses[:, :3, :3]
    cx, cy, cz = (poses[:, :3, 3] - p.c4 * rot[:, :, 2]).T
    with np.errstate(invalid="ignore", divide="ignore"):
        nx1 = np.sqrt(np.maximum(cx**2 + cy**2 - p.b**2, 0.0)) - p.a1
        tmp1 = np.arctan2(cy, cx)
        tmp2 = np.arctan2(p.b, nx1 + p.a1)
        t1_i = tmp1 - tmp2
        t1_ii = tmp1 + tmp2 - np.pi

        dz = cz - p.c1
        s1_2 = nx1**2 + dz**2
        s2_2 = (nx1 + 2 * p.a1) ** 2 + dz**2
        kappa_2 = p.a2**2 + p.c3**2
        c2_2 = p.c2**2
        psi3 = np.arctan2(p.a2, p.c3)

        s1, s2 = np.sqrt(s1_2), np.sqrt(s2_2)
        a = _acos_or_nan((s1_2 + c2_2 - kappa_2) / (2 * s1 * p.c2))
        b = np.arctan2(nx1, dz)
        c = _acos_or_nan((s2_2 + c2_2 - kappa_2) / (2 * s2 * p.c2))
        d = np.arctan2(nx1 + 2 * p.a1, dz)
        e = _acos_or_nan((s1_2 - c2_2 - kappa_2) / (2 * p.c2 * np.sqrt(kappa_2)))
        f = _acos_or_nan((s2_2 - c2_2 - kappa_2) / (2 * p.c2 * np.sqrt(kappa_2)))

    arms = [
        (t1_i, -a + b, e - psi3),
        (t1_i, a + b, -e - psi3),
        (t1_ii, -c - d, f - psi3),
        (t1_ii, c - d, -f - psi3),
    ]

    def r(i, j):
        return rot[:, i, j]

    offsets, signs = np.asarray(p.offsets, float), np.asarray(p.signs, float)
    out = np.full((len(poses), 8, 6), np.nan)
    for k, (t1, t2, t3) in enumerate(arms):
        bad = np.isnan(t2) | np.isnan(t3)
        st1, ct1 = np.sin(t1), np.cos(t1)
        s23, c23 = np.sin(t2 + t3), np.cos(t2 + t3)
        m = r(0, 2) * s23 * ct1 + r(1, 2) * s23 * st1 + r(2, 2) * c23
        t5 = np.arctan2(np.sqrt(np.maximum(1.0 - m * m, 0.0)), m)
        t4 = np.arctan2(
            r(1, 2) * ct1 - r(0, 2) * st1,
            r(0, 2) * c23 * ct1 + r(1, 2) * c23 * st1 - r(2, 2) * s23,
        )
        t6 = np.arctan2(
            r(0, 1) * s23 * ct1 + r(1, 1) * s23 * st1 + r(2, 1) * c23,
            -r(0, 0) * s23 * ct1 - r(1, 0) * s23 * st1 - r(2, 0) * c23,
        )
        for w, (w4, w5, w6) in enumerate(((t4, t5, t6), (t4 + np.pi, -t5, t6 - np.pi))):
            q = np.stack([t1, t2, t3, w4, w5, w6], axis=1)
            q = (q - offsets) * signs
            q = (q + np.pi) % (2 * np.pi) - np.pi
            q[bad] = np.nan
            out[:, 2 * k + w] = q
    return out


def inverse(p: OPWParams, pose) -> list[np.ndarray]:
    """All (up to 8) joint solutions reaching one flange pose; unreachable
    branches are dropped (see inverse_all)."""
    sols = inverse_all(p, pose)[0]
    return list(sols[~np.isnan(sols[:, 0])])
