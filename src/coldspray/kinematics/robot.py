"""KUKA robot model: OPW kinematics wrapped in KUKA conventions.

Joints are A1..A6 in degrees. Poses are 4x4 frames in mm, in the robot base
frame. The flange frame follows KUKA ($TOOL = NULLFRAME): X out of the flange,
identity orientation at A1..A6 = (0, -90, 90, 0, 0, 0).
"""

import numpy as np

from coldspray.kinematics import opw
from coldspray.models import RobotConfig, ToolConfig
from coldspray.tool import tool_frame

HOME_DEG = np.array([0.0, -90.0, 90.0, 0.0, 0.0, 0.0])


class Robot:
    def __init__(self, cfg: RobotConfig, tool: ToolConfig | None = None):
        o = cfg.opw
        self.cfg = cfg
        self.params = opw.OPWParams(
            o.a1, o.a2, o.b, o.c1, o.c2, o.c3, o.c4,
            offsets=tuple(np.radians(o.offsets_deg)),
            signs=tuple(o.signs),
        )
        self.limits = np.array(cfg.joint_limits_deg, dtype=float)
        # Fixed rotation from the OPW flange frame to KUKA's flange frame,
        # chosen so the flange orientation is identity at HOME.
        home_rot = opw.forward(self.params, np.radians(HOME_DEG))[:3, :3]
        self._opw_to_kuka = np.eye(4)
        self._opw_to_kuka[:3, :3] = home_rot.T
        self.tool = tool_frame(tool)
        self._to_opw = np.linalg.inv(self.tool) @ np.linalg.inv(self._opw_to_kuka)   # TCP pose -> OPW flange

    def flange(self, joints_deg) -> np.ndarray:
        """Flange pose for joints (6,), or poses (n, 4, 4) for (n, 6)."""
        return opw.forward(self.params, np.radians(joints_deg)) @ self._opw_to_kuka

    def fk(self, joints_deg) -> np.ndarray:
        """TCP pose for joint angles A1..A6 (degrees)."""
        return self.flange(joints_deg) @ self.tool

    def ik(self, tcp_pose, seed_deg=HOME_DEG) -> list[np.ndarray]:
        """Joint solutions within limits, closest to `seed_deg` first.

        Axes with more than ±180° of travel (A6) also get their ±360° variants.
        """
        cands = self._in_limits(tcp_pose)
        if not len(cands):
            return []
        dist = np.max(np.abs(cands - np.asarray(seed_deg, dtype=float)), axis=1)
        return list(cands[np.argsort(dist, kind="stable")])

    def ik_all(self, tcp_pose) -> list[np.ndarray]:
        """Every in-limit joint solution for a TCP pose (A6 also at +-360 deg), unsorted."""
        return list(self._in_limits(tcp_pose))

    def _in_limits(self, tcp_pose) -> np.ndarray:
        """(n, 6) in-limit solutions, in OPW order with A6 -360 / 0 / +360 variants."""
        cands = self._candidates(np.asarray(tcp_pose, dtype=float)[None])[0]
        return cands[~np.isnan(cands[:, 0])]

    def _candidates(self, tcp_poses) -> np.ndarray:
        """(n, 24, 6) solutions for n TCP poses: the 8 OPW branches, each with A6
        -360 / 0 / +360, in OPW order. Unreachable or out-of-limit rows are NaN."""
        targets = np.asarray(tcp_poses, dtype=float).reshape(-1, 4, 4) @ self._to_opw
        sols = np.degrees(opw.inverse_all(self.params, targets))           # (n, 8, 6)
        cands = np.repeat(sols, 3, axis=1)
        cands[:, :, 5] += np.tile([-360.0, 0.0, 360.0], 8)
        with np.errstate(invalid="ignore"):
            ok = np.all((cands >= self.limits[:, 0]) & (cands <= self.limits[:, 1]), axis=2)
        cands[~ok] = np.nan
        return cands

    def ik_path(self, poses, seed_deg=HOME_DEG, hint=None) -> np.ndarray:
        """Joint angles for a sequence of TCP poses, each solved from the
        previous one so the arm configuration stays continuous.
        Unreachable poses give a row of NaN.
        """
        out = np.full((len(poses), 6), np.nan)
        if not len(poses):
            return out
        cands = self._candidates(np.asarray(poses, dtype=float))               # (n, 24, 6)
        valid = ~np.isnan(cands[:, :, 0])
        any_ok = valid.any(axis=1)
        filled = np.where(np.isnan(cands), np.inf, cands)
        seed = np.asarray(seed_deg, dtype=float)
        use_hint = np.zeros(len(poses), bool) if hint is None else ~np.isnan(np.asarray(hint)[:, 0])
        for i in range(len(poses)):
            if use_hint[i]:
                seed = np.asarray(hint[i], dtype=float)     # solver's chosen configuration
            if any_ok[i]:
                c = filled[i]
                out[i] = seed = c[np.argmin(np.max(np.abs(c - seed), axis=1))]
        return out

    def link_frames(self, joints_deg) -> list[np.ndarray]:
        """Frames of the moving links A1..A6 in the robot root frame.

        Each frame turns with its own joint, so a link's CAD geometry drawn at
        pose q0 moves to pose q by  link_frames(q)[k] @ inv(link_frames(q0)[k]).
        """
        return list(self.link_frames_batch(np.asarray(joints_deg, float)[None])[0])

    def link_frames_batch(self, joints_deg) -> np.ndarray:
        """link_frames for every row of (n, 6) joints at once: (n, 6, 4, 4)."""
        p = self.params
        q = np.radians(np.asarray(joints_deg, float)) * p.signs + p.offsets
        n = len(q)
        t1 = _rz(q[:, 0])
        t2 = t1 @ _tr(p.a1, p.b, p.c1, n) @ _ry(q[:, 1])
        t3 = t2 @ _tr(0, 0, p.c2, n) @ _ry(q[:, 2])
        t4 = t3 @ _tr(p.a2, 0, p.c3, n) @ _rz(q[:, 3])  # A4 turns about the forearm, at the wrist centre
        t5 = t4 @ _ry(q[:, 4])
        t6 = t5 @ _rz(q[:, 5])
        return np.stack([t1, t2, t3, t4, t5, t6], axis=1)

    def link_points(self, joints_deg) -> np.ndarray:
        """Key points along the arm (robot frame) for drawing:
        root, A1 top, A2, A3, wrist centre, flange, TCP."""
        p = self.params
        q = np.radians(joints_deg) * p.signs + p.offsets
        rot1 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0], [np.sin(q[0]), np.cos(q[0]), 0], [0, 0, 1]])
        a2 = rot1 @ [p.a1, p.b, p.c1]
        a3 = a2 + rot1 @ [p.c2 * np.sin(q[1]), 0, p.c2 * np.cos(q[1])]
        flange = self.flange(joints_deg)
        wrist = flange[:3, 3] - p.c4 * flange[:3, 0]  # KUKA flange X is the A6 axis
        tcp = (flange @ self.tool)[:3, 3]
        return np.array([[0, 0, 0], [0, 0, p.c1], a2, a3, wrist, flange[:3, 3], tcp])

    def within_limits(self, joints_deg) -> bool:
        q = np.asarray(joints_deg)
        return bool(np.all(q >= self.limits[:, 0]) and np.all(q <= self.limits[:, 1]))


def _rz(t):
    m = np.zeros(t.shape + (4, 4))
    c, s = np.cos(t), np.sin(t)
    m[..., 0, 0], m[..., 0, 1], m[..., 1, 0], m[..., 1, 1] = c, -s, s, c
    m[..., 2, 2] = m[..., 3, 3] = 1.0
    return m


def _ry(t):
    m = np.zeros(t.shape + (4, 4))
    c, s = np.cos(t), np.sin(t)
    m[..., 0, 0], m[..., 0, 2], m[..., 2, 0], m[..., 2, 2] = c, s, -s, c
    m[..., 1, 1] = m[..., 3, 3] = 1.0
    return m


def _tr(x, y, z, n):
    m = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
    m[:, :3, 3] = [x, y, z]
    return m
