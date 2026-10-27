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

    def flange(self, joints_deg) -> np.ndarray:
        return opw.forward(self.params, np.radians(joints_deg)) @ self._opw_to_kuka

    def fk(self, joints_deg) -> np.ndarray:
        """TCP pose for joint angles A1..A6 (degrees)."""
        return self.flange(joints_deg) @ self.tool

    def ik(self, tcp_pose, seed_deg=HOME_DEG) -> list[np.ndarray]:
        """Joint solutions within limits, closest to `seed_deg` first.

        Axes with more than ±180° of travel (A6) also get their ±360° variants.
        """
        flange = np.asarray(tcp_pose) @ np.linalg.inv(self.tool)
        target = flange @ np.linalg.inv(self._opw_to_kuka)
        candidates = []
        for sol in opw.inverse(self.params, target):
            q = np.degrees(sol)
            for turn in (-360.0, 0.0, 360.0):
                v = q.copy()
                v[5] += turn
                if self.within_limits(v):
                    candidates.append(v)
        seed = np.asarray(seed_deg, dtype=float)
        candidates.sort(key=lambda v: np.max(np.abs(v - seed)))
        return candidates

    def ik_all(self, tcp_pose) -> list[np.ndarray]:
        """Every in-limit joint solution for a TCP pose (A6 also at +-360 deg), unsorted."""
        flange = np.asarray(tcp_pose) @ np.linalg.inv(self.tool)
        target = flange @ np.linalg.inv(self._opw_to_kuka)
        out = []
        for sol in opw.inverse(self.params, target):
            q = np.degrees(sol)
            for turn in (-360.0, 0.0, 360.0):
                v = q.copy()
                v[5] += turn
                if self.within_limits(v):
                    out.append(v)
        return out

    def ik_path(self, poses, seed_deg=HOME_DEG, hint=None) -> np.ndarray:
        """Joint angles for a sequence of TCP poses, each solved from the
        previous one so the arm configuration stays continuous.
        Unreachable poses give a row of NaN.
        """
        out = np.full((len(poses), 6), np.nan)
        seed = np.asarray(seed_deg, dtype=float)
        for i, pose in enumerate(poses):
            if hint is not None and not np.isnan(hint[i, 0]):
                seed = np.asarray(hint[i], dtype=float)     # solver's chosen configuration
            sols = self.ik(pose, seed)
            if sols:
                out[i] = seed = sols[0]
        return out

    def link_frames(self, joints_deg) -> list[np.ndarray]:
        """Frames of the moving links A1..A6 in the robot root frame.

        Each frame turns with its own joint, so a link's CAD geometry drawn at
        pose q0 moves to pose q by  link_frames(q)[k] @ inv(link_frames(q0)[k]).
        """
        p = self.params
        q = np.radians(joints_deg) * p.signs + p.offsets

        def rz(t):
            m = np.eye(4)
            m[:2, :2] = [[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]
            return m

        def ry(t):
            m = np.eye(4)
            m[[0, 0, 2, 2], [0, 2, 0, 2]] = [np.cos(t), np.sin(t), -np.sin(t), np.cos(t)]
            return m

        def tr(x, y, z):
            m = np.eye(4)
            m[:3, 3] = [x, y, z]
            return m

        t1 = rz(q[0])
        t2 = t1 @ tr(p.a1, p.b, p.c1) @ ry(q[1])
        t3 = t2 @ tr(0, 0, p.c2) @ ry(q[2])
        t4 = t3 @ tr(p.a2, 0, p.c3) @ rz(q[3])  # A4 turns about the forearm, at the wrist centre
        t5 = t4 @ ry(q[4])
        t6 = t5 @ rz(q[5])
        return [t1, t2, t3, t4, t5, t6]

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
