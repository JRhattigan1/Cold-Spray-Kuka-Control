"""KUKA robot model: OPW kinematics wrapped in KUKA conventions.

Joints are A1..A6 in degrees. Poses are 4x4 frames in mm, in the robot base
frame. The flange frame follows KUKA ($TOOL = NULLFRAME): X out of the flange,
identity orientation at A1..A6 = (0, -90, 90, 0, 0, 0).
"""

import numpy as np

from coldspray.kinematics import opw
from coldspray.models import RobotConfig, ToolConfig
from coldspray.transforms import frame

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
        self.tool = frame(**tool.tcp.model_dump()) if tool else np.eye(4)

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

    def within_limits(self, joints_deg) -> bool:
        q = np.asarray(joints_deg)
        return bool(np.all(q >= self.limits[:, 0]) and np.all(q <= self.limits[:, 1]))
