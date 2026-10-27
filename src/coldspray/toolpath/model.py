"""Toolpath: the format shared by planners, the simulator and post-processors.

Poses are 4x4 TCP frames in the work BASE frame (mm): the turntable faceplate,
origin at its top centre. The tool X axis is the spray direction; tool Y
follows the travel direction. A move may also set external axes (turntable
tilt and rotation, degrees); axes it doesn't mention keep their previous value.
"""

from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from coldspray.transforms import frame


@dataclass
class Move:
    pose: np.ndarray
    speed: float                      # mm/s along the part surface (ignored for PTP)
    gun_on: bool = False              # spray state during this move
    kind: Literal["LIN", "PTP"] = "LIN"
    axes: dict[str, float] = field(default_factory=dict)  # e.g. {"E2": 0.0, "E5": 90.0}
    tag: str = ""                     # what the move is for: contour, infill, turn, jump, purge, approach
    dwell_s: float = 0.0              # wait this long at the point after arriving


@dataclass
class Samples:
    poses: np.ndarray                 # (n, 4, 4) in BASE
    gun_on: np.ndarray                # (n,)
    move_index: np.ndarray            # (n,) which move each sample belongs to
    axes: dict[str, np.ndarray]       # external axis values per sample

    def __len__(self):
        return len(self.poses)

    def axes_at(self, i: int) -> dict[str, float]:
        return {k: float(v[i]) for k, v in self.axes.items()}


@dataclass
class Toolpath:
    name: str
    moves: list[Move] = field(default_factory=list)
    skipped_points: int = 0           # points a planner dropped (e.g. surface tilt limit)
    joint_hint: np.ndarray | None = None   # (n_moves, 6) joints to seed IK from (solver; NaN = none)

    def hint_for(self, move_index) -> np.ndarray | None:
        """`joint_hint` per sample (rows by move), or None if there is none or it no longer fits."""
        h = self.joint_hint
        return None if h is None or len(h) != len(self.moves) else h[np.asarray(move_index)]

    def axis_values(self) -> list[dict[str, float]]:
        """External axis values in force at each move (carried forward)."""
        state, out = {}, []
        for m in self.moves:
            state = state | m.axes
            out.append(dict(state))
        return out

    def transformed(self, t: np.ndarray) -> "Toolpath":
        """Copy with every pose replaced by `t @ pose` (everything else kept)."""
        return Toolpath(self.name, [replace(m, pose=t @ m.pose, axes=dict(m.axes)) for m in self.moves],
                        self.skipped_points, self.joint_hint)

    def rolled(self, deg: float) -> "Toolpath":
        """Copy with the gun turned `deg` about its spray axis (tool X) at every pose."""
        r = frame(c=deg)
        return Toolpath(self.name, [replace(m, pose=m.pose @ r, axes=dict(m.axes)) for m in self.moves],
                        self.skipped_points, self.joint_hint)

    def sample(self, step: float = 5.0, angle_step: float = 2.0) -> Samples:
        """Points every ~`step` mm (and every ~`angle_step` degrees of external
        axis motion) along the path, for simulation and checks.

        PTP moves are interpolated in Cartesian space here, which is only an
        approximation of the robot.
        """
        axes_per_move = self.axis_values()
        names = sorted({k for a in axes_per_move for k in a})
        first = {k: axes_per_move[-1].get(k, 0.0) for k in names}  # value before an axis is first set
        values = [{k: a.get(k, first[k]) for k in names} for a in axes_per_move]

        m = len(self.moves)
        P = np.array([mv.pose for mv in self.moves])
        V = np.array([[v[k] for k in names] for v in values]).reshape(m, len(names))
        gun_all = np.array([mv.gun_on for mv in self.moves])
        if m == 1:
            return Samples(P[:1].copy(), gun_all[:1], np.array([0]), {k: V[:1, j] for j, k in enumerate(names)})
        dl = P[1:, :3, 3] - P[:-1, :3, 3]
        dist = np.array([np.linalg.norm(r) for r in dl])        # per-vector norm: same ulp as before at exact step multiples
        turn = np.abs(V[1:] - V[:-1]).max(axis=1) if names else np.zeros(m - 1)
        n = np.maximum(1, np.maximum(np.ceil(dist / step), np.ceil(turn / angle_step))).astype(int)
        seg = np.repeat(np.arange(1, m), n)                      # destination move per sample
        start = np.cumsum(n) - n
        j = np.arange(len(seg)) - np.repeat(start, n)            # 0..n-1 within segment
        t = ((j + 1) / np.repeat(n, n))[:, None]
        q = Rotation.from_matrix(P[:, :3, :3]).as_quat()
        q0, q1 = q[seg - 1], q[seg]
        d = d_raw = np.einsum("ij,ij->i", q0, q1)
        q1 = np.where((d < 0)[:, None], -q1, q1)
        d = np.abs(d).clip(0, 1)
        th = np.arccos(d)
        sth = np.sin(th)
        ok = sth > 1e-9
        tt = t[:, 0]
        w0 = np.where(ok, np.sin((1 - tt) * th) / np.where(ok, sth, 1), 1 - tt)
        w1 = np.where(ok, np.sin(tt * th) / np.where(ok, sth, 1), tt)
        qi = w0[:, None] * q0 + w1[:, None] * q1
        qi /= np.linalg.norm(qi, axis=1)[:, None]
        for i in np.unique(seg[np.abs(d_raw) < 1e-9]):           # 180 deg flips: sign is a coin toss, defer to scipy
            sel = seg == i
            qi[sel] = Slerp([0, 1], Rotation.from_matrix([P[i - 1, :3, :3], P[i, :3, :3]]))(tt[sel]).as_quat()
        out = np.tile(np.eye(4), (len(seg) + 1, 1, 1))
        out[0] = P[0]
        out[1:, :3, :3] = Rotation.from_quat(qi).as_matrix()
        out[1:, :3, 3] = (1 - t) * P[seg - 1, :3, 3] + t * P[seg, :3, 3]
        gun = np.concatenate([gun_all[:1], gun_all[seg]])
        index = np.concatenate([[0], seg])
        ax = {k: np.concatenate([V[:1, c], (1 - tt) * V[seg - 1, c] + tt * V[seg, c]])
              for c, k in enumerate(names)}
        return Samples(out, gun, index, ax)


def spray_frame(point, spray_dir, travel_dir) -> np.ndarray:
    """TCP frame at `point` with tool X along `spray_dir`, Y towards `travel_dir`."""
    x = np.asarray(spray_dir, float)
    x /= np.linalg.norm(x)
    y = np.asarray(travel_dir, float) - np.dot(travel_dir, x) * x
    y /= np.linalg.norm(y)
    t = np.eye(4)
    t[:3, :3] = np.column_stack([x, y, np.cross(x, y)])
    t[:3, 3] = point
    return t
