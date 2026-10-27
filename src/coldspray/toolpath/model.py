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
    kind: Literal["LIN", "PTP", "CIRC"] = "LIN"
    axes: dict[str, float] = field(default_factory=dict)  # e.g. {"E2": 0.0, "E5": 90.0}
    tag: str = ""                     # what the move is for: contour, infill, turn, jump, purge, approach
    dwell_s: float = 0.0              # wait this long at the point after arriving
    aux: np.ndarray | None = None     # CIRC: a pose on the arc between the previous move and this one


def cross(a, b) -> np.ndarray:
    """np.cross for 3-vectors (single or stacked (n, 3) with a single one), without its overhead."""
    a, b = np.asarray(a), np.asarray(b)
    return np.stack([a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1], a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2],
                     a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]], axis=-1)


def circle(p0, p1, p2):
    """Circle through three points: (centre, unit normal, radius, sweep rad from p0 via p1 to p2),
    or None if they are (nearly) collinear."""
    p0, p1, p2 = (np.asarray(p, float)[:3] for p in (p0, p1, p2))
    a, b = p1 - p0, p2 - p0
    n = cross(a, b)
    nn = n @ n
    if nn < 1e-12:
        return None
    c = p0 + (cross(n, a) * (b @ b) + cross(b, n) * (a @ a)) / (2 * nn)
    n = n / np.sqrt(nn)
    u, v = p0 - c, p2 - c
    sweep = np.arctan2(cross(u, v) @ n, u @ v) % (2 * np.pi)
    return c, n, float(np.linalg.norm(u)), float(sweep)


def exp_rot(v) -> np.ndarray:
    """Rotation matrices (n, 3, 3) from rotation vectors (n, 3) (Rodrigues)."""
    v = np.atleast_2d(np.asarray(v, float))
    th = np.linalg.norm(v, axis=1)
    k = v / np.where(th > 1e-12, th, 1.0)[:, None]
    K = np.zeros((len(v), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -k[:, 2], k[:, 1], -k[:, 0]
    K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = k[:, 2], -k[:, 1], k[:, 0]
    s, c = np.sin(th)[:, None, None], np.cos(th)[:, None, None]
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def log_rot(r) -> np.ndarray:
    """Rotation vector of one rotation matrix."""
    th = np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))
    if th < 1e-9:
        return np.zeros(3)
    if th > np.pi - 1e-3:                       # axis from the skew part is ill-conditioned
        return Rotation.from_matrix(r).as_rotvec()
    return th / (2 * np.sin(th)) * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def slerp_mats(r0, r1, t) -> np.ndarray:
    """Rotation matrices slerped from r0 to r1 at fractions t."""
    w = log_rot(r0.T @ r1)
    return r0 @ exp_rot(np.outer(np.atleast_1d(t), w))


def circ_poses(start, aux, end, t) -> np.ndarray:
    """Poses at arc fractions `t` of a CIRC from `start` via `aux` to `end` (4x4 each).
    Position on the circle; orientation as KUKA $CIRC_TYPE = #PATH: turned with the
    arc, plus the remaining change slerped."""
    t = np.atleast_1d(np.asarray(t, float))
    c, n, r, sweep = circle(start[:3, 3], aux[:3, 3], end[:3, 3])
    rot = exp_rot(np.outer(t * sweep, n))
    out = np.tile(np.eye(4), (len(t), 1, 1))
    out[:, :3, 3] = rot @ (start[:3, 3] - c) + c
    r1 = exp_rot(-sweep * n)[0] @ end[:3, :3]
    out[:, :3, :3] = rot @ slerp_mats(start[:3, :3], r1, t)
    return out


def move_length(prev: np.ndarray, m: "Move") -> float:
    """Travel of move `m` from pose `prev` (mm): arc length for CIRC, else straight."""
    if m.kind == "CIRC" and m.aux is not None:
        g = circle(prev[:3, 3], m.aux[:3, 3], m.pose[:3, 3])
        if g is not None:
            return g[2] * g[3]
    return float(np.linalg.norm(m.pose[:3, 3] - prev[:3, 3]))


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
    zigzag_regions: int = 0           # Surface regions zigzagged because no other pattern fits them

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
        return Toolpath(self.name, [replace(m, pose=t @ m.pose, axes=dict(m.axes),
                                            aux=None if m.aux is None else t @ m.aux) for m in self.moves],
                        self.skipped_points, self.joint_hint)

    def rolled(self, deg: float) -> "Toolpath":
        """Copy with the gun turned `deg` about its spray axis (tool X) at every pose."""
        r = frame(c=deg)
        return Toolpath(self.name, [replace(m, pose=m.pose @ r, axes=dict(m.axes),
                                            aux=None if m.aux is None else m.aux @ r) for m in self.moves],
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
        circs = [i for i in range(1, m) if self.moves[i].kind == "CIRC" and self.moves[i].aux is not None]
        for i in circs:
            dist[i - 1] = move_length(P[i - 1], self.moves[i])
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
        for i in circs:                                          # along the arc
            sel = np.flatnonzero(seg == i)
            out[1 + sel] = circ_poses(P[i - 1], self.moves[i].aux, P[i], tt[sel])
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
