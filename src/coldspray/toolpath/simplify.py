"""Fewer, better moves: merge straight gun-on LIN runs into one LIN, and fit
circular arcs to curved runs (Move kind "CIRC" with an `aux` pose on the arc).

Only spraying moves are touched (gun on, LIN, not approach / jump / purge),
and a run keeps one speed, one tag and one table position; a dwell ends a run.
A merged move reproduces every point it replaces within `tol_mm`, and the gun
orientation within `tol_deg` of how the robot interpolates it (LIN: slerp
along the line; CIRC: $CIRC_TYPE = #PATH, see model.circ_poses).
"""

import copy
from dataclasses import replace

import numpy as np

from coldspray.toolpath.model import Toolpath, circ_poses, circle, cross, slerp_mats

TOL_MM = 0.05
TOL_DEG = 0.5
MAX_SWEEP = np.pi            # one CIRC turns at most 180 deg
MAX_RADIUS = 2000.0          # mm; flatter than this stays LIN
SKIP_TAGS = {"approach", "jump", "purge"}


def _angles(R_pred, R_act) -> np.ndarray:
    """Angle (deg) between rotation matrices, pairwise."""
    tr = np.einsum("nij,nij->n", R_pred, R_act)
    return np.degrees(np.arccos(np.clip((tr - 1) / 2, -1, 1)))


class _Fit:
    def __init__(self, P, R, tol_mm, tol_deg):
        self.P, self.R, self.tol, self.tol_deg = P, R, tol_mm, tol_deg
        self.pos = P[:, :3, 3]

    def line(self, s, e) -> bool:
        if e == s + 1:
            return True
        p = self.pos[s:e + 1] - self.pos[s]
        d = p[-1]
        L2 = d @ d
        if L2 < 1e-12:
            return False
        t = p @ d / L2
        if np.any(np.diff(t) < -1e-9) or t.min() < -1e-9 or t.max() > 1 + 1e-9:
            return False
        off = p - t[:, None] * d
        if np.einsum("ij,ij->i", off, off).max() > self.tol ** 2:
            return False
        rots = slerp_mats(self.R[s], self.R[e], np.clip(t, 0, 1))
        return _angles(rots, self.R[s:e + 1]).max() <= self.tol_deg

    def arc(self, s, e):
        """The aux index if points s..e lie on one arc, else None."""
        if e < s + 2:
            return None
        mid = (s + e) // 2
        g = circle(self.pos[s], self.pos[mid], self.pos[e])
        if g is None:
            return None
        c, n, r, sweep = g
        if r > MAX_RADIUS or sweep > MAX_SWEEP or sweep < 1e-6:
            return None
        q = self.pos[s:e + 1] - c
        h = q @ n
        inplane = q - h[:, None] * n
        rad = np.linalg.norm(inplane, axis=1) - r
        if (h ** 2 + rad ** 2).max() > self.tol ** 2:
            return None
        u = self.pos[s] - c
        ang = np.arctan2(cross(u, inplane) @ n, inplane @ u) % (2 * np.pi)
        ang[0] = 0.0
        ang[-1] = sweep
        if np.any(np.diff(ang) < -1e-9) or ang.max() > sweep + 1e-9:
            return None
        # The points are LIN corners: the straight segments between them must lie on the
        # arc too (sagitta), or e.g. a rectangle's corners would pass as its circumcircle.
        if (r * (1 - np.cos(np.diff(ang) / 2))).max() > self.tol:
            return None
        poses = circ_poses(self.P[s], self.P[mid], self.P[e], ang / sweep)
        if _angles(poses[:, :3, :3], self.R[s:e + 1]).max() > self.tol_deg:
            return None
        return mid


def _longest(fits, s, j) -> int:
    """Largest e in (s, j] with fits(s, e) (exponential then binary search); s if none."""
    lo, step = s, 1
    while True:
        e = min(s + step, j)
        if not fits(s, e):
            hi = e
            break
        lo = e
        if e == j:
            return lo
        step *= 2
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if fits(s, mid):
            lo = mid
        else:
            hi = mid
    return lo


def simplify(path: Toolpath, tol_mm: float = TOL_MM, tol_deg: float = TOL_DEG) -> Toolpath:
    moves = path.moves
    n = len(moves)
    if n < 3:
        return path
    axes = path.axis_values()
    P = np.array([m.pose for m in moves])
    fit = _Fit(P, P[:, :3, :3], tol_mm, tol_deg)

    def ok(k):
        m = moves[k]
        return m.gun_on and m.kind == "LIN" and m.aux is None and m.tag not in SKIP_TAGS

    def merged(s, e, **kw):
        ax = {}
        for k in range(s + 1, e + 1):
            ax |= moves[k].axes
        return replace(moves[e], axes=ax, **kw)

    out = [moves[0]]
    i = 0
    while i < n - 1:
        if not ok(i + 1) or axes[i + 1] != axes[i]:
            out.append(moves[i + 1])
            i += 1
            continue
        j, first = i + 1, moves[i + 1]
        while (j + 1 < n and ok(j + 1) and not moves[j].dwell_s and axes[j + 1] == axes[i]
               and moves[j + 1].speed == first.speed and moves[j + 1].tag == first.tag):
            j += 1
        s = i
        while s < j:
            e_line = _longest(fit.line, s, j)
            e_arc = _longest(lambda a, b: b == a + 1 or fit.arc(a, b) is not None, s, j)
            if e_arc > e_line and e_arc >= s + 2:
                mid = fit.arc(s, e_arc)
                out.append(merged(s, e_arc, kind="CIRC", aux=P[mid].copy()))
                s = e_arc
            else:
                out.append(merged(s, e_line) if e_line > s + 1 else moves[e_line])
                s = e_line
        i = j
    if len(out) == n:
        return path
    new = copy.copy(path)                       # keeps planner extras (e.g. zigzag_regions)
    new.moves, new.joint_hint = out, None
    return new
