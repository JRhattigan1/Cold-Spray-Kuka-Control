"""Predicted coating thickness and process estimates (cycle time, powder use).

Deposit model
-------------
While the spray is on, the gun lays down material at a rate with a Gaussian
footprint about its spray axis (tool X):

    rate(r) = A * exp(-r^2 / (2 sigma^2))        [mm of coating per second]

r is the distance of a surface point from the spray axis. track_width_mm is
taken as the footprint's FWHM, so sigma = track_width / (2 sqrt(2 ln 2))
(= track_width / 2.355).

Calibration: one pass at speed V deposits a ridge of peak height
A sqrt(2 pi) sigma / V; passes `step_over_mm` apart sum to a flat layer
A 2 pi sigma^2 / (V s). Setting that to layer_thickness_mm at the process
traverse_speed_mm_s and step_over_mm gives A. Slower moves, closer passes and
dwells therefore build thicker, exactly in proportion.

On a surface tilted to the spray the thickness is scaled by cos(angle between
the spray and the surface normal) (the same mass spread over a larger
footprint); faces turned away get nothing. Not modelled: shadowing by other
geometry, standoff changes (beyond the search radius), deposition efficiency
versus angle, and build-up changing the surface.

The part is refined so no triangle edge is longer than sigma and thickness is
computed per triangle (at its centroid), so area-weighted statistics are exact
sums over the triangles.
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from coldspray.toolpath.model import move_length

FWHM_TO_SIGMA = 1.0 / (2.0 * np.sqrt(2.0 * np.log(2.0)))
CHUNK = 2000                # samples per neighbour query (bounds memory)
MAX_TRIANGLES = 400_000     # refinement cap


def sigma_mm(proc) -> float:
    """Gaussian sigma of the track: track_width_mm is the FWHM."""
    return proc.track_width_mm * FWHM_TO_SIGMA


def peak_rate(proc) -> float:
    """A (mm/s at the spray axis) so one layer of passes gives layer_thickness_mm."""
    s = sigma_mm(proc)
    return proc.layer_thickness_mm * proc.traverse_speed_mm_s * proc.step_over_mm / (2 * np.pi * s * s)


# --- mesh refinement ------------------------------------------------------------
def refine(vertices, faces, max_edge: float, max_triangles: int = MAX_TRIANGLES):
    """Split triangles at the midpoint of their longest edge until every edge is
    <= max_edge (or the triangle cap is hit). Midpoints are shared between
    triangles split on the same edge; elsewhere T-junctions are left (fine for
    colouring). Returns (vertices, faces)."""
    v = np.asarray(vertices, float)
    f = np.asarray(faces, np.int64).reshape(-1, 3)
    while True:
        p = v[f]                                                   # (n, 3, 3)
        e = np.linalg.norm(p[:, [1, 2, 0]] - p, axis=2)            # edge k: corner k -> k+1
        k = e.argmax(axis=1)
        split = e[np.arange(len(f)), k] > max_edge
        if not split.any() or len(f) + split.sum() > max_triangles:
            return v, f
        fs = f[split]
        ks = k[split]
        rows = np.arange(len(fs))
        a = fs[rows, ks]
        b = fs[rows, (ks + 1) % 3]
        c = fs[rows, (ks + 2) % 3]
        key = np.sort(np.stack([a, b], 1), axis=1)
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        mid = len(v) + inv
        v = np.vstack([v, 0.5 * (v[uniq[:, 0]] + v[uniq[:, 1]])])
        f = np.vstack([f[~split], np.stack([a, mid, c], 1), np.stack([mid, b, c], 1)])


# --- spray samples --------------------------------------------------------------
def spray_samples(path, proc, step: float | None = None):
    """(positions (n,3), spray dirs (n,3), seconds (n,)) of the gun while spraying.

    The path is sampled every ~sigma/2; each sample carries the time spent
    reaching it (distance / move speed), plus spray_settle_s where the spray
    switches on (the gun waits at the previous point) and dwell_s of moves that
    end with the spray on."""
    step = step or 0.5 * sigma_mm(proc)
    s = path.sample(step, angle_step=2.0)
    pts = s.poses[:, :3, 3]
    dirs = s.poses[:, :3, 0]
    idx = np.asarray(s.move_index)
    speed = np.array([m.speed for m in path.moves], float)
    on_m = np.array([m.gun_on for m in path.moves], bool)
    dwell = np.array([m.dwell_s for m in path.moves], float)
    dt = np.zeros(len(s))
    if len(s) > 1:
        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        sp = speed[idx[1:]]
        dt[1:] = np.divide(d, sp, out=np.zeros_like(d), where=sp > 0)
    on = on_m[idx].copy()
    # dwells at the end of a move: the move's last sample
    last = np.flatnonzero(np.r_[idx[1:] != idx[:-1], True])
    dt[last] += np.where(on_m[idx[last]], dwell[idx[last]], 0.0)
    # settle where the spray switches on: the gun waits at the previous sample
    first = np.flatnonzero(np.r_[False, idx[1:] != idx[:-1]])
    sw = first[on_m[idx[first]] & ~on_m[idx[first - 1]]]
    keep = on & (dt > 0)
    settle = np.full(len(sw), float(proc.spray_settle_s))
    return (np.vstack([pts[keep], pts[sw - 1]]), np.vstack([dirs[keep], dirs[sw - 1]]),
            np.concatenate([dt[keep], settle]))


# --- thickness ------------------------------------------------------------------
@dataclass
class Deposit:
    vertices: np.ndarray      # refined mesh (BASE mm)
    faces: np.ndarray
    thickness: np.ndarray     # per triangle, mm
    area: np.ndarray          # per triangle, mm^2

    def stats(self, floor: float = 0.1) -> dict:
        """Area-weighted min / mean / max over the sprayed area (triangles above
        `floor` x the maximum) and the % of that area below 80% of the mean."""
        t, a = self.thickness, self.area
        top = float(t.max()) if len(t) else 0.0
        if top <= 0:
            return {"min": 0.0, "mean": 0.0, "max": 0.0, "below80_pct": 0.0, "area_mm2": 0.0}
        m = t > floor * top
        tm, am = t[m], a[m]
        mean = float((tm * am).sum() / am.sum())
        return {"min": float(tm.min()), "mean": mean, "max": top,
                "below80_pct": float(100 * am[tm < 0.8 * mean].sum() / am.sum()),
                "area_mm2": float(am.sum())}


def thickness(vertices, faces, path, proc, refine_to: float | None = None, cosine: bool = True) -> Deposit:
    """Predicted coating thickness (mm) on the mesh (BASE coordinates, the same
    frame as the toolpath poses) after running `path`. See the module notes."""
    sig = sigma_mm(proc)
    v, f = refine(vertices, faces, refine_to or sig)
    p = v[f]
    cen = p.mean(axis=1)
    n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    area2 = np.linalg.norm(n, axis=1)
    n = n / np.where(area2 > 0, area2, 1.0)[:, None]
    out = np.zeros(len(f))
    pos, dirs, dt = spray_samples(path, proc)
    if len(dt) and len(f):
        dirs = dirs / np.linalg.norm(dirs, axis=1)[:, None]
        aim = pos + dirs * proc.standoff_mm
        radius = 3.0 * sig + 0.25 * proc.standoff_mm             # standoff may differ a little
        tree = cKDTree(cen)
        amp = peak_rate(proc)
        for c0 in range(0, len(dt), CHUNK):
            sl = slice(c0, c0 + CHUNK)
            hits = tree.query_ball_point(aim[sl], radius)
            counts = np.fromiter((len(h) for h in hits), int, len(hits))
            if not counts.sum():
                continue
            j = np.concatenate([np.asarray(h, int) for h in hits if h])
            i = np.repeat(np.arange(len(hits)), counts) + c0
            rel = cen[j] - pos[i]
            ax = np.einsum("ij,ij->i", rel, dirs[i])
            r2 = np.einsum("ij,ij->i", rel, rel) - ax * ax
            w = amp * dt[i] * np.exp(-r2 / (2 * sig * sig))
            if cosine:
                w *= np.clip(-np.einsum("ij,ij->i", n[j], dirs[i]), 0.0, None)
            w[ax <= 0] = 0.0
            out += np.bincount(j, w, minlength=len(f))
    return Deposit(v, f, out, 0.5 * area2)


# --- estimates ------------------------------------------------------------------
@dataclass
class Estimate:
    cycle_s: float            # whole program
    spray_s: float            # spray on (travel, settle and dwells)
    powder_g: float           # spray_s x powder_feed_g_min

    def text(self) -> str:
        return (f"Cycle ≈ {self.cycle_s / 60:.1f} min · spray on {self.spray_s / 60:.1f} min · "
                f"powder ≈ {self.powder_g:.0f} g")


def estimate(path, proc) -> Estimate:
    """Cycle time (same model as gui.steps.toolpath.estimate_time_s: each move at
    its speed, spray_settle_s where the spray switches on, plus dwells) and
    powder use (spray-on time x powder_feed_g_min; all powder fed, no
    deposition efficiency)."""
    moves = path.moves
    total = spray = 0.0
    if moves:
        total = sum(m.dwell_s for m in moves)
        spray = moves[0].dwell_s if moves[0].gun_on else 0.0
    on = False
    for a, b in zip(moves, moves[1:]):
        dt = move_length(a.pose, b) / b.speed
        settle = proc.spray_settle_s if b.gun_on and not on else 0.0
        total += dt + settle
        if b.gun_on:
            spray += dt + settle + b.dwell_s
        on = b.gun_on
    feed = getattr(proc, "powder_feed_g_min", 0.0)
    return Estimate(total, spray, spray / 60.0 * feed)
