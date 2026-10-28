"""Warm-up / powder purge point: a fixed spot in free space where the gun
switches on and dwells until the powder flow settles, before it travels to
the part with the spray still on. It comes back at the end (and every N
layers) and switches off there, so the spray never starts or stops over the
part.

The point is fixed in the cell (WORLD) while the toolpath is in BASE, which
moves with the turntable, so each purge pose is converted with the table axes
in force at that point of the path.
"""

from dataclasses import replace

import numpy as np

from coldspray.cell import base_worlds, robroot
from coldspray.toolpath.model import Move, Toolpath, spray_frame


def purge_world(purge, back_mm: float = 0.0) -> np.ndarray:
    """TCP frame in WORLD at the purge point (or `back_mm` back along the spray)."""
    d = np.asarray(purge.spray_dir, float)
    d /= np.linalg.norm(d)
    side = (0.0, 0.0, 1.0) if abs(d[2]) < 0.9 else (1.0, 0.0, 0.0)
    return spray_frame(np.asarray(purge.point, float) - back_mm * d, d, side)


def layer_ends(path: Toolpath) -> list[int]:
    """Index of the retract that ends each layer: the first `approach` move
    after spraying that is followed by the next layer's approach, or ends the
    path (layers.py tags). A lone lift between strokes is not a layer end, and
    Surface coats are not layers (joined by turns). Paths without tags have none."""
    m = path.moves
    return [i for i in range(1, len(m))
            if m[i].tag == "approach" and m[i - 1].tag not in ("approach", "purge")
            and (i + 1 == len(m) or m[i + 1].tag == "approach")
            and any(x.gun_on for x in m[:i])]


CLEAR_MARGIN_MM = 50.0     # via-point height above the part's bounding box


def _hits_box(a, b, lo, hi) -> bool:
    """Does the segment a-b cross the box lo..hi (slab test)?"""
    d = b - a
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-9:
            if a[k] < lo[k] or a[k] > hi[k]:
                return False
            continue
        u, v = sorted(((lo[k] - a[k]) / d[k], (hi[k] - a[k]) / d[k]))
        t0, t1 = max(t0, u), min(t1, v)
        if t0 > t1:
            return False
    return True


def _clearance_vias(path: Toolpath, start_pose, from_pose, speed):
    """BASE moves that rise over the part when the straight way from the purge pose to the
    path start would cross its bounding box (sprayed points + margin): up above the box, across,
    then the path's own approach comes down over the start. Empty when the line is clear."""
    pts = np.array([m.pose[:3, 3] for m in path.moves if m.gun_on])
    if not len(pts):
        return []
    lo, hi = pts.min(0) - CLEAR_MARGIN_MM, pts.max(0) + CLEAR_MARGIN_MM
    a, b = from_pose[:3, 3], start_pose[:3, 3]
    if not _hits_box(a, b, lo, hi):
        return []
    out = []
    for src, base in ((a, from_pose), (b, start_pose)):
        pose = base.copy()
        pose[:3, 3] = (src[0], src[1], max(src[2], hi[2]))
        if abs(pose[2, 3] - src[2]) > 1e-6:
            out.append(Move(pose, speed, gun_on=True, tag="purge"))
    return out


def add_purge(path: Toolpath, cell, speed: float | None = None) -> Toolpath:
    """Copy of `path` with purge visits added (see the module docstring).
    `cell.purge` (if any, and enabled) gives point, spray_dir, dwell_s,
    at_start, at_end, every_n_layers, approach_mm. All moves from the first
    purge visit to the last keep the spray on."""
    purge = getattr(cell, "purge", None) if cell is not None else None
    if purge is None or not purge.enabled or not path.moves:
        return path
    axes = path.axis_values()
    speed = speed or max(m.speed for m in path.moves)
    at, back = purge_world(purge), purge_world(purge, purge.approach_mm)

    from_world = np.linalg.inv(base_worlds(cell, axes))           # WORLD -> BASE at each move's table

    def to_base(world, i):
        return from_world[i] @ world

    def visit(i, dwell):
        return Move(to_base(at, i), speed, gun_on=True, tag="purge", dwell_s=dwell)

    moves = [replace(m, axes=dict(m.axes)) for m in path.moves]
    out, span = [], []
    if purge.at_start:
        out += [Move(to_base(back, 0), speed, kind="PTP", axes=dict(axes[0]), tag="purge"),
                visit(0, purge.dwell_s)]
        span.append(len(out) - 1)
        out += _clearance_vias(path, moves[0].pose, out[-1].pose, speed)
    ends = set(layer_ends(path)[:-1]) if purge.every_n_layers else set()
    n = 0
    for i, m in enumerate(moves):
        out.append(m)
        if i in ends:
            n += 1
            if n % purge.every_n_layers == 0:
                out.append(visit(i, purge.dwell_s))
                span.append(len(out) - 1)
    if purge.at_end:
        out.append(visit(len(moves) - 1, 0.0))
        span.append(len(out) - 1)
    if span:
        first, last = min(span), max(span)
        if not purge.at_end:
            last = max(last, max((k for k, m in enumerate(out) if m.gun_on), default=last))
        if not purge.at_start:
            first = min(first, min((k for k, m in enumerate(out) if m.gun_on), default=first))
        for k in range(first, last + 1):
            out[k].gun_on = True
            if out[k].kind == "PTP":
                out[k].kind = "LIN"             # spray on: keep the gun on a straight line
    return Toolpath(path.name, out, path.skipped_points)


BACK_DIR = (0.0, 1.0, 0.0)          # WORLD direction of the back of the cell (filtration unit)


def _unit_mesh(unit):
    import trimesh
    tri = unit.triangulate()
    return trimesh.Trimesh(np.asarray(tri.points, float), tri.faces.reshape(-1, 4)[:, 1:], process=False)


def suggest_purge(cell, meshes, robot, e1: float = 0.0, standoff: float = 150.0,
                  clearance: float = 100.0, step: float = 50.0, max_dist: float = 1500.0):
    """Purge (point, spray_dir) in WORLD: the spray goes exactly along the cell's back
    direction (`cell.back_dir`, default +Y) into the "Filtration unit". Candidate X/Z lines
    start in the unit's footprint closest to the robot root and fan out in X and Z; along
    each, a ray along spray_dir must hit the unit MESH, and the point starts
    `standoff + clearance` before the hit and steps back (-spray_dir) until it is at least
    `standoff` from the mesh and the gun reaches it (gantry at `e1`), at most `max_dist`
    from the hit. Raises ValueError without a filtration unit or if nothing qualifies."""
    from coldspray.sim.markers import marker_pose

    unit = (getattr(meshes, "groups", None) or {}).get("Filtration unit")
    if unit is None or unit.n_points == 0:
        raise ValueError("No filtration unit in the cell CAD")
    tm = _unit_mesh(unit)
    lo, hi = np.array(unit.bounds[0::2], float), np.array(unit.bounds[1::2], float)
    root_t = robroot(cell, e1)
    root = root_t[:3, 3]
    d = np.asarray(getattr(cell, "back_dir", None) or BACK_DIR, float)
    d /= np.linalg.norm(d)
    a = int(np.argmax(np.abs(d)))                                   # spray axis
    u, v = [k for k in range(3) if k != a]
    inv = np.linalg.inv(root_t)

    def cands(k):
        c = float(np.clip(root[k], lo[k] + 50, hi[k] - 50))
        vals = np.arange(lo[k] + 50, hi[k] - 49, 100.0)
        return sorted(np.append(vals, c), key=lambda x: abs(x - c))

    lines = sorted(((x, y) for x in cands(u) for y in cands(v)),
                   key=lambda p: np.hypot(p[0] - root[u], p[1] - root[v]))
    for cu, cv in lines:
        o = np.zeros(3)
        o[u], o[v] = cu, cv
        o[a] = (lo[a] if d[a] > 0 else hi[a]) - d[a] * (max_dist + 1000.0)
        hits, _, _ = tm.ray.intersects_location([o], [d])
        if not len(hits):
            continue
        hit = hits[np.argmin((hits - o) @ d)]
        t = standoff + clearance
        while t <= max_dist:
            point = hit - t * d
            if (robot.ik(inv @ marker_pose(tuple(point), tuple(d)), cell.home_deg)
                    and trimesh_dist(tm, point) >= standoff):
                return tuple(float(x) for x in point), tuple(float(x) for x in d)
            t += step
    raise ValueError("No reachable purge point in front of the filtration unit")


def trimesh_dist(tm, point) -> float:
    """Unsigned distance from `point` to the mesh surface."""
    import trimesh
    return float(trimesh.proximity.closest_point(tm, [point])[1][0])
