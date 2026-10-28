"""Coat selected faces of a part: a raster that follows the surface.

The selected triangles are cut by parallel planes `step_over_mm` apart; each
cut is chained into a polyline on the surface, resampled every <= 2 mm, and the
gun is held at the standoff along the local normal, spraying back along it.
Normals are face normals blended along each stroke (Gaussian over `blend_mm`
of arc length), so a stroke over an edge turns the gun smoothly instead of
snapping. Strokes zig-zag; open strokes are extended tangentially by
`overrun_mm` so the turnarounds sit off the coated area. Closed cuts (rings
round a cylinder) are sprayed as loops.

Raster direction: strokes run along the longest extent of the selection as
seen along its mean normal. If the normals cancel out (a full cylinder) the
cuts are taken across the longest 3D extent instead, giving rings.

Other infill patterns (`infill_pattern` other than zigzag): a region that faces
one way is flattened along its mean normal, filled like a layer (contour
passes, then the pattern; see patterns.py) and the strokes are dropped back
onto the faces. Strokes more than 1.5 x step-over apart are joined by a lifted
jump, as in Layers. A region that wraps round (a face turned away from its mean
normal, e.g. a whole cylinder) can't be flattened and keeps the zigzag.

Per-face split: when the caller passes CAD face ids (`face_ids`), a region that
wraps round (its faces don't all face its mean normal, e.g. a cylinder side, a
fillet and the end face) is split into its CAD faces. Each face is rastered on
its own, in its own frame, the faces taken nearest-first, with a gun-off move
(lifted by `approach_height_mm`) from one face to the next, so no stroke crosses
a face boundary. A region on a single CAD face, or one that faces one way, is
planned as before.
"""

from typing import NamedTuple

import numpy as np
import shapely
from shapely.geometry import MultiPoint

from coldspray.models import ProcessConfig
from coldspray.planner.patterns import PATTERNS, contour_loops
from coldspray.toolpath.model import Move, Toolpath

RESAMPLE_MM = 2.0
WEAK_MEAN = 0.3      # |mean normal| below this: no single "view" direction
BRIDGE_DEG = 30.0    # pieces of one cut are joined if their normals agree this well
FOLD_COS = 0.05      # pattern fill: every face must face the mean normal at least this much
EDGE_MM = 1e-3       # pattern fill: strokes kept this far inside the region so they land on it
OVERRUN_PATTERNS = ("oneway",)   # pattern strokes that run edge to edge get the overrun


class Stroke(NamedTuple):
    p: np.ndarray            # points on the surface
    n: np.ndarray            # surface normals at them
    closed: bool             # a loop (sprayed back to its start)
    region: int
    tag: str = "infill"
    overrun: bool = True     # extended by overrun_mm at both ends (open strokes)
    far_jumps: bool = False  # starts with a lifted jump when > 1.5 x step-over from the last stroke


def raster_axes(points, normals, areas) -> tuple[np.ndarray, np.ndarray, bool]:
    """(stroke direction, cut-plane normal, mean normal is usable)."""
    mean = (normals * areas[:, None]).sum(0) / areas.sum()
    centred = points - points.mean(0)
    strong = np.linalg.norm(mean) >= WEAK_MEAN
    if strong:
        n = mean / np.linalg.norm(mean)
        e1 = np.cross(n, np.eye(3)[np.argmin(np.abs(n))])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        rect = np.asarray(MultiPoint(np.column_stack([centred @ e1, centred @ e2]))
                          .minimum_rotated_rectangle.exterior.coords)
        sides = np.diff(rect[:3], axis=0)
        u = sides[np.argmax(np.linalg.norm(sides, axis=1))]
        d = u[0] * e1 + u[1] * e2
        d /= np.linalg.norm(d)
        return d, np.cross(n, d), True
    axis = np.linalg.svd(centred, full_matrices=False)[2][0]
    helper = np.eye(3)[np.argmin(np.abs(axis))]
    return np.cross(axis, helper) / np.linalg.norm(np.cross(axis, helper)), axis, False


def chain(segments: np.ndarray, seg_normals: np.ndarray) -> list[tuple[np.ndarray, np.ndarray, bool]]:
    """Join (n, 2, 3) segments sharing endpoints into polylines.
    Returns (points, point normals, closed) per polyline."""
    ends = segments.reshape(-1, 3)
    from scipy.spatial import cKDTree  # lazy: keeps app start-up fast
    tree = cKDTree(ends)
    key = np.arange(len(ends))
    for i, j in sorted(tree.query_pairs(1e-5)):
        key[j] = key[i] if key[i] < key[j] else key[j]
    for i in range(len(key)):                      # resolve chains of merges
        while key[key[i]] != key[i]:
            key[i] = key[key[i]]
    node = key.reshape(-1, 2)
    adj: dict[int, list[int]] = {}
    for s, (a, b) in enumerate(node):
        if a != b:
            adj.setdefault(a, []).append(s)
            adj.setdefault(b, []).append(s)
    used = np.zeros(len(segments), bool)
    out = []
    starts = [v for v, ss in adj.items() if len(ss) == 1] + list(adj)
    for v0 in starts:
        free = [s for s in adj[v0] if not used[s]]
        if not free:
            continue
        v, pts, nrm = v0, [ends[v0]], []
        while True:
            free = [s for s in adj[v] if not used[s]]
            if not free:
                break
            s = free[0]
            used[s] = True
            a, b = node[s]
            v = b if a == v else a
            pts.append(ends[v])
            nrm.append(seg_normals[s])
        if len(pts) < 2:
            continue
        closed = v == v0 and len(pts) > 3
        nrm = np.array(nrm)
        pn = np.vstack([nrm[:1], (nrm[:-1] + nrm[1:]) / 2, nrm[-1:]])
        out.append((np.array(pts), pn, closed))
    return out


def resample(points, normals, step=RESAMPLE_MM) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Points no more than `step` apart (original corners kept), with normals
    interpolated; also returns the arc length of each point."""
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    extra = [np.linspace(s[i], s[i + 1], int(np.ceil((s[i + 1] - s[i]) / step)) + 1)[:-1]
             for i in range(len(s) - 1)]
    t = np.concatenate(extra + [s[-1:]])
    p = np.column_stack([np.interp(t, s, points[:, k]) for k in range(3)])
    n = np.column_stack([np.interp(t, s, normals[:, k]) for k in range(3)])
    return p, n / np.linalg.norm(n, axis=1, keepdims=True), t


def blend(normals, arc, width, closed=False) -> np.ndarray:
    """Gaussian blend of normals along the stroke (sigma = width / 2)."""
    if width <= 0 or len(normals) < 3:
        return normals
    d = arc[:, None] - arc[None, :]
    if closed:
        total = arc[-1]
        d = (d + total / 2) % total - total / 2
    w = np.exp(-0.5 * (d / (width / 2)) ** 2)
    n = w @ normals
    return n / np.linalg.norm(n, axis=1, keepdims=True)


def surface_strokes(vertices, faces, coat_mask, step_over, blend_mm=10.0, max_tilt_deg=60.0, angle_deg=0.0):
    """Ordered strokes [(points, normals, closed)] on the surface, zig-zagged,
    plus the number of points skipped for tilting too far from the mean normal.
    `angle_deg` turns the strokes about the mean normal from the longest side
    (no effect when the faces wrap round: the cuts are rings)."""
    import trimesh  # lazy: keeps app start-up fast
    mesh = trimesh.Trimesh(np.asarray(vertices, float), np.asarray(faces)[np.asarray(coat_mask, bool)])
    if len(mesh.faces) == 0:
        return [], 0
    d, c, strong = raster_axes(mesh.vertices, mesh.face_normals, mesh.area_faces)
    if strong and angle_deg:
        a = np.radians(angle_deg)
        d, c = np.cos(a) * d + np.sin(a) * c, np.cos(a) * c - np.sin(a) * d
    mean = (mesh.face_normals * mesh.area_faces[:, None]).sum(0)
    mean /= np.linalg.norm(mean) if strong else 1.0
    offs = mesh.vertices @ c
    lines, face_idx = [], []
    for o in np.arange(offs.min() + step_over / 2, offs.max(), step_over):
        seg, fi = trimesh.intersections.mesh_plane(mesh, c, c * o, return_faces=True)
        lines.append((seg, mesh.face_normals[fi]))
    strokes, skipped, flip = [], 0, False
    cos_max = np.cos(np.radians(max_tilt_deg))
    for seg, sn in lines:
        if len(seg) == 0:
            continue
        pieces = []
        for pts, nrm, closed in chain(seg, sn):
            p, n, arc = resample(pts, nrm)
            n = blend(n, arc, blend_mm, closed)
            if closed:
                p, n = p[:-1], n[:-1]
                if np.dot(np.cross(n[0], p[1] - p[0]), c) < 0:   # loops all run the same way round
                    p, n = p[::-1], n[::-1]
                pieces.append((p, n, True))
                continue
            if (p[-1] - p[0]) @ d < 0:
                p, n = p[::-1], n[::-1]
            ok = (n @ mean >= cos_max) if strong else np.ones(len(p), bool)
            skipped += int((~ok).sum())
            run = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
            pieces += [(p[a:b], n[a:b], False) for a, b in zip(run[::2], run[1::2]) if b - a >= 2]
        pieces.sort(key=lambda q: q[0][0] @ d)
        joined = []
        for p, n, closed in pieces:          # bridge gaps (holes) between pieces facing the same way
            if (joined and not closed and not joined[-1][2]
                    and joined[-1][1][-1] @ n[0] >= np.cos(np.radians(BRIDGE_DEG))):
                joined[-1] = (np.vstack([joined[-1][0], p]), np.vstack([joined[-1][1], n]), False)
            else:
                joined.append((p, n, closed))
        if flip:
            joined = [(p[::-1], n[::-1], cl) for p, n, cl in joined[::-1]]
        flip = not flip
        strokes += joined
    return strokes, skipped


def pattern_strokes(vertices, faces, coat_mask, proc, index=0, blend_mm=10.0, max_tilt_deg=60.0):
    """Contour passes and `proc.infill_pattern` over a region that faces one way:
    (strokes [(points, normals, closed, tag)], points skipped for tilt), or None when
    the region wraps round and can't be flattened. `index` plays the layer index
    (raster angle, seam position), so successive coats differ as layers do."""
    import trimesh  # lazy: keeps app start-up fast
    mesh = trimesh.Trimesh(np.asarray(vertices, float), np.asarray(faces)[np.asarray(coat_mask, bool)])
    if len(mesh.faces) == 0:
        return [], 0
    d, c, strong = raster_axes(mesh.vertices, mesh.face_normals, mesh.area_faces)
    n = np.cross(d, c)                                   # the mean normal
    if not strong or (mesh.face_normals @ n).min() < FOLD_COS:
        return None
    o = mesh.vertices.mean(0)
    uv = (mesh.vertices - o) @ np.column_stack([d, c])
    region = shapely.union_all(shapely.polygons(uv[mesh.faces]))
    region = region.buffer(1e-4, join_style="mitre").buffer(-1e-4, join_style="mitre")   # seal seams
    flat = [(s, "contour") for s in contour_loops(region, proc)]
    start = flat[0][0][0] if flat else None
    fill = region.buffer(-proc.contour_passes * proc.step_over_mm - EDGE_MM)
    infill = PATTERNS[proc.infill_pattern](fill, proc, index, start=start)
    if proc.infill_pattern in OVERRUN_PATTERNS:          # the overrun is measured from the face edge
        infill = [_to_edge(s, region) for s in infill]
    flat += [(s, "infill") for s in infill]

    rays = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    above = ((mesh.vertices - o) @ n).max() + 1.0
    cos_max = np.cos(np.radians(max_tilt_deg))
    out, skipped = [], 0
    for s, tag in flat:
        closed = len(s) > 3 and np.allclose(s[0], s[-1])
        x, y = s[:, 0], s[:, 1]
        if (abs(x @ np.roll(y, 1) - y @ np.roll(x, 1)) / 2 < proc.step_over_mm ** 2 if np.allclose(s[0], s[-1])
                else np.linalg.norm(np.diff(s, axis=0), axis=1).sum() < proc.step_over_mm):
            continue                         # slivers where an inset closes up (centre of a ring)
        q, _, _ = resample(np.column_stack([s, np.zeros(len(s))]), np.tile([0.0, 0.0, 1.0], (len(s), 1)))
        if closed:
            q = q[:-1]
        origins = o + q[:, :1] * d + q[:, 1:2] * c + n * above
        loc, ray, tri = rays.intersects_location(origins, np.tile(-n, (len(q), 1)), multiple_hits=False)
        p, fn, ok = np.zeros((len(q), 3)), np.tile(n, (len(q), 1)), np.zeros(len(q), bool)
        p[ray], fn[ray], ok[ray] = loc, mesh.face_normals[tri], True
        whole = ok.all()
        runs = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
        for a, b in zip(runs[::2], runs[1::2]):
            arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p[a:b], axis=0), axis=1))])
            nb = blend(fn[a:b], arc, blend_mm, closed and whole)
            good = nb @ n >= cos_max
            skipped += int((~good).sum())
            if closed and whole and good.all():
                out.append((p[a:b], nb, True, tag))
                continue
            kept = np.flatnonzero(np.diff(np.concatenate([[0], good.astype(int), [0]])))
            out += [(p[a:b][i:j], nb[i:j], False, tag) for i, j in zip(kept[::2], kept[1::2]) if j - i >= 2]
    return out, skipped


def _to_edge(s, region):
    """Open flat stroke `s` (k, 2) with each end run on along its direction to the
    region's boundary (EDGE_MM inside), so the overrun is measured from the edge of
    the face, not from the inset the pattern was filled in."""
    if len(s) < 2 or np.allclose(s[0], s[-1]):
        return s
    from shapely.geometry import LineString, Point
    edge = region.boundary
    reach = np.ptp(np.asarray(region.envelope.exterior.coords), axis=0).sum() + 1.0
    ends = []
    for a, b in ((s[1], s[0]), (s[-2], s[-1])):
        t = b - a
        if np.linalg.norm(t) < 1e-12:
            ends.append(None)
            continue
        t = t / np.linalg.norm(t)
        hit = LineString([b, b + t * reach]).intersection(edge)
        d = Point(b).distance(hit) if not hit.is_empty else 0.0
        ends.append(b + t * (d - EDGE_MM) if d > EDGE_MM else None)
    head = [ends[0]] if ends[0] is not None else []
    tail = [ends[1]] if ends[1] is not None else []
    return np.vstack(head + [s] + tail)


def coat_regions(vertices, faces, coat_mask) -> list[np.ndarray]:
    """The picked triangles split into separate regions, as masks over `faces`.
    Faces that share an edge are one region (strokes run on over the edge);
    faces with a gap between them are planned apart, so the gap isn't sprayed.
    Regions are ordered along BASE X, then Y."""
    faces = np.asarray(faces)
    sel = np.flatnonzero(np.asarray(coat_mask, bool))
    if not len(sel):
        return []
    import trimesh  # lazy: keeps app start-up fast
    mesh = trimesh.Trimesh(np.asarray(vertices, float), faces[sel], process=True, validate=False)
    groups = trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(len(sel)), min_len=1)
    out = []
    for g in groups:
        if mesh.area_faces[np.asarray(g)].sum() < 1e-6:
            continue                        # degenerate slivers from the tessellation: nothing to coat
        m = np.zeros(len(faces), bool)
        m[sel[np.asarray(g)]] = True
        out.append(m)
    if not out:
        return []
    centre = [np.asarray(vertices)[faces[m]].reshape(-1, 3).mean(0) for m in out]
    return [out[i] for i in sorted(range(len(out)), key=lambda i: (round(centre[i][0], 3), centre[i][1]))]


def _wraps(vertices, faces, mask) -> bool:
    """The triangles under `mask` don't all face their (area-weighted) mean normal."""
    tri = np.asarray(vertices, float)[np.asarray(faces)[mask]]
    cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = np.linalg.norm(cr, axis=1)
    keep = area > 1e-12
    if not keep.any():
        return False
    nrm = cr[keep] / area[keep, None]
    mean = (nrm * area[keep, None]).sum(0) / area[keep].sum()
    if np.linalg.norm(mean) < WEAK_MEAN:
        return True
    return bool((nrm @ (mean / np.linalg.norm(mean))).min() < FOLD_COS)


def split_faces(vertices, faces, regions, face_ids) -> tuple[list[np.ndarray], list[int]]:
    """`regions` (coat_regions) with each one that wraps round and spans several
    CAD faces (`face_ids`, per triangle) split per face: (masks, the region each
    came from). The faces of a split region are ordered nearest-first (centroids),
    starting from the face first along BASE X, then Y."""
    face_ids = np.asarray(face_ids)
    v, f = np.asarray(vertices, float), np.asarray(faces)
    masks, group = [], []
    for g, m in enumerate(regions):
        ids = np.unique(face_ids[m])
        if len(ids) < 2 or not _wraps(v, f, m):
            masks.append(m)
            group.append(g)
            continue
        parts = [q for i in ids for q in coat_regions(v, f, m & (face_ids == i))]
        centre = [v[f[q]].reshape(-1, 3).mean(0) for q in parts]
        todo = sorted(range(len(parts)), key=lambda i: (round(centre[i][0], 3), centre[i][1]))
        order = [todo.pop(0)]
        while todo:
            k = min(todo, key=lambda i: np.linalg.norm(centre[i] - centre[order[-1]]))
            todo.remove(k)
            order.append(k)
        masks += [parts[i] for i in order]
        group += [g] * len(order)
    return masks, group


def _extend(p, n, length):
    """Stroke with its ends pushed out tangentially by `length` (normals held)."""
    if length <= 0 or len(p) < 2:
        return p, n
    t0 = (p[0] - p[1]) / np.linalg.norm(p[0] - p[1])
    t1 = (p[-1] - p[-2]) / np.linalg.norm(p[-1] - p[-2])
    return (np.vstack([p[0] + t0 * length, p, p[-1] + t1 * length]),
            np.vstack([n[:1], n, n[-1:]]))


def _cross(a, b) -> np.ndarray:
    """np.cross of two 3-vectors, same arithmetic (so bit-identical) without its overhead."""
    a0, a1, a2 = (float(v) for v in a)
    b0, b1, b2 = (float(v) for v in b)
    return np.array([a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0])


def _spray_frame(point, spray_dir, travel_dir) -> np.ndarray:
    """toolpath.model.spray_frame with the fast `_cross` (same result)."""
    x = np.asarray(spray_dir, float)
    x /= np.linalg.norm(x)
    y = np.asarray(travel_dir, float) - np.dot(travel_dir, x) * x
    y /= np.linalg.norm(y)
    t = np.eye(4)
    t[:3, 0], t[:3, 1], t[:3, 2] = x, y, _cross(x, y)
    t[:3, 3] = point
    return t


def plan_surface(vertices, faces, coat_mask, proc: ProcessConfig, name="surface",
                 table_axes: dict | None = None, *, coats: int = 1, max_tilt_deg: float = 60.0,
                 blend_mm: float = 10.0, face_ids=None) -> Toolpath:
    """Toolpath coating the triangles where `coat_mask` is true (vertices in BASE).
    Separate regions (coat_regions) are rastered one after the other, with a
    jump (lifted by `jump_height_mm`) between them. Each region is filled with
    `proc.infill_pattern` where it can be flattened, else zigzagged (module doc).
    The number of points dropped for tilting more than `max_tilt_deg` from the
    mean normal is left in `path.skipped_points`; `path.zigzag_regions` counts
    regions that had to fall back to the zigzag. With `face_ids` (CAD face per
    triangle) a region that wraps round is split per face (split_faces); moves
    between faces of one region are gun-off, lifted by `approach_height_mm`."""
    regions = coat_regions(vertices, faces, coat_mask)
    group = list(range(len(regions)))
    if face_ids is not None:
        regions, group = split_faces(vertices, faces, regions, face_ids)
    pattern = proc.infill_pattern
    cut: dict[int, list] = {}                # region -> plane-cut strokes, planned once
    strokes, skipped, fallback = [], 0, set()
    for coat in range(coats):
        # odd coats run the regions, and the zigzag strokes, backwards
        order = range(len(regions)) if coat % 2 == 0 else range(len(regions) - 1, -1, -1)
        for r in order:
            filled = (pattern_strokes(vertices, faces, regions[r], proc, coat, blend_mm, max_tilt_deg)
                      if pattern != "zigzag" else None)
            if filled is not None:
                s, k = filled
                skipped += k
                strokes += [Stroke(p, n, c, r, tag, pattern in OVERRUN_PATTERNS and tag == "infill", True)
                            for p, n, c, tag in s]
                continue
            if pattern != "zigzag":
                fallback.add(r)
            if r not in cut:
                cut[r] = surface_strokes(vertices, faces, regions[r], proc.step_over_mm, blend_mm, max_tilt_deg,
                                         proc.raster_start_deg)
            s, k = cut[r]
            skipped += k
            if coat % 2:
                s = [(p[::-1], n[::-1], c) for p, n, c in s[::-1]]
            strokes += [Stroke(p, n, c, r) for p, n, c in s]
    path = Toolpath(name)
    path.skipped_points = skipped
    path.zigzag_regions = len(fallback)
    if not strokes:
        return path

    def pose(p, n, travel):
        if np.linalg.norm(_cross(travel, n)) < 1e-9:
            travel = np.eye(3)[np.argmin(np.abs(n))]
        return _spray_frame(p + n * proc.standoff_mm, -n, travel)

    for p, n, closed, region, tag, overrun, far_jumps in strokes:
        if closed:
            p, n = np.vstack([p, p[:1]]), np.vstack([n, n[:1]])
        elif overrun:
            p, n = _extend(p, n, proc.overrun_mm)
        tang = np.gradient(p, axis=0)
        if not path.moves:
            lift = p[0] + n[0] * proc.approach_height_mm
            path.moves.append(Move(pose(lift, n[0], tang[0]), proc.rapid_speed_mm_s, kind="PTP",
                                   axes=dict(table_axes or {}), tag="approach"))
            path.moves.append(Move(pose(p[0], n[0], tang[0]), proc.rapid_speed_mm_s, tag="approach"))
        elif region != prev_region and group[region] == group[prev_region]:
            # next CAD face of a region that wraps round: gun off, lifted off one face and onto the next
            h, travel = proc.approach_height_mm, p[0] - prev_p
            for q, nn in ((prev_p + prev_n * h, prev_n), (p[0] + n[0] * h, n[0]), (p[0], n[0])):
                path.moves.append(Move(pose(q, nn, travel), proc.rapid_speed_mm_s, gun_on=False, tag="jump"))
        elif region != prev_region or (far_jumps and np.linalg.norm(p[0] - prev_p) > 1.5 * proc.step_over_mm):
            # another region, or across a hole: jump, lifted by jump_height_mm, as Layers does
            h, travel = proc.jump_height_mm, p[0] - prev_p
            for q, nn in ((prev_p + prev_n * h, prev_n), (p[0] + n[0] * h, n[0]), (p[0], n[0])):
                path.moves.append(Move(pose(q, nn, travel), proc.rapid_speed_mm_s, gun_on=True, tag="jump"))
        elif not (far_jumps and np.allclose(p[0], prev_p)):
            # turn to the next stroke, lifted mid-turn by jump_height_mm along the mean
            # normal (0: straight across at spray height; the solver lifts any that collide)
            travel = p[0] - prev_p
            if proc.jump_height_mm > 0:
                nm = prev_n + n[0]
                nm = nm / np.linalg.norm(nm) if np.linalg.norm(nm) > 1e-9 else n[0]
                mid = (prev_p + p[0]) / 2 + nm * proc.jump_height_mm
                path.moves.append(Move(pose(mid, nm, travel), proc.traverse_speed_mm_s,
                                       gun_on=True, tag="turn"))
            path.moves.append(Move(pose(p[0], n[0], travel),
                                   proc.traverse_speed_mm_s, gun_on=True, tag="turn"))
        for i in range(1, len(p)):
            path.moves.append(Move(pose(p[i], n[i], tang[i]), proc.traverse_speed_mm_s,
                                   gun_on=True, tag=tag))
        prev_p, prev_n, prev_region = p[-1], n[-1], region
    last = path.moves[-1].pose
    up = last.copy()
    up[:3, 3] = last[:3, 3] - last[:3, 0] * proc.approach_height_mm
    path.moves.append(Move(up, proc.rapid_speed_mm_s, tag="approach"))
    return path
