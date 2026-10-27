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
"""

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from shapely.geometry import MultiPoint

from coldspray.models import ProcessConfig
from coldspray.toolpath.model import Move, Toolpath, spray_frame

RESAMPLE_MM = 2.0
WEAK_MEAN = 0.3      # |mean normal| below this: no single "view" direction
BRIDGE_DEG = 30.0    # pieces of one cut are joined if their normals agree this well


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


def surface_strokes(vertices, faces, coat_mask, step_over, blend_mm=10.0, max_tilt_deg=60.0):
    """Ordered strokes [(points, normals, closed)] on the surface, zig-zagged,
    plus the number of points skipped for tilting too far from the mean normal."""
    mesh = trimesh.Trimesh(np.asarray(vertices, float), np.asarray(faces)[np.asarray(coat_mask, bool)])
    if len(mesh.faces) == 0:
        return [], 0
    d, c, strong = raster_axes(mesh.vertices, mesh.face_normals, mesh.area_faces)
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


def coat_regions(vertices, faces, coat_mask) -> list[np.ndarray]:
    """The picked triangles split into separate regions, as masks over `faces`.
    Faces that share an edge are one region (strokes run on over the edge);
    faces with a gap between them are planned apart, so the gap isn't sprayed.
    Regions are ordered along BASE X, then Y."""
    faces = np.asarray(faces)
    sel = np.flatnonzero(np.asarray(coat_mask, bool))
    if not len(sel):
        return []
    mesh = trimesh.Trimesh(np.asarray(vertices, float), faces[sel], process=True, validate=False)
    groups = trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(len(sel)), min_len=1)
    out = []
    for g in groups:
        m = np.zeros(len(faces), bool)
        m[sel[np.asarray(g)]] = True
        out.append(m)
    centre = [np.asarray(vertices)[faces[m]].reshape(-1, 3).mean(0) for m in out]
    return [out[i] for i in sorted(range(len(out)), key=lambda i: (round(centre[i][0], 3), centre[i][1]))]


def _extend(p, n, length):
    """Stroke with its ends pushed out tangentially by `length` (normals held)."""
    if length <= 0 or len(p) < 2:
        return p, n
    t0 = (p[0] - p[1]) / np.linalg.norm(p[0] - p[1])
    t1 = (p[-1] - p[-2]) / np.linalg.norm(p[-1] - p[-2])
    return (np.vstack([p[0] + t0 * length, p, p[-1] + t1 * length]),
            np.vstack([n[:1], n, n[-1:]]))


def plan_surface(vertices, faces, coat_mask, proc: ProcessConfig, name="surface",
                 table_axes: dict | None = None, *, coats: int = 1, max_tilt_deg: float = 60.0,
                 blend_mm: float = 10.0) -> Toolpath:
    """Toolpath coating the triangles where `coat_mask` is true (vertices in BASE).
    Separate regions (coat_regions) are rastered one after the other, with a
    jump (lifted by `jump_height_mm`) between them. The number of points dropped
    for tilting more than `max_tilt_deg` from the mean normal is left in
    `path.skipped_points`."""
    strokes, skipped = [], 0
    for r, mask in enumerate(coat_regions(vertices, faces, coat_mask)):
        s, k = surface_strokes(vertices, faces, mask, proc.step_over_mm, blend_mm, max_tilt_deg)
        strokes += [(p, n, c, r) for p, n, c in s]
        skipped += k
    path = Toolpath(name)
    path.skipped_points = skipped * coats
    if not strokes:
        return path

    def pose(p, n, travel):
        if np.linalg.norm(np.cross(travel, n)) < 1e-9:
            travel = np.eye(3)[np.argmin(np.abs(n))]
        return spray_frame(p + n * proc.standoff_mm, -n, travel)

    for coat in range(coats):
        run = strokes if coat % 2 == 0 else [(p[::-1], n[::-1], c, r) for p, n, c, r in strokes[::-1]]
        for k, (p, n, closed, region) in enumerate(run):
            if closed:
                p, n = np.vstack([p, p[:1]]), np.vstack([n, n[:1]])
            else:
                p, n = _extend(p, n, proc.overrun_mm)
            tang = np.gradient(p, axis=0)
            if not path.moves:
                lift = p[0] + n[0] * proc.approach_height_mm
                path.moves.append(Move(pose(lift, n[0], tang[0]), proc.rapid_speed_mm_s, kind="PTP",
                                       axes=dict(table_axes or {}), tag="approach"))
                path.moves.append(Move(pose(p[0], n[0], tang[0]), proc.rapid_speed_mm_s, tag="approach"))
            elif region != prev_region:
                # another region: jump across the gap, lifted by jump_height_mm, as Layers does
                h, travel = proc.jump_height_mm, p[0] - prev_p
                for q, nn in ((prev_p + prev_n * h, prev_n), (p[0] + n[0] * h, n[0]), (p[0], n[0])):
                    path.moves.append(Move(pose(q, nn, travel), proc.rapid_speed_mm_s, gun_on=True, tag="jump"))
            else:
                # lift mid-turn: gun goes a further standoff_mm off the surface along the mean
                # normal, so a turn over a convex part cannot graze it
                nm = prev_n + n[0]
                nm = nm / np.linalg.norm(nm) if np.linalg.norm(nm) > 1e-9 else n[0]
                mid = (prev_p + p[0]) / 2 + nm * proc.standoff_mm
                travel = p[0] - prev_p
                path.moves.append(Move(pose(mid, nm, travel), proc.traverse_speed_mm_s,
                                       gun_on=True, tag="turn"))
                path.moves.append(Move(pose(p[0], n[0], travel),
                                       proc.traverse_speed_mm_s, gun_on=True, tag="turn"))
            for i in range(1, len(p)):
                path.moves.append(Move(pose(p[i], n[i], tang[i]), proc.traverse_speed_mm_s,
                                       gun_on=True, tag="infill"))
            prev_p, prev_n, prev_region = p[-1], n[-1], region
    last = path.moves[-1].pose
    up = last.copy()
    up[:3, 3] = last[:3, 3] - last[:3, 0] * proc.approach_height_mm
    path.moves.append(Move(up, proc.rapid_speed_mm_s, tag="approach"))
    return path
