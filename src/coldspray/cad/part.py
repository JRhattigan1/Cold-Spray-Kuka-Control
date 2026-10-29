"""A part mesh with its CAD faces, placed on the turntable faceplate.

`Part.placement` maps part-CAD coordinates to the work BASE frame (faceplate
top centre, z = 0 on the faceplate surface). A part may be several bodies (an
assembly from one STEP file): one is the build body that is sliced and
sprayed; the others (the component being repaired, fixtures) only take up
space. Everything stands on the "floor": the top of an optional substrate
plate, or the faceplate itself. The base face, if one is picked, points down
(-Z) and the assembly's lowest point sits on the floor. The start face, if one
is picked, is a flat face of the build body that the layers grow away from
(e.g. the underside of a repair patch, against the component).

The KRL base, if one is picked, is a frame on the part (part-CAD coords) that
the program's points are written relative to, so it can be taught on the real
part: +Z out of a flat face, +X along one of its straight edges, the origin at
that edge's end (a corner). See `krl_base_frame`; or from three flat faces
(`krl_base_from_planes`) or a corner vertex (`krl_base_at_vertex`), then turned
about its Z (`spin_frame`) or flipped (`flip_z`). A base not made on one face
has `krl_base_face` None.
"""

from dataclasses import dataclass, field, replace

import numpy as np

from coldspray.transforms import frame

DOWN = np.array([0.0, 0.0, -1.0])


@dataclass
class PlanarFace:
    point: np.ndarray    # a point on the face plane, part-CAD coords
    normal: np.ndarray   # outward unit normal, part-CAD coords
    area: float          # mm^2


@dataclass
class CylFace:
    point: np.ndarray    # a point on the cylinder axis, part-CAD coords
    axis: np.ndarray     # unit axis direction, part-CAD coords
    radius: float        # mm
    area: float          # mm^2


@dataclass
class Part:
    name: str
    vertices: np.ndarray      # (n, 3) part-CAD coords
    faces: np.ndarray         # (m, 3) triangles
    face_ids: np.ndarray      # (m,) B-rep face index per triangle
    planar: dict[int, PlanarFace]
    placement: np.ndarray = field(default_factory=lambda: np.eye(4))   # part-CAD -> BASE
    base_face: int | None = None          # flat face kept pointing down onto the floor
    body_ids: np.ndarray | None = None    # (m,) body index per triangle; None = one body
    build_body: int = 0                   # the body that is sliced and sprayed
    start_face: int | None = None         # flat face of the build body the layers grow away from
    cylinders: dict[int, CylFace] = field(default_factory=dict)   # B-rep face index -> cylindrical face
    coat_faces: set[int] = field(default_factory=set)   # CAD faces to coat (surface mode), any body
    krl_base: np.ndarray | None = None    # 4x4 part-CAD <- KRL base frame (None: the faceplate BASE)
    krl_base_face: int | None = None      # the flat face it was picked on
    fixed_in_cell: bool = False           # held in the cell, not on the turntable (table kept at 0)

    @property
    def n_bodies(self) -> int:
        return 1 if self.body_ids is None else int(self.body_ids.max()) + 1

    def in_build(self) -> np.ndarray:
        """(m,) True for the triangles of the build body."""
        return np.ones(len(self.faces), bool) if self.body_ids is None else self.body_ids == self.build_body

    def body_of_face(self, face_id: int) -> int:
        if self.body_ids is None:
            return 0
        return int(self.body_ids[np.flatnonzero(self.face_ids == face_id)[0]])

    def without(self, bodies) -> "Part":
        """Copy without the given bodies (renumbered); the build body must stay."""
        drop = set(bodies) - {None}
        if not drop or self.body_ids is None:
            return replace(self, placement=self.placement.copy(), coat_faces=set(self.coat_faces))
        if self.build_body in drop:
            raise ValueError("the build body can't be left out")
        keep = ~np.isin(self.body_ids, list(drop))
        vertices, faces = _submesh(self.vertices, self.faces[keep])
        kept = [b for b in range(self.n_bodies) if b not in drop]
        renumber = np.zeros(self.n_bodies, np.int64)
        renumber[kept] = np.arange(len(kept))
        face_ids = self.face_ids[keep]
        present = set(face_ids.tolist())
        return replace(self, vertices=vertices, faces=faces, face_ids=face_ids,
                       planar={k: p for k, p in self.planar.items() if k in present},
                       cylinders={k: c for k, c in self.cylinders.items() if k in present},
                       placement=self.placement.copy(),
                       base_face=self.base_face if self.base_face in present else None,
                       start_face=self.start_face if self.start_face in present else None,
                       coat_faces=self.coat_faces & present,
                       krl_base=self.krl_base if self.krl_base_face in present | {None} else None,
                       krl_base_face=self.krl_base_face if self.krl_base_face in present else None,
                       body_ids=renumber[self.body_ids[keep]] if len(kept) > 1 else None,
                       build_body=int(renumber[self.build_body]) if len(kept) > 1 else 0)


def grow_tangent(part: Part, faces, max_deg: float = 10.0) -> set[int]:
    """`faces` plus every CAD face reached from them across shared edges where
    the triangle normals on the two sides are within `max_deg` (smooth joins,
    e.g. a fillet onto a flat; not a sharp box edge)."""
    import trimesh
    mesh = trimesh.Trimesh(part.vertices, part.faces, process=False)
    mesh.merge_vertices()                       # faces are tessellated apart: join their edges
    pairs = mesh.face_adjacency
    ids = part.face_ids[pairs]
    across = ids[:, 0] != ids[:, 1]
    if part.body_ids is not None:               # never cross into another body
        across &= part.body_ids[pairs[:, 0]] == part.body_ids[pairs[:, 1]]
    n = mesh.face_normals
    cos = np.einsum("ij,ij->i", n[pairs[across, 0]], n[pairs[across, 1]])
    links = np.unique(np.sort(ids[across][cos >= np.cos(np.radians(max_deg))], axis=1), axis=0)
    nbrs: dict[int, list[int]] = {}
    for f, g in links.tolist():
        nbrs.setdefault(f, []).append(g)
        nbrs.setdefault(g, []).append(f)
    out, todo = set(faces), list(faces)
    while todo:
        f = todo.pop()
        for g in nbrs.get(f, ()):
            if g not in out:
                out.add(g)
                todo.append(g)
    return out


def _face_rim(part: Part, face_id: int) -> tuple[np.ndarray, np.ndarray]:
    """(vertices, rim segments) of CAD face `face_id`: its outline (and hole) edges."""
    import trimesh
    mesh = trimesh.Trimesh(part.vertices, part.faces[part.face_ids == face_id])   # merges vertices
    v, edges = mesh.vertices, mesh.edges_sorted
    _, index, count = np.unique(edges, axis=0, return_index=True, return_counts=True)
    return v, edges[index[count == 1]]


def krl_base_frame(part: Part, face_id: int, click) -> np.ndarray:
    """Part-CAD <- KRL base picked by a click (part-CAD point) on flat face `face_id`:
    +Z the face's outward normal, +X along the face's straight edge nearest the
    click, the origin at the end of that edge nearest the click."""
    z = np.asarray(part.planar[face_id].normal, float)
    z /= np.linalg.norm(z)
    v, rim = _face_rim(part, face_id)
    click = np.asarray(click, float)
    a, ab = v[rim[:, 0]], v[rim[:, 1]] - v[rim[:, 0]]
    t = np.clip(np.einsum("ij,ij->i", click - a, ab) / np.einsum("ij,ij->i", ab, ab), 0, 1)
    k = int(np.argmin(np.linalg.norm(a + t[:, None] * ab - click, axis=1)))
    d = ab[k] / np.linalg.norm(ab[k])
    # the whole straight edge: rim segments that continue it in line
    line, grown = {int(rim[k, 0]), int(rim[k, 1])}, True
    while grown:
        grown = False
        for i, j in rim:
            if (i in line) != (j in line) and all(np.linalg.norm(np.cross(v[p] - a[k], d)) < 1e-3 for p in (i, j)):
                line |= {int(i), int(j)}
                grown = True
    line = list(line)
    along = v[line] @ d
    lo, hi = v[line[int(np.argmin(along))]], v[line[int(np.argmax(along))]]
    origin, other = (lo, hi) if np.linalg.norm(click - lo) <= np.linalg.norm(click - hi) else (hi, lo)
    x = other - origin
    x = (x - (x @ z) * z) / np.linalg.norm(x - (x @ z) * z)
    out = np.eye(4)
    out[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    out[:3, 3] = origin
    return out


def _frame_xz(origin, x, z) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    out[:3, 3] = origin
    return out


def krl_base_from_planes(planes, toward, min_deg: float = 10.0) -> np.ndarray:
    """Part-CAD <- KRL base from three flat faces (`PlanarFace`s, in pick order): the
    origin where the three planes meet, +Z the 1st face's outward normal, +X along the
    line where the 1st and 2nd planes meet, pointing from the origin toward the side of
    the point `toward` (e.g. the part's centre: into the part's extent).
    ValueError if two planes are within `min_deg` of parallel or the 3rd is near
    parallel to the 1st-2nd line (no single meeting point)."""
    n = np.array([np.asarray(p.normal, float) / np.linalg.norm(p.normal) for p in planes])
    d = np.cross(n[0], n[1])
    s = np.sin(np.radians(min_deg))
    if min(np.linalg.norm(np.cross(n[i], n[j])) for i, j in ((0, 1), (0, 2), (1, 2))) < s \
            or abs(n[2] @ d) / np.linalg.norm(d) < s:
        raise ValueError("Two of the faces are (nearly) parallel: pick three faces meeting at a corner")
    origin = np.linalg.solve(n, [n[k] @ np.asarray(planes[k].point, float) for k in range(3)])
    x = d / np.linalg.norm(d)
    if x @ (np.asarray(toward, float) - origin) < 0:
        x = -x
    return _frame_xz(origin, x, n[0])


def vertex_edges(part: Part, point, tol: float = 1e-4) -> tuple[np.ndarray, np.ndarray]:
    """(unit directions of the CAD-face rim edges leaving the vertex at `point`,
    the area-weighted mean outward normal of the triangles around it)."""
    point = np.asarray(point, float)
    near = np.flatnonzero(np.linalg.norm(part.vertices - point, axis=1) < tol)
    tris = np.flatnonzero(np.isin(part.faces, near).any(axis=1))
    tri = part.vertices[part.faces[tris]]
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]).sum(axis=0)
    normal = normal / np.linalg.norm(normal) if np.linalg.norm(normal) > 1e-12 else np.array([0, 0, 1.0])
    dirs = []
    for f in sorted(set(part.face_ids[tris].tolist())):
        v, rim = _face_rim(part, f)
        for i, j in rim:
            for a, b in ((i, j), (j, i)):
                if np.linalg.norm(v[a] - point) < tol:
                    dirs.append((v[b] - point) / np.linalg.norm(v[b] - point))
    dirs = np.unique(np.round(np.array(dirs).reshape(-1, 3), 9), axis=0)
    return dirs, normal


def krl_base_at_vertex(point, z, edges, toward=None) -> np.ndarray:
    """Part-CAD <- KRL base at a vertex `point`: +Z along `z` (the flat face under the
    cursor's normal, or the vertex's mean normal), +X along the edge (unit `edges`
    directions from the vertex, laid into the XY plane) closest to the direction
    `toward` (cursor - vertex); the first edge without one. No edge off the Z
    axis: X is the part-CAD X (or Y) laid into the plane."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    cand = np.asarray(edges, float).reshape(-1, 3)
    cand = cand - np.outer(cand @ z, z)
    lens = np.linalg.norm(cand, axis=1)
    cand = cand[lens > 0.1] / lens[lens > 0.1, None]
    if len(cand):
        k = 0
        if toward is not None:
            t = np.asarray(toward, float) - (np.asarray(toward, float) @ z) * z
            if np.linalg.norm(t) > 1e-9:
                k = int(np.argmax(cand @ t))
        x = cand[k]
    else:
        x = np.array([1.0, 0, 0]) - z[0] * z
        if np.linalg.norm(x) < 0.1:
            x = np.array([0, 1.0, 0]) - z[1] * z
        x /= np.linalg.norm(x)
    return _frame_xz(np.asarray(point, float), x, z)


def spin_frame(t, deg: float) -> np.ndarray:
    """Frame `t` turned by `deg` about its own Z (origin and Z kept)."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    rz = np.eye(4)
    rz[:2, :2] = [[c, -s], [s, c]]
    return np.asarray(t, float) @ rz


def flip_z(t) -> np.ndarray:
    """Frame `t` with Z reversed: origin and X kept, Y follows (right-handed)."""
    out = np.array(t, float)
    out[:3, 1:3] *= -1
    return out


@dataclass
class Substrate:
    """Plate the part is built on, centred on the faceplate."""
    length: float = 200.0   # along BASE X
    width: float = 200.0    # along BASE Y
    thickness: float = 10.0

    @property
    def top(self) -> float:
        return self.thickness

    def mesh(self) -> tuple[np.ndarray, np.ndarray]:
        """(vertices, faces) of the plate in BASE."""
        import trimesh
        box = trimesh.creation.box(extents=(self.length, self.width, self.thickness))
        return np.asarray(box.vertices) + [0.0, 0.0, self.thickness / 2], np.asarray(box.faces)


def rotation_between(a, b) -> np.ndarray:
    """Smallest rotation taking unit vector `a` to unit vector `b`
    (180 degrees about X, or another axis normal to `a`, if they are opposite)."""
    a, b = np.asarray(a, float) / np.linalg.norm(a), np.asarray(b, float) / np.linalg.norm(b)
    axis, c = np.cross(a, b), np.dot(a, b)
    s = np.linalg.norm(axis)
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        axis = np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0])
        axis = np.cross(axis, a)  # = X when a is along Z
        axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]) / s
    return np.eye(3) + s * k + (1 - c) * k @ k


def _apply(t, points) -> np.ndarray:
    return points @ t[:3, :3].T + t[:3, 3]


def placed_vertices(part: Part) -> np.ndarray:
    """Part vertices in BASE."""
    return _apply(part.placement, part.vertices)


def _submesh(vertices, faces) -> tuple[np.ndarray, np.ndarray]:
    """Only the vertices `faces` use, re-indexed."""
    used, inverse = np.unique(faces, return_inverse=True)
    return vertices[used], inverse.reshape(faces.shape)


def build_mesh(part: Part) -> tuple[np.ndarray, np.ndarray]:
    """(vertices in BASE, faces) of the build body."""
    return _submesh(placed_vertices(part), part.faces[part.in_build()])


def build_frame(part: Part) -> np.ndarray:
    """BASE <- build frame: layers lie parallel to its XY plane and stack along
    its +Z from z = 0. From the start face if one is set (Z into the build
    body), otherwise the bottom of the build body with BASE axes."""
    if part.start_face is None:
        v, _ = build_mesh(part)
        return frame(z=v[:, 2].min())
    face = part.planar[part.start_face]
    r, t = part.placement[:3, :3], part.placement[:3, 3]
    z = -(r @ face.normal)
    x = np.array([1.0, 0.0, 0.0]) - z[0] * z
    if np.linalg.norm(x) < 0.1:
        x = np.array([0.0, 1.0, 0.0]) - z[1] * z
    x /= np.linalg.norm(x)
    out = np.eye(4)
    out[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    out[:3, 3] = r @ face.point + t
    return out


def other_mesh(part: Part) -> tuple[np.ndarray, np.ndarray] | None:
    """(vertices in BASE, faces) of every body except the build body, or None."""
    rest = ~part.in_build()
    return _submesh(placed_vertices(part), part.faces[rest]) if rest.any() else None


def _xy_centre(points) -> np.ndarray:
    return (points[:, :2].min(axis=0) + points[:, :2].max(axis=0)) / 2


def _face_down(part: Part, face_id: int, placement, plate_top: float) -> np.ndarray:
    """`placement` turned so the face points down, lowest point on the floor
    at `plate_top`; the XY translation is left as it is."""
    face = part.planar[face_id]
    t = np.array(placement, float)
    t[:3, :3] = rotation_between(t[:3, :3] @ face.normal, DOWN) @ t[:3, :3]
    t[2, 3] = plate_top - (part.vertices @ t[:3, :3].T)[:, 2].min()
    return t


def snap_face_down(part: Part, face_id: int, plate_top: float) -> np.ndarray:
    """Placement with planar face `face_id` pointing down (normal -Z) and the
    lowest point on the floor at `plate_top` (for a single convex part: the
    face lies on the floor), keeping the XY bounding-box centre where it was."""
    if face_id not in part.planar:
        raise ValueError("Only flat faces can be placed on the plate")
    before = _xy_centre(placed_vertices(part))
    t = _face_down(part, face_id, part.placement, plate_top)
    t[:2, 3] += before - _xy_centre(_apply(t, part.vertices))
    return t


def centre_on_faceplate(part: Part) -> np.ndarray:
    """Placement moved in XY so the part's bounding box is centred on the table axis."""
    t = np.array(part.placement, float)
    t[:2, 3] -= _xy_centre(placed_vertices(part))
    return t


def constrain(part: Part, placement, plate_top: float) -> np.ndarray:
    """Nearest placement to `placement` that keeps the base face down on the floor:
    tilt removed, yaw and XY translation kept. Unchanged if no build face is set."""
    if part.base_face is None:
        return placement
    return _face_down(part, part.base_face, placement, plate_top)


def drop_on_plate(part: Part, plate_top: float) -> np.ndarray:
    """Placement with the same orientation, lowest point on the plate and
    XY bounding box centred on the table axis."""
    t = centre_on_faceplate(part)
    t[2, 3] += plate_top - _apply(t, part.vertices)[:, 2].min()
    return t


def lower_to_floor(part: Part, plate_top: float) -> np.ndarray:
    """Placement with the same orientation and XY, moved along Z so the lowest
    point (all bodies) is on the floor at `plate_top`."""
    t = np.array(part.placement, float)
    t[2, 3] += plate_top - placed_vertices(part)[:, 2].min()
    return t
