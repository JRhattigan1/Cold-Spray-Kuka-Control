"""A part mesh with its CAD faces, placed on the turntable faceplate.

`Part.placement` maps part-CAD coordinates to the work BASE frame (faceplate
top centre, z = 0 on the faceplate surface). A part may be several bodies (an
assembly from one STEP file): one is the build body that is sliced and
sprayed; the others (the component being repaired, fixtures) only take up
space. Everything stands on the "floor": the top of an optional substrate
plate, or the faceplate itself. The base face, if one is picked, points down
(-Z) and the assembly's lowest point sits on the floor.
"""

from dataclasses import dataclass, field

import numpy as np
import trimesh

DOWN = np.array([0.0, 0.0, -1.0])


@dataclass
class PlanarFace:
    point: np.ndarray    # a point on the face plane, part-CAD coords
    normal: np.ndarray   # outward unit normal, part-CAD coords
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
