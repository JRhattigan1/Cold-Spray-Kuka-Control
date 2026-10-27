"""NX-style placement constraints and a small solver for the part placement.

Entities are planes, axes and points. A `moving` entity belongs to the part
and is given in part-CAD coords (it moves with the placement); a fixed one is
a cell reference in BASE (faceplate, substrate top, datum planes through the
table centre). A constraint ties entity `a` to entity `b` (and `c` for
`centre_between`). `solve` finds the placement nearest the current one (the
smallest rotation about `pivot` plus the smallest translation) that satisfies
every active constraint, reports per-constraint residuals, the degrees of
freedom left, and whether the set conflicts.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

TOL = 1e-3          # residual (mm, or unit-vector difference) counted as satisfied
_REG = 1e-5         # weight of the "smallest move" pull towards the start pose
_SCALE = 100.0      # mm per radian when weighing rotation against translation


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


@dataclass
class Plane:
    point: np.ndarray
    normal: np.ndarray
    moving: bool = False

    def __post_init__(self):
        self.point, self.normal = np.asarray(self.point, float), _unit(self.normal)


@dataclass
class Axis:
    point: np.ndarray
    dir: np.ndarray
    moving: bool = False

    def __post_init__(self):
        self.point, self.dir = np.asarray(self.point, float), _unit(self.dir)


@dataclass
class PointRef:
    point: np.ndarray
    moving: bool = False

    def __post_init__(self):
        self.point = np.asarray(self.point, float)


# --- entities from the part and the cell ------------------------------------

def part_plane(part, face_id: int) -> Plane:
    """The plane of a flat part face (outward normal), moving with the part."""
    f = part.planar[face_id]
    return Plane(f.point, f.normal, moving=True)


def part_axis(part, face_id: int) -> Axis:
    """The axis of a cylindrical part face, moving with the part."""
    c = getattr(part, "cylinders", {})[face_id]
    return Axis(c.point, c.axis, moving=True)


def faceplate(offset: float = 0.0) -> Plane:
    """Faceplate top (BASE z = 0), normal up."""
    return Plane((0, 0, offset), (0, 0, 1))


def substrate_top(substrate, offset: float = 0.0) -> Plane:
    return Plane((0, 0, substrate.top + offset), (0, 0, 1))


_DATUM_NORMALS = {"XY": (0, 0, 1), "XZ": (0, 1, 0), "YZ": (1, 0, 0)}


def datum(name: str, offset: float = 0.0) -> Plane:
    """Datum plane XY / XZ / YZ through the table centre, moved `offset` along its normal."""
    n = np.array(_DATUM_NORMALS[name], float)
    return Plane(n * offset, n)


def table_axis() -> Axis:
    """The turntable axis (BASE Z through the faceplate centre)."""
    return Axis((0, 0, 0), (0, 0, 1))


def reference(name: str, part=None, substrate=None, offset: float = 0.0):
    """Entity from a pick name: "face:<id>" (plane, or axis of a cylinder),
    "datum:XY|XZ|YZ", "faceplate", "substrate" or "table_axis"."""
    kind, _, arg = name.partition(":")
    if kind == "face":
        fid = int(arg)
        return part_plane(part, fid) if fid in part.planar else part_axis(part, fid)
    if kind == "datum":
        return datum(arg, offset)
    if kind == "faceplate":
        return faceplate(offset)
    if kind == "substrate":
        return substrate_top(substrate, offset) if substrate is not None else faceplate(offset)
    if kind == "table_axis":
        return table_axis()
    raise ValueError(f"unknown reference {name!r}")


# --- constraints --------------------------------------------------------------

KINDS = ("touch", "align", "distance", "parallel", "perpendicular", "angle", "centre", "centre_between", "fix")


@dataclass
class Constraint:
    kind: str
    a: object = None
    b: object = None
    value: float = 0.0              # distance (mm) or angle (deg)
    c: object = None                # second part face for centre_between
    suppressed: bool = False
    pose: np.ndarray | None = None  # the locked placement for fix

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"unknown constraint {self.kind!r}")


def touch(a, b): return Constraint("touch", a, b)
def align(a, b): return Constraint("align", a, b)
def distance(a, b, d): return Constraint("distance", a, b, d)
def parallel(a, b): return Constraint("parallel", a, b)
def perpendicular(a, b): return Constraint("perpendicular", a, b)
def angle(a, b, deg): return Constraint("angle", a, b, deg)
def centre(a, b): return Constraint("centre", a, b)
def centre_between(a, c, b): return Constraint("centre_between", a, b, c=c)
def fix(placement): return Constraint("fix", pose=np.array(placement, float))


def _place(e, t):
    """Entity in BASE under placement `t` (fixed entities unchanged)."""
    if not e.moving:
        return e
    r, p = t[:3, :3], t[:3, :3] @ e.point + t[:3, 3]
    if isinstance(e, Plane):
        return Plane(p, r @ e.normal)
    if isinstance(e, Axis):
        return Axis(p, r @ e.dir)
    return PointRef(p)


def _dir(e):
    return e.normal if isinstance(e, Plane) else e.dir


def _off_line(p, ax):
    """Vector from line `ax` to point `p`, normal to the line."""
    d = p - ax.point
    return d - (d @ ax.dir) * ax.dir


def _residual(k: Constraint, t) -> np.ndarray:
    if k.kind == "fix":
        rv = Rotation.from_matrix(t[:3, :3] @ k.pose[:3, :3].T).as_rotvec() * _SCALE
        return np.concatenate([rv, t[:3, 3] - k.pose[:3, 3]])
    a, b = _place(k.a, t), _place(k.b, t)
    if k.kind == "touch":
        return np.append(a.normal + b.normal, (a.point - b.point) @ b.normal)
    if k.kind == "align":
        return np.append(a.normal - b.normal, (a.point - b.point) @ b.normal)
    if k.kind == "distance":
        return np.append(np.cross(a.normal, b.normal), abs((a.point - b.point) @ b.normal) - k.value)
    if k.kind == "parallel":
        return np.cross(_dir(a), _dir(b))
    if k.kind == "perpendicular":
        return np.array([_dir(a) @ _dir(b)])
    if k.kind == "angle":
        return np.array([_dir(a) @ _dir(b) - np.cos(np.radians(k.value))])
    if k.kind == "centre":
        if isinstance(a, PointRef):
            return _off_line(a.point, b)
        if isinstance(b, PointRef):
            return _off_line(b.point, a)
        return np.concatenate([np.cross(a.dir, b.dir), _off_line(a.point, b)])
    if k.kind == "centre_between":
        c = _place(k.c, t)
        return np.append(np.cross(a.normal, b.normal), ((a.point + c.point) / 2 - b.point) @ b.normal)
    raise ValueError(k.kind)


@dataclass
class SolveResult:
    placement: np.ndarray
    residuals: list[float]          # per constraint (0 for suppressed ones)
    dof_left: int
    status: str                     # "ok" | "conflicting" (some residual above tol)
    redundant: bool = False         # some constraints fix the same DOF (fine if consistent)


def _pose(x, start, pivot):
    """Start placement rotated by rotvec x[:3]/_SCALE about the BASE image of
    part point `pivot`, then moved by x[3:]."""
    r = Rotation.from_rotvec(x[:3] / _SCALE).as_matrix()
    t = np.eye(4)
    c = start[:3, :3] @ pivot + start[:3, 3]
    t[:3, :3] = r @ start[:3, :3]
    t[:3, 3] = r @ (start[:3, 3] - c) + c + x[3:]
    return t


def _rank(j) -> int:
    if j.size == 0:
        return 0
    s = np.linalg.svd(j, compute_uv=False)
    return int((s > 1e-6 * max(s[0], 1.0)).sum())


def _jacobian(fun, x, eps=1e-6) -> np.ndarray:
    f0 = fun(x)
    return np.column_stack([(fun(x + eps * e) - f0) / eps for e in np.eye(6)])


def solve(placement, constraints, pivot=None, tol=TOL) -> SolveResult:
    """Nearest placement to `placement` meeting the active `constraints`.

    Parameters are a rotation vector (about `pivot`, part-CAD coords, default
    the part origin) and a translation; a light pull towards zero keeps the
    move as small as possible and makes under-constrained sets well posed."""
    start = np.array(placement, float)
    pivot = np.zeros(3) if pivot is None else np.asarray(pivot, float)
    active = [k for k in constraints if not k.suppressed]

    def cons(x):
        t = _pose(x, start, pivot)
        return np.concatenate([_residual(k, t) for k in active]) if active else np.zeros(0)

    def run(x0):
        return least_squares(lambda x: np.concatenate([cons(x), _REG * x]), x0, method="lm", xtol=1e-12,
                             ftol=1e-12, gtol=1e-12).x

    x = np.zeros(6)
    if active:
        x = run(x)
        # a face pointing exactly the wrong way (180 deg) is a stationary point
        # of the normal residual: retry from half-turns and keep the best
        if np.abs(cons(x)).max() > tol:
            for ax in np.eye(3):
                y = run(np.concatenate([ax * np.pi * _SCALE, np.zeros(3)]))
                if np.linalg.norm(cons(y)) < np.linalg.norm(cons(x)):
                    x = y
    t = _pose(x, start, pivot)
    residuals = [0.0 if k.suppressed else float(np.linalg.norm(_residual(k, t))) for k in constraints]
    rank = _rank(_jacobian(cons, x)) if active else 0
    own = sum(_rank(_jacobian(lambda x, k=k: _residual(k, _pose(x, start, pivot)), x)) for k in active)
    status = "conflicting" if max(residuals, default=0.0) > tol else "ok"
    return SolveResult(t, residuals, 6 - rank, status, own > rank)
