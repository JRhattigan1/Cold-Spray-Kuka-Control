"""Collision checking along a toolpath (FCL via trimesh).

Moving bodies: robot links A1..A6, spray gun and nozzle.
Checked against: the fixed cell (cropped to the robot's working volume at the
configured gantry position), the turntable body and faceplate (moving with
E2/E5), the workpiece - part, the substrate plate it sits on and any other
bodies of the part's assembly ("fixture": the component being repaired,
clamps), all clamped to the faceplate - and, for the gun and nozzle only, the
robot's own base and arm. Workpiece bodies touching each other are not
checked (all are environment).

Contacts that exist in the CAD as drawn (robot at its CAD pose and gantry
position, turntable at 0 - parts that touch by design, e.g. the robot base
on its mounting) are learned once and ignored; contacts with the part,
substrate, fixture and turntable are never ignored.

Limits: meshes are simplified (robot links decimated, large cell meshes
merged to ~SIMPLIFY_MM), so near misses of a few mm may be reported either
way. Mesh-vs-mesh checks find surfaces crossing: a body that is entirely
inside another at a checked point is not reported - check points closely
enough (a few mm of travel) that entering it is caught.
"""

from dataclasses import dataclass

import fcl
import numpy as np
import pyvista as pv
import trimesh

from coldspray.cell import base_world, robroot, turntable_motion
from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.models import CellConfig
from coldspray.sim.cell_model import OPTIONAL, CellMeshes

LINK_NAMES = ["A1", "A2", "A3", "A4", "A5", "A6"]
WORKPIECE = ("part", "substrate", "fixture")     # bodies clamped to the faceplate
NEVER_ALLOWED = {*WORKPIECE, "faceplate", "turntable"}
SIMPLIFY_MM = 10.0          # merge cell-mesh points closer than this
SIMPLIFY_ABOVE = 50_000     # ... for meshes with more triangles than this


@dataclass
class Collision:
    sample: int
    pairs: set[tuple[str, str]]


class _Manager(trimesh.collision.CollisionManager):
    """trimesh's manager, but building each BVH from plain numpy arrays: trimesh
    passes its TrackedArray, which FCL walks row by row ~50x slower."""

    def _get_fcl_obj(self, mesh):
        bvh = fcl.BVHModel()
        v = np.ascontiguousarray(np.asarray(mesh.vertices).view(np.ndarray), dtype=np.float64)
        f = np.ascontiguousarray(np.asarray(mesh.faces).view(np.ndarray), dtype=np.int64)
        bvh.beginModel(num_tris_=len(f), num_vertices_=len(v))
        bvh.addSubModel(verts=v, triangles=f)
        bvh.endModel()
        return bvh


def _triangles(poly: pv.PolyData) -> tuple[np.ndarray, np.ndarray]:
    if not poly.is_all_triangles:
        poly = poly.triangulate()
    return np.asarray(poly.points), poly.faces.reshape(-1, 4)[:, 1:]


def _trimesh(poly: pv.PolyData, reduction=0.0) -> trimesh.Trimesh:
    if reduction > 0 and poly.n_cells > 2000:
        poly = poly.triangulate().decimate(reduction)
    return trimesh.Trimesh(*_triangles(poly), process=False)


def _cell_mesh(poly: pv.PolyData, centre, half_size) -> trimesh.Trimesh | None:
    """Triangles with a corner inside the cube of `half_size` around `centre`,
    simplified if large."""
    if poly.n_cells == 0:
        return None
    pts, faces = _triangles(poly)
    inside = np.all(np.abs(pts - centre) <= half_size, axis=1)
    faces = faces[inside[faces].any(axis=1)]
    if not len(faces):
        return None
    if len(faces) > SIMPLIFY_ABOVE:
        cropped = pv.PolyData(pts, np.hstack([np.full((len(faces), 1), 3), faces]).ravel())
        merged = cropped.clean(tolerance=SIMPLIFY_MM, absolute=True, lines_to_points=False,
                               polys_to_lines=False, strips_to_polys=False)
        pts, faces = _triangles(merged)
        a, b, c = faces.T
        faces = faces[(a != b) & (b != c) & (a != c)]   # drop collapsed triangles
    return trimesh.Trimesh(pts, faces, process=False)


class CollisionChecker:
    def __init__(self, robot: Robot, cell: CellConfig, meshes: CellMeshes, gun=None, part=None,
                 cad_pose=HOME_DEG, reach_mm=3600.0, substrate=None, fixture=None):
        self.robot, self.cell = robot, cell
        self.world_from_robot = robroot(cell)
        self.cad_frames = robot.link_frames(cad_pose)
        self.gun = gun

        self.arm = _Manager()      # A1..A6
        self.tool = _Manager()     # gun + nozzle
        self.env = _Manager()      # cell, table, part, substrate, fixture
        self.own = _Manager()      # robot base + A1..A5, for the tool
        for k, name in enumerate(LINK_NAMES, start=1):
            mesh = _trimesh(meshes.robot[k], 0.85)
            self.arm.add_object(name, mesh)
            if name != "A6":
                self.own.add_object(name, mesh)
        self.own.add_object("robot base", _trimesh(meshes.robot[0], 0.85), self.world_from_robot)
        if gun is not None:
            self.tool.add_object("gun", trimesh.Trimesh(gun.vertices, gun.faces, process=False))
            if gun.nozzle is not None:
                self.tool.add_object("nozzle", trimesh.Trimesh(*gun.nozzle, process=False))

        # Fixed equipment within reach of the robot at the configured gantry position
        centre = self.world_from_robot[:3, 3]
        sources = [("cell", meshes.static)] + [(lbl, p) for lbl, p in meshes.groups.items()
                                               if OPTIONAL[lbl][2] is None]
        for label, poly in sources:
            mesh = _cell_mesh(poly, centre, reach_mm)
            if mesh is not None:
                self.env.add_object(label, mesh)
        rotating = meshes.rotating
        for label, poly in meshes.groups.items():
            if OPTIONAL[label][2] == "rotating":
                rotating = rotating.merge(poly)
        for label, poly in (("turntable", meshes.tilting), ("faceplate", rotating)):
            if poly.n_cells:
                self.env.add_object(label, _trimesh(poly))
        self.set_workpiece(part, substrate, fixture)

        # Learn contacts that exist in the CAD as drawn
        self.allowed: set[tuple[str, str]] = set()
        working = self.world_from_robot
        self.set_placement(robroot(cell, self._cad_e1(cell)) if cell.gantry else robroot(cell))
        contacts = self.check(np.asarray(cad_pose, float), {})
        self.allowed = {p for p in contacts if not NEVER_ALLOWED & set(p)}
        self.set_placement(working)

    @staticmethod
    def _cad_e1(cell: CellConfig) -> float:
        """Gantry position at which the CAD draws the carriage (E1 = 0, clamped to the limits)."""
        lo, hi = cell.gantry.limits_mm
        return float(np.clip(0.0, lo, hi))

    def set_placement(self, world_from_robot):
        """Robot root in WORLD. The cell was cropped around the gantry position the
        checker was built for - build a new checker for a different position."""
        self.world_from_robot = world_from_robot
        self.own.set_transform("robot base", world_from_robot)

    def set_gun(self, robot: Robot, gun):
        """Swap the tool (e.g. another nozzle): new TCP and gun / nozzle meshes."""
        self.robot, self.gun = robot, gun
        for name in ("gun", "nozzle"):
            if name in self.tool._objs:
                self.tool.remove_object(name)
        if gun is not None:
            self.tool.add_object("gun", trimesh.Trimesh(gun.vertices, gun.faces, process=False))
            if gun.nozzle is not None:
                self.tool.add_object("nozzle", trimesh.Trimesh(*gun.nozzle, process=False))

    def set_workpiece(self, part_mesh=None, substrate_mesh=None, fixture_mesh=None):
        """Replace the part, substrate and fixture meshes (vertices, faces in BASE
        coordinates); None removes that body."""
        for name, mesh in zip(WORKPIECE, (part_mesh, substrate_mesh, fixture_mesh)):
            if name in self.env._objs:
                self.env.remove_object(name)
            if mesh is not None:
                self.env.add_object(name, trimesh.Trimesh(*mesh, process=False))

    def _place(self, q, axes):
        """Move robot, gun and turntable bodies to joints `q` / table `axes`."""
        frames = self.robot.link_frames(q)
        for k, name in enumerate(LINK_NAMES):
            m = self.world_from_robot @ frames[k] @ np.linalg.inv(self.cad_frames[k])
            self.arm.set_transform(name, m)
            if name != "A6":
                self.own.set_transform(name, m)
        if self.gun is not None:
            g = self.world_from_robot @ self.robot.flange(q) @ self.gun.to_flange
            for name in ("gun", "nozzle"):
                if name in self.tool._objs:
                    self.tool.set_transform(name, g)
        tilt, face = turntable_motion(self.cell, axes)
        base = base_world(self.cell, axes)
        for name, m in (("turntable", tilt), ("faceplate", face)) + tuple((w, base) for w in WORKPIECE):
            if name in self.env._objs:
                self.env.set_transform(name, m)

    def check(self, q, axes) -> set[tuple[str, str]]:
        """Colliding (moving body, other body) pairs at joints `q` and table `axes`."""
        self._place(q, axes)
        pairs = set()
        for mgr in (self.arm, self.tool):
            hit, names = mgr.in_collision_other(self.env, return_names=True)
            if hit:
                pairs |= set(names)
        hit, names = self.tool.in_collision_other(self.own, return_names=True)
        if hit:
            pairs |= set(names)
        return pairs - self.allowed

    def check_path(self, joints, axes_per_sample, every=1, progress=None, cancel=None) -> list[Collision]:
        """Check every `every`-th sample (and the last); unreachable samples are skipped.

        `progress(fraction)`: called with rising values 0..1 (1.0 at the end).
        `cancel()`: polled before each sample; True stops early and returns the hits
        found so far. This usually runs on a worker thread, so both callbacks are
        called from that thread: keep them trivial and thread-safe (store a float /
        read a flag that the GUI polls with a QTimer, e.g. JobProgress.set_fraction
        and JobProgress.is_cancelled) - never touch widgets from them."""
        n = len(joints)
        hits = []
        if n == 0:
            if progress is not None:
                progress(1.0)
            return hits
        samples = sorted(set(range(0, n, max(1, every))) | {n - 1})
        report = max(1, len(samples) // 200)              # at most ~200 progress calls
        if progress is not None:
            progress(0.0)
        for k, i in enumerate(samples):
            if cancel is not None and cancel():
                return hits
            if progress is not None and k % report == 0:
                progress(k / len(samples))
            if np.isnan(joints[i, 0]):
                continue
            pairs = self.check(joints[i], axes_per_sample(i))
            if pairs:
                hits.append(Collision(i, pairs))
        if progress is not None:
            progress(1.0)
        return hits
