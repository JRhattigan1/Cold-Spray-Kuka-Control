"""Read STEP parts and turn them into triangle meshes."""

from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Compound

from coldspray.cad.part import CylFace, PlanarFace


def load_step(path):
    """The whole STEP file as one OpenCascade shape (units: mm)."""
    reader = STEPControl_Reader()
    if reader.ReadFile(str(Path(path))) != IFSelect_RetDone:
        raise ValueError(f"could not read STEP file {path}")
    reader.TransferRoots()
    return reader.OneShape()


def tessellate(shape, deflection=0.1, angle=0.2):
    """Triangle mesh (vertices Nx3, faces Mx3) of a shape.

    `deflection` is the max distance (mm) between mesh and true surface.
    """
    v, f, _, _ = _mesh(shape, deflection, angle, planes=False)
    return v, f


def _plane(face):
    """The face's plane (outward normal) if it is flat, else None."""
    surf = BRepAdaptor_Surface(face)
    if surf.GetType() != GeomAbs_Plane:
        return None
    pln = surf.Plane()
    p, d = pln.Location(), pln.Axis().Direction()
    n = np.array([d.X(), d.Y(), d.Z()])
    if face.Orientation() == TopAbs_REVERSED:
        n = -n
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    return PlanarFace(np.array([p.X(), p.Y(), p.Z()]), n / np.linalg.norm(n), props.Mass())


def _cylinder(face):
    """The face's cylinder (axis point, unit axis, radius, area) if it is one, else None."""
    surf = BRepAdaptor_Surface(face)
    if surf.GetType() != GeomAbs_Cylinder:
        return None
    cyl = surf.Cylinder()
    ax = cyl.Axis()
    p, d = ax.Location(), ax.Direction()
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    axis = np.array([d.X(), d.Y(), d.Z()])
    return CylFace(np.array([p.X(), p.Y(), p.Z()]), axis / np.linalg.norm(axis), cyl.Radius(), props.Mass())


def tessellate_with_ids(shape, deflection=0.1, angle=0.2):
    """Like `tessellate`, plus the B-rep face index of each triangle and the
    flat faces: (vertices, faces, face_ids, {face index: PlanarFace}).
    Faces are numbered from 0 in TopExp_Explorer order.
    """
    return _mesh(shape, deflection, angle, planes=True)


def bodies(shape) -> list:
    """The solids of a shape, plus one compound of any faces outside a solid."""
    out, exp = [], TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(exp.Current())
        exp.Next()
    loose, builder = TopoDS_Compound(), BRep_Builder()
    builder.MakeCompound(loose)
    exp, n = TopExp_Explorer(shape, TopAbs_FACE, TopAbs_SOLID), 0
    while exp.More():
        builder.Add(loose, exp.Current())
        n += 1
        exp.Next()
    return out + ([loose] if n else [])


def tessellate_bodies(shape, deflection=0.1, angle=0.2):
    """`tessellate_with_ids` body by body: (vertices, faces, face_ids, planar,
    body_ids, cylinders), with face indices numbered on across the bodies."""
    BRepMesh_IncrementalMesh(shape, deflection, False, angle, True)
    vs, fs, ids, planar, body_ids, n_verts, n_faces = [], [], [], {}, [], 0, 0
    cylinders = {}
    for b, body in enumerate(bodies(shape)):
        cyl = {}
        v, f, i, p, count = _mesh(body, deflection, angle, planes=True, count=True, cylinders=cyl,
                                    meshed=True)
        cylinders |= {k + n_faces: c for k, c in cyl.items()}
        vs.append(v)
        fs.append(f + n_verts)
        ids.append(i + n_faces)
        planar |= {k + n_faces: face for k, face in p.items()}
        body_ids.append(np.full(len(f), b, np.int64))
        n_verts, n_faces = n_verts + len(v), n_faces + count
    if not vs:
        raise ValueError("no faces in the shape")
    return np.vstack(vs), np.vstack(fs), np.concatenate(ids), planar, np.concatenate(body_ids), cylinders


def _mesh(shape, deflection, angle, planes, count=False, cylinders=None, meshed=False):
    """`cylinders`, if given, is filled with {face index: CylFace}.
    `meshed=True` skips BRepMesh (the caller already meshed the shape)."""
    if not meshed:
        BRepMesh_IncrementalMesh(shape, deflection, False, angle, True)
    verts, faces, ids, planar, offset = [], [], [], {}, 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    index = 0
    while exp.More():
        face = TopoDS.Face(exp.Current())
        plane = _plane(face) if planes else None
        if plane is not None:
            planar[index] = plane
        elif cylinders is not None:
            cyl = _cylinder(face)
            if cyl is not None:
                cylinders[index] = cyl
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X(), p.Y(), p.Z()))
            flip = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = (n - 1 + offset for n in tri.Triangle(i).Get())
                faces.append((a, c, b) if flip else (a, b, c))
            ids += [index] * tri.NbTriangles()
            offset += tri.NbNodes()
        exp.Next()
        index += 1
    out = (np.array(verts, float).reshape(-1, 3), np.array(faces, np.int64).reshape(-1, 3),
           np.array(ids, np.int64), planar)
    return out + (index,) if count else out
