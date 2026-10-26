"""Read STEP parts and turn them into triangle meshes."""

from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

from coldspray.cad.part import PlanarFace


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


def tessellate_with_ids(shape, deflection=0.1, angle=0.2):
    """Like `tessellate`, plus the B-rep face index of each triangle and the
    flat faces: (vertices, faces, face_ids, {face index: PlanarFace}).
    Faces are numbered from 0 in TopExp_Explorer order.
    """
    return _mesh(shape, deflection, angle, planes=True)


def _mesh(shape, deflection, angle, planes):
    BRepMesh_IncrementalMesh(shape, deflection, False, angle, True)
    verts, faces, ids, planar, offset = [], [], [], {}, 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    index = 0
    while exp.More():
        face = TopoDS.Face(exp.Current())
        plane = _plane(face) if planes else None
        if plane is not None:
            planar[index] = plane
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
    return (np.array(verts, float).reshape(-1, 3), np.array(faces, np.int64).reshape(-1, 3),
            np.array(ids, np.int64), planar)
