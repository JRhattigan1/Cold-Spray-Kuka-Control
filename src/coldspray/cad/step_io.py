"""Read STEP parts and turn them into triangle meshes."""

from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


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
    BRepMesh_IncrementalMesh(shape, deflection, False, angle, True)
    verts, faces, offset = [], [], 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
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
            offset += tri.NbNodes()
        exp.Next()
    return np.array(verts, float).reshape(-1, 3), np.array(faces, np.int64).reshape(-1, 3)
