"""Convert a STEP assembly into a lightweight mesh cache for the viewer.

Reading a large STEP file takes minutes, so it is done once:

    python -m coldspray.cad.step_cache assets/cad/cell.stp assets/cad/cache/cell

This writes <out>.npz (one triangle mesh per unique part, in part coordinates)
and <out>.json (every part instance: its assembly path, which mesh it uses
and its 4x4 placement in mm). Assembly structure and names are preserved so
parts can later be grouped by robot axis, gantry, turntable, etc.
"""

import json
import sys
import time
from pathlib import Path

import numpy as np
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDF import TDF_Label, TDF_Tool
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool

from coldspray.cad.step_io import tessellate


def _name(label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return TCollection_AsciiString(attr.Get()).ToCString()
    return ""


def _matrix(loc: TopLoc_Location) -> np.ndarray:
    t = loc.Transformation()
    m = np.eye(4)
    for r in range(3):
        for c in range(4):
            m[r, c] = t.Value(r + 1, c + 1)
    return m


def _tessellate(shape, deflection: float, angle: float):
    v, f = tessellate(shape, deflection, angle)
    return v.astype(np.float32), f.astype(np.int32)


def convert(step_path, out_path, deflection=2.0, angle=0.5, log=print):
    t0 = time.time()
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
        raise RuntimeError(f"could not read {step_path}")
    reader.Transfer(doc)
    log(f"read STEP in {time.time() - t0:.0f} s")

    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    meshes: dict[str, int] = {}      # label entry -> mesh index
    arrays: dict[str, np.ndarray] = {}
    instances = []

    def mesh_for(label) -> int:
        entry = TCollection_AsciiString()
        TDF_Tool.Entry_s(label, entry)
        key = entry.ToCString()
        if key not in meshes:
            v, f = _tessellate(tool.GetShape_s(label), deflection, angle)
            meshes[key] = idx = len(meshes)
            arrays[f"v{idx}"], arrays[f"f{idx}"] = v, f
            if idx % 200 == 0:
                log(f"  meshed {idx} parts ({time.time() - t0:.0f} s)")
        return meshes[key]

    def walk(label, matrix, path):
        if tool.IsReference_s(label):
            ref = TDF_Label()
            tool.GetReferredShape_s(label, ref)
            walk(ref, matrix @ _matrix(tool.GetLocation_s(label)), path + [_name(ref) or _name(label)])
        elif tool.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            tool.GetComponents_s(label, comps, False)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), matrix, path)
        else:
            idx = mesh_for(label)
            if len(arrays[f"f{idx}"]):
                instances.append({"path": path, "mesh": idx, "matrix": matrix.round(6).tolist()})

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        root = roots.Value(i)
        walk(root, np.eye(4), [_name(root)])

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out.with_suffix(".npz"), **arrays)
    out.with_suffix(".json").write_text(json.dumps({"source": Path(step_path).name, "instances": instances}))
    tris = sum(len(a) for k, a in arrays.items() if k[0] == "f")
    log(f"{len(meshes)} unique parts, {len(instances)} instances, {tris:,} triangles, {time.time() - t0:.0f} s")


if __name__ == "__main__":
    convert(sys.argv[1], sys.argv[2], *(float(a) for a in sys.argv[3:5]))
