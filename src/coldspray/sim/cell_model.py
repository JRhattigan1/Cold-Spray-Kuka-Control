"""Cell CAD meshes grouped by what moves them.

Built from the mesh cache written by coldspray.cad.step_cache:
  static    - everything fixed, in WORLD
  enclosure - booth walls (drawn see-through so the cell is visible)
  carriage  - gantry carriage parts, in WORLD with the gantry at 0
  robot     - 7 meshes (base, A1..A6) in the robot root frame, at the CAD pose
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pyvista as pv

from coldspray.models import PROJECT_ROOT

ROBOT_PARTS = {
    "robot base": 0,
    "robot rotating column A1": 1,
    "robot linkarm A2": 2,
    "robot elbow A3": 3,
    "robot arm A4": 4,
    "robot hand A5": 5,
    "robot twist tool A6": 6,
}
CARRIAGE = ("__gantry_moveplate", "__mounting_plates_for_KUKA_robot_base")
ENCLOSURE = ("__enclosure",)


@dataclass
class CellMeshes:
    static: pv.PolyData
    enclosure: pv.PolyData
    carriage: pv.PolyData
    robot: list[pv.PolyData] = field(default_factory=list)


def _merge(chunks) -> pv.PolyData:
    if not chunks:
        return pv.PolyData()
    verts, faces, offset = [], [], 0
    for v, f in chunks:
        verts.append(v)
        faces.append(f + offset)
        offset += len(v)
    f = np.vstack(faces)
    return pv.PolyData(np.vstack(verts), np.hstack([np.full((len(f), 1), 3), f]).ravel())


def load_cell_meshes(cache: str) -> CellMeshes:
    cache = Path(cache)
    if not cache.is_absolute():
        cache = PROJECT_ROOT / cache
    index = json.loads(cache.with_suffix(".json").read_text())
    data = np.load(cache.with_suffix(".npz"))

    groups = {"static": [], "enclosure": [], "carriage": []}
    robot: list[list] = [[] for _ in ROBOT_PARTS]
    for inst in index["instances"]:
        v, f = data[f"v{inst['mesh']}"].astype(float), data[f"f{inst['mesh']}"]
        m = np.array(inst["matrix"])
        names = inst["path"]
        link = next((ROBOT_PARTS[n] for n in names if n in ROBOT_PARTS), None)
        if link is not None:
            robot[link].append((v, f))  # part coordinates are the robot root frame
            continue
        world = v @ m[:3, :3].T + m[:3, 3]
        if any(n in CARRIAGE for n in names):
            groups["carriage"].append((world, f))
        elif any(n in ENCLOSURE for n in names):
            groups["enclosure"].append((world, f))
        else:
            groups["static"].append((world, f))

    missing = [n for n, k in ROBOT_PARTS.items() if not robot[k]]
    if missing:
        raise ValueError(f"robot parts not found in cell CAD: {missing}")
    return CellMeshes(
        static=_merge(groups["static"]),
        enclosure=_merge(groups["enclosure"]),
        carriage=_merge(groups["carriage"]),
        robot=[_merge(parts) for parts in robot],
    )
