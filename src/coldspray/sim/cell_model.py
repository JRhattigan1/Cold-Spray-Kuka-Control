"""Cell CAD meshes grouped by what moves them, or by what can be hidden.

Built from the mesh cache written by coldspray.cad.step_cache:
  static    - everything fixed that is always shown, in WORLD
  groups    - fixed equipment that can be shown / hidden (see OPTIONAL), in WORLD
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
TILTING = ("__table_rotaryunit",)   # turntable body: moves with the tilt axis
ROTATING = ("__tabletop",)          # faceplate: moves with tilt and rotation
FIXED = ("__motorfortilt",)         # tilt drive sits on the fixed base frame
# label -> (assembly names in the cell CAD, shown by default, moves with: None / "rotating")
OPTIONAL = {
    "Booth walls": (("__enclosure",), False, None),
    "Table-top extensions": (("__tabletop_extensions",), False, "rotating"),
    "Rear cabinets / water cooling": (("__rear_electronics_and_water_cooling",), False, None),
    "Filtration unit": (("__filtration unit 2",), False, None),
    "Hoppers": (("__hoppers",), False, None),
    "Control arm": (("__LCD_screen",), False, None),
}


@dataclass
class CellMeshes:
    static: pv.PolyData
    carriage: pv.PolyData
    tilting: pv.PolyData = field(default_factory=pv.PolyData)
    rotating: pv.PolyData = field(default_factory=pv.PolyData)
    robot: list[pv.PolyData] = field(default_factory=list)
    groups: dict[str, pv.PolyData] = field(default_factory=dict)


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

    static, carriage, tilting, rotating = [], [], [], []
    optional = {label: [] for label in OPTIONAL}
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
        group = next((label for label, (assemblies, *_) in OPTIONAL.items()
                      if any(n in assemblies for n in names)), None)
        if any(n in CARRIAGE for n in names):
            carriage.append((world, f))
        elif any(n in FIXED for n in names):
            static.append((world, f))
        elif any(n in TILTING for n in names):
            tilting.append((world, f))
        elif any(n in ROTATING for n in names):
            rotating.append((world, f))
        elif group is not None:
            optional[group].append((world, f))
        else:
            static.append((world, f))

    missing = [n for n, k in ROBOT_PARTS.items() if not robot[k]]
    if missing:
        raise ValueError(f"robot parts not found in cell CAD: {missing}")
    return CellMeshes(
        static=_merge(static),
        carriage=_merge(carriage),
        tilting=_merge(tilting),
        rotating=_merge(rotating),
        robot=[_merge(parts) for parts in robot],
        groups={label: _merge(chunks) for label, chunks in optional.items() if chunks},
    )
