"""Slice a part mesh into horizontal layers of 2D polygons."""

from dataclasses import dataclass

import numpy as np
import trimesh
from shapely.geometry import MultiPolygon, Polygon
from shapely.ops import unary_union


@dataclass
class Layer:
    index: int
    z_bottom: float          # surface the layer is sprayed onto
    z_top: float
    region: MultiPolygon     # area to deposit, in part XY (mm)


def place_on_plate(vertices: np.ndarray) -> np.ndarray:
    """Shift a part so its lowest point sits on z = 0 (the substrate)."""
    return vertices - [0.0, 0.0, vertices[:, 2].min()]


def slice_mesh(vertices, faces, layer_thickness: float) -> list[Layer]:
    """Cut the mesh at the middle of every layer from z = 0 up to its top."""
    mesh = trimesh.Trimesh(vertices, faces, process=True)
    top = vertices[:, 2].max()
    n = int(np.ceil(top / layer_thickness - 1e-9))
    layers = []
    for i in range(n):
        z0 = i * layer_thickness
        z_mid = min(z0 + layer_thickness / 2, top - 1e-6)
        section = mesh.section(plane_origin=(0, 0, z_mid), plane_normal=(0, 0, 1))
        if section is None:
            continue
        to_2d = np.eye(4)
        to_2d[2, 3] = -z_mid
        planar, _ = section.to_2D(to_2D=to_2d, check=False)
        region = unary_union([p for p in planar.polygons_full if p.area > 1e-6])
        if region.is_empty:
            continue
        if isinstance(region, Polygon):
            region = MultiPolygon([region])
        layers.append(Layer(i, z0, z0 + layer_thickness, region))
    return layers
