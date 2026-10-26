"""Turn sliced layers into spray passes (contours + raster infill).

Per layer: `contour_passes` perimeter loops, then raster lines filling the
rest at `step_over_mm` spacing, rotated by `raster_angle_step_deg` each layer.
Nozzle height is the top of the previous layer + standoff. The spray stays on
through each layer (powder feed can't switch quickly) and is switched off for
the move to the next layer.

Neighbouring strokes (gap up to 1.5 x step-over) are joined at spray height.
Longer gaps are "jumps": the gun lifts by `jump_height_mm` and moves at rapid
speed, so little powder deposits on the way across holes or between islands.

v1 limitation: the gun always points straight down.
"""

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Polygon

from coldspray.cad.slicer import Layer
from coldspray.models import ProcessConfig
from coldspray.toolpath.model import Move, Toolpath, spray_frame

DOWN = (0.0, 0.0, -1.0)
ALONG_X = (1.0, 0.0, 0.0)
SIMPLIFY_MM = 0.05  # max deviation when thinning dense contour points


def _polygons(geom) -> list[Polygon]:
    if geom.is_empty:
        return []
    return list(geom.geoms) if isinstance(geom, MultiPolygon) else [geom]


def contour_loops(region: MultiPolygon, proc: ProcessConfig) -> list[np.ndarray]:
    """Perimeter loops, the first one half a track inside the part edge."""
    loops = []
    for k in range(proc.contour_passes):
        inset = region.buffer(-(proc.track_width_mm / 2 + k * proc.step_over_mm))
        for poly in _polygons(inset.simplify(SIMPLIFY_MM)):
            for ring in [poly.exterior, *poly.interiors]:
                loops.append(np.asarray(ring.coords))
    return loops


def raster_segments(region, step: float, angle_deg: float) -> list[np.ndarray]:
    """Parallel lines `step` apart across `region` at `angle_deg`, clipped to it."""
    if region.is_empty:
        return []
    rotated = affinity.rotate(region, -angle_deg, origin=(0, 0))
    xmin, ymin, xmax, ymax = rotated.bounds
    segments = []
    for y in np.arange(ymin + step / 2, ymax, step):
        cut = rotated.intersection(LineString([(xmin - 1, y), (xmax + 1, y)]))
        parts = getattr(cut, "geoms", [cut])
        for seg in parts:
            if seg.length > 1e-6:
                back = affinity.rotate(seg, angle_deg, origin=(0, 0))
                segments.append(np.asarray(back.coords)[[0, -1]])
    return segments


def _order(segments: list[np.ndarray], start) -> list[np.ndarray]:
    """Greedy nearest-neighbour ordering, flipping segments as needed."""
    remaining = list(segments)
    out, here = [], np.asarray(start, float)
    while remaining:
        d = [min(np.linalg.norm(s[0] - here), np.linalg.norm(s[-1] - here)) for s in remaining]
        s = remaining.pop(int(np.argmin(d)))
        if np.linalg.norm(s[-1] - here) < np.linalg.norm(s[0] - here):
            s = s[::-1]
        out.append(s)
        here = s[-1]
    return out


def plan_layers(layers: list[Layer], proc: ProcessConfig, name="part", table_axes: dict | None = None) -> Toolpath:
    """`table_axes` fixes the turntable for the whole program, e.g. {"E2": 30, "E5": 0};
    the gun then sprays along the (tilted) faceplate normal."""
    path = Toolpath(name)

    def at(xy, z):
        return spray_frame((xy[0], xy[1], z), DOWN, ALONG_X)

    for layer in layers:
        z = layer.z_bottom + proc.standoff_mm
        angle = (layer.index * proc.raster_angle_step_deg) % 180
        infill_area = layer.region.buffer(-proc.contour_passes * proc.step_over_mm)
        strokes = contour_loops(layer.region, proc)
        start = strokes[0][0] if strokes else np.zeros(2)
        strokes += _order(raster_segments(infill_area, proc.step_over_mm, angle), start)
        if not strokes:
            continue

        first = strokes[0][0]
        kind = "PTP" if not path.moves else "LIN"
        path.moves.append(Move(at(first, z + proc.approach_height_mm), proc.rapid_speed_mm_s, kind=kind,
                               axes=dict(table_axes or {}) if kind == "PTP" else {}))
        path.moves.append(Move(at(first, z), proc.rapid_speed_mm_s))
        for stroke in strokes:
            here = path.moves[-1].pose[:2, 3]
            if np.linalg.norm(stroke[0] - here) > 1.5 * proc.step_over_mm:
                up = z + proc.jump_height_mm
                for p, h in ((here, up), (stroke[0], up), (stroke[0], z)):
                    path.moves.append(Move(at(p, h), proc.rapid_speed_mm_s, gun_on=True))
            for p in stroke:
                if np.allclose(path.moves[-1].pose[:2, 3], p):
                    continue
                path.moves.append(Move(at(p, z), proc.traverse_speed_mm_s, gun_on=True))
        last = strokes[-1][-1]
        path.moves.append(Move(at(last, z + proc.approach_height_mm), proc.rapid_speed_mm_s))
    return path
