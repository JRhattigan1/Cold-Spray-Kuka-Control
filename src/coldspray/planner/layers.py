"""Turn sliced layers into spray passes (contours + raster infill).

Per layer: `contour_passes` perimeter loops, then raster lines filling the
rest at `step_over_mm` spacing, rotated by `raster_angle_step_deg` each layer.
Nozzle height is the top of the previous layer + standoff. The spray stays on
through each layer and is switched off for the move to the next layer.

v1 limitations: raster segments are joined by straight moves (which can cross
holes or concave notches), and the gun always points straight down.
"""

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Polygon

from coldspray.cad.slicer import Layer
from coldspray.models import ProcessConfig
from coldspray.toolpath.model import Move, Toolpath, spray_frame

DOWN = (0.0, 0.0, -1.0)
ALONG_X = (1.0, 0.0, 0.0)


def _polygons(geom) -> list[Polygon]:
    if geom.is_empty:
        return []
    return list(geom.geoms) if isinstance(geom, MultiPolygon) else [geom]


def contour_loops(region: MultiPolygon, proc: ProcessConfig) -> list[np.ndarray]:
    """Perimeter loops, the first one half a track inside the part edge."""
    loops = []
    for k in range(proc.contour_passes):
        inset = region.buffer(-(proc.track_width_mm / 2 + k * proc.step_over_mm))
        for poly in _polygons(inset):
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


def plan_layers(layers: list[Layer], proc: ProcessConfig, name="part") -> Toolpath:
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
        path.moves.append(Move(at(first, z + proc.approach_height_mm), proc.rapid_speed_mm_s, kind=kind))
        path.moves.append(Move(at(first, z), proc.rapid_speed_mm_s))
        for stroke in strokes:
            for p in stroke:
                path.moves.append(Move(at(p, z), proc.traverse_speed_mm_s, gun_on=True))
        last = strokes[-1][-1]
        path.moves.append(Move(at(last, z + proc.approach_height_mm), proc.rapid_speed_mm_s))
    return path
