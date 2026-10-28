"""Infill pattern library: pure 2D stroke generators for one layer.

Each pattern is `fn(region, proc, layer_index, start=None) -> list[stroke]`,
where a stroke is an (n, 2) array sprayed from its first point to its last and
`start` is where the gun is when the pattern begins (strokes are ordered from
there). `region` is the area to fill, usually the layer region inset by the
contour passes. Gaps between strokes longer than 1.5 x step-over become lifted
jumps in `plan_layers`, so e.g. `oneway` gets its lifted return for free.

`PATTERNS` maps the config names to the functions.
"""

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Polygon, box

SIMPLIFY_MM = 0.05  # max deviation when thinning dense contour points
ISLAND_STEPS = 8    # islands: tile side = this x step-over


def _polygons(geom) -> list[Polygon]:
    if geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    return [g for g in getattr(geom, "geoms", []) if isinstance(g, Polygon)]


def contour_loops(region: MultiPolygon, proc) -> list[np.ndarray]:
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


def _norms(d: np.ndarray) -> np.ndarray:
    """Row lengths of (n, 2) `d`, bit-identical to np.linalg.norm per row (both use dot)."""
    return np.sqrt((d[:, None, :] @ d[:, :, None]).ravel())


def _order(segments: list[np.ndarray], start) -> list[np.ndarray]:
    """Greedy nearest-neighbour ordering, flipping segments as needed."""
    n = len(segments)
    if n == 0:
        return []
    firsts = np.array([s[0] for s in segments], float)
    lasts = np.array([s[-1] for s in segments], float)
    taken = np.zeros(n, bool)
    out, here = [], np.asarray(start, float)
    for _ in range(n):
        d0, d1 = _norms(firsts - here), _norms(lasts - here)
        d = np.where(taken, np.inf, np.minimum(d0, d1))
        j = int(np.argmin(d))
        taken[j] = True
        s = segments[j][::-1] if d1[j] < d0[j] else segments[j]
        out.append(s)
        here = s[-1]
    return out


def rotate_seam(loop: np.ndarray, layer_index: int, fraction: float = 0.382) -> np.ndarray:
    """Start a closed loop `layer_index * fraction` of the way round, so seams don't stack.

    The default step is the golden fraction, which spreads seams evenly over many layers."""
    pts = loop[:-1] if len(loop) > 2 and np.allclose(loop[0], loop[-1]) else loop
    if len(pts) < 2:
        return loop
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
    target = (layer_index * fraction % 1.0) * seg.sum()
    k = int(np.searchsorted(np.cumsum(seg), target, side="right")) % len(pts)
    rolled = np.roll(pts, -k, axis=0)
    return np.vstack([rolled, rolled[:1]])


def _angle(proc, layer_index: int) -> float:
    return (proc.raster_start_deg + layer_index * proc.raster_angle_step_deg) % 180


def _start(start):
    return np.zeros(2) if start is None else np.asarray(start, float)


def zigzag(region, proc, layer_index: int, start=None) -> list[np.ndarray]:
    """Raster lines, nearest-neighbour ordered so neighbours alternate direction (today's behaviour)."""
    return _order(raster_segments(region, proc.step_over_mm, _angle(proc, layer_index)), _start(start))


def oneway(region, proc, layer_index: int, start=None) -> list[np.ndarray]:
    """Raster lines all sprayed the same way; the return to each line start is a lifted jump."""
    out = []
    for s in zigzag(region, proc, layer_index, start):
        d = s[-1] - s[0]
        out.append(s if (d[0], d[1]) >= (0, 0) else s[::-1])  # lexicographic: one fixed direction
    return out


def concentric(region, proc, layer_index: int, start=None) -> list[np.ndarray]:
    """Inward offset loops, `step_over` apart, the first half a track inside the edge."""
    loops, k = [], 0
    while True:
        inset = region.buffer(-(proc.track_width_mm / 2 + k * proc.step_over_mm))
        polys = _polygons(inset.simplify(SIMPLIFY_MM))
        if not polys:
            # a region thinner than one track: spray its centre line once
            if k == 0 and not region.is_empty:
                loops += [np.asarray(p.exterior.coords) for p in _polygons(region.buffer(-1e-3))]
            break
        loops += [rotate_seam(np.asarray(r.coords), layer_index)
                  for p in polys for r in [p.exterior, *p.interiors]]
        k += 1
    return _order_loops(loops, _start(start))


def _order_loops(loops: list[np.ndarray], here) -> list[np.ndarray]:
    """Keep loop order (outside in) but start each loop at its vertex nearest the gun."""
    out = []
    for loop in loops:
        pts = loop[:-1]
        k = int(np.argmin(np.linalg.norm(pts - here, axis=1)))
        rolled = np.roll(pts, -k, axis=0)
        out.append(np.vstack([rolled, rolled[:1]]))
        here = out[-1][-1]
    return out


def spiral(region, proc, layer_index: int, start=None) -> list[np.ndarray]:
    """Concentric loops joined into continuous paths: each loop leaves its ring one step-over
    before closing it and heads straight for the next one when that is within 1.5 x step-over,
    so the path spirals in and the gun never lifts inside one island."""
    out = []
    for loop in concentric(region, proc, layer_index, start):
        if out and len(out[-1]) > 2 and np.linalg.norm(loop[0] - out[-1][-1]) <= 1.5 * proc.step_over_mm:
            prev, end = out[-1][-2], out[-1][-1]
            seg = np.linalg.norm(end - prev)
            cut = end - (end - prev) * min(proc.step_over_mm, seg / 2) / seg if seg > 0 else end
            if region.buffer(1e-6).contains(LineString([cut, loop[0]])):
                out[-1] = np.vstack([out[-1][:-1], cut, loop])  # leave the ring early: step in diagonally
                continue
            out.append(loop)
        else:
            out.append(loop)
    return out


def crosshatch(region, proc, layer_index: int, start=None) -> list[np.ndarray]:
    """Alternate perpendicular full-density rasters layer by layer (0/90, then rotated
    by `raster_angle_step_deg` every pair of layers)."""
    angle = (proc.raster_start_deg + 90 * (layer_index % 2)
             + (layer_index // 2) * proc.raster_angle_step_deg) % 180
    return _order(raster_segments(region, proc.step_over_mm, angle), _start(start))


def islands(region, proc, layer_index: int, start=None) -> list[np.ndarray]:
    """Checkerboard tiles, neighbouring tiles rastered at 90 degrees to each other, to
    break up long tracks and reduce residual stress. The board shifts half a tile per layer."""
    if region.is_empty:
        return []
    tile = ISLAND_STEPS * proc.step_over_mm
    shift = (layer_index % 2) * tile / 2
    xmin, ymin, xmax, ymax = region.bounds
    here, out = _start(start), []
    xs = np.arange(xmin - shift, xmax, tile)
    ys = np.arange(ymin - shift, ymax, tile)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs if j % 2 == 0 else xs[::-1]):  # boustrophedon over the tiles
            ii = i if j % 2 == 0 else len(xs) - 1 - i
            piece = region.intersection(box(x, y, x + tile, y + tile))
            angle = (_angle(proc, layer_index) + 90 * ((ii + j) % 2)) % 180
            strokes = _order(raster_segments(piece, proc.step_over_mm, angle), here)
            if strokes:
                out += strokes
                here = strokes[-1][-1]
    return out


PATTERNS = {
    "zigzag": zigzag,
    "oneway": oneway,
    "concentric": concentric,
    "spiral": spiral,
    "crosshatch": crosshatch,
    "islands": islands,
}
