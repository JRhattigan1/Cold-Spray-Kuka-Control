"""G1: move infill turning points off the component (overspray masking).

`extend_strokes` lengthens each straight raster stroke at both ends so the gun
turns round off the part, with the spray still on:
- "overrun": `overrun_mm` past the part's edge (`body`, else the stroke end), along the stroke;
- "beyond_plane": until the stroke crosses `plane` (point, normal; only its XY
  trace in the layer frame is used), plus `overrun_mm`. An end whose direction
  does not head towards the plane falls back to plain overrun. Ends on a hole
edge are not extended (a zero-length "turn" segment is kept so strokes stay 4 points).
"inside" returns the strokes unchanged.
"""

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from coldspray.sim.datums import datum_plane

RASTER_PATTERNS = ("zigzag", "oneway", "islands")


def parse_plane(spec: str):
    """"YZ", "datum:YZ" or "YZ:60" (offset in mm) -> (point, normal), or None if empty/unknown."""
    if not spec:
        return None
    name, offset = spec, 0.0
    head, _, tail = spec.rpartition(":")
    try:
        offset, name = float(tail), head
    except ValueError:
        pass
    try:
        return datum_plane(name, offset)
    except (KeyError, ValueError):
        return None


def _reach(end, u, plane, z):
    """Distance along u from `end` to the plane's trace at height z, or None if not ahead."""
    p, n = (np.asarray(a, float) for a in plane)
    nu = n[0] * u[0] + n[1] * u[1]
    if abs(nu) < 1e-9:
        return None
    t = (n @ p - n[2] * z - n[0] * end[0] - n[1] * end[1]) / nu
    return t if t > 0 else None


def _to_body_edge(end, d, body, far) -> float:
    """Distance along d from `end` to where the ray leaves `body` (0 if it starts outside)."""
    hit = body.intersection(LineString([end, end + d * far]))
    pieces = getattr(hit, "geoms", [hit])
    return max((p.length for p in pieces if p.length > 0 and p.distance(Point(end)) < 1e-6), default=0.0)


def extend_strokes(strokes, region, proc, plane=None, z=0.0, body=None):
    """`region`: the area the strokes fill (often inset from the part by contour passes).
    `body`: the part's own outline; turns go `overrun_mm` past its edge, not past `region`'s."""
    mode = proc.turn_mode
    if mode == "inside":
        return strokes
    o = proc.overrun_mm
    # ends at a hole edge stay put: extending them would spray across the hole onto the part
    filled = unary_union([Polygon(g.exterior) for g in getattr(region, "geoms", [region])]).buffer(1e-3)
    if body is not None:
        body = unary_union([Polygon(g.exterior) for g in getattr(body, "geoms", [body])])
        x0, y0, x1, y1 = body.bounds
        far = 2 * np.hypot(x1 - x0, y1 - y0) + 1.0
    out = []
    for s in strokes:
        s = np.asarray(s, float)
        u = s[-1] - s[0]
        length = np.linalg.norm(u)
        if len(s) != 2 or length < 1e-9:
            out.append(s)
            continue
        u = u / length
        ends = []
        for end, d in ((s[0], -u), (s[-1], u)):
            if filled.contains(Point(end + d * 2e-3)):
                ends.append(end)
                continue
            t = _reach(end, d, plane, z) if mode == "beyond_plane" and plane is not None else None
            edge = _to_body_edge(end, d, body, far) if body is not None else 0.0
            ends.append(end + d * (max(t or 0.0, edge) + o))
        out.append(np.vstack([ends[0], s[0], s[-1], ends[1]]))  # always 4 points: 1 and 3 are the turns
    return out
