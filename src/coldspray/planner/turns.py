"""G1: move infill turning points off the component (overspray masking).

`extend_strokes` lengthens each straight raster stroke at both ends so the gun
turns round off the part, with the spray still on:
- "overrun": `overrun_mm` past each end, along the stroke direction;
- "beyond_plane": until the stroke crosses `plane` (point, normal; only its XY
  trace in the layer frame is used), plus `overrun_mm`. An end whose direction
  does not head towards the plane falls back to plain overrun. Ends on a hole
edge are not extended (a zero-length "turn" segment is kept so strokes stay 4 points).
"inside" returns the strokes unchanged.
"""

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from coldspray.sim.datums import datum_plane

RASTER_PATTERNS = ("zigzag", "oneway", "crosshatch", "islands")


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


def extend_strokes(strokes, region, proc, plane=None, z=0.0):
    mode = proc.turn_mode
    if mode == "inside":
        return strokes
    o = proc.overrun_mm
    # ends at a hole edge stay put: extending them would spray across the hole onto the part
    filled = unary_union([Polygon(g.exterior) for g in getattr(region, "geoms", [region])]).buffer(1e-3)
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
            ends.append(end + d * ((t or 0.0) + o))
        out.append(np.vstack([ends[0], s[0], s[-1], ends[1]]))  # always 4 points: 1 and 3 are the turns
    return out
