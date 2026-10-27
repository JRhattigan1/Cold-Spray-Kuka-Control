"""Tilted nozzle poses for contour (boundary) passes.

With `contour_tilt_deg > 0` the gun on a perimeter loop leans outward, towards
the nearest part edge, so the spray hits the edge wall and builds it square.
The aim point stays on the loop (half a track inside the edge) at the layer
surface, and the nozzle sits back along the tilted spray axis at the standoff.

Outward normals come from the ring orientation; the region decides which side
is "outward" (away from material), so it works for exteriors and holes alike.
They are smoothed over about one track width so corners don't flip the wrist.
"""

import numpy as np
import shapely

from coldspray.toolpath.model import spray_frame

DOWN = np.array([0.0, 0.0, -1.0])
ALONG_X = (1.0, 0.0, 0.0)


def densify_corners(ring: np.ndarray, width: float) -> np.ndarray:
    """Closed ring (first == last) with extra points `width/2` either side of
    each vertex, so the smoothed normal turns only near the corners."""
    pts = np.asarray(ring, float)[:, :2]
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        L = np.linalg.norm(b - a)
        if L > 2 * width:
            u = (b - a) / L
            out += [a + u * width / 2, b - u * width / 2]
        out.append(b)
    return np.array(out)


def outward_normals(ring: np.ndarray, region, width: float) -> np.ndarray:
    """Unit horizontal normals at each ring vertex pointing away from material
    (out of the part for exteriors, into the hole for holes), averaged over
    edges within `width/2` of arc length either side of the vertex."""
    pts = np.asarray(ring, float)[:, :2]
    d = np.diff(pts, axis=0)
    L = np.linalg.norm(d, axis=1)
    ok = L > 1e-9
    d, L = d[ok], L[ok]
    n_edge = np.column_stack([d[:, 1], -d[:, 0]]) / L[:, None]  # right-hand side of travel
    s_mid = np.cumsum(L) - L / 2
    s_vert = _arc(pts)
    total = L.sum()
    gap = np.abs(s_vert[:, None] - s_mid[None, :])
    gap = np.minimum(gap, total - gap)                       # closed ring: wrap around
    w = (gap < width / 2 + L[None, :] / 2 - 1e-9) * L[None, :]
    n = w @ n_edge
    n /= np.linalg.norm(n, axis=1, keepdims=True).clip(1e-12)
    # Which side is outward? Stepping outward gets closer to the region edge.
    step = 0.25 * width
    edge = region.boundary
    closer = shapely.distance(edge, shapely.points(pts + step * n)) < shapely.distance(edge, shapely.points(pts - step * n))
    return n if closer.mean() >= 0.5 else -n


def _arc(pts: np.ndarray) -> np.ndarray:
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])


def tilted_ring_poses(ring: np.ndarray, region, z_aim: float, proc, tilt_deg: float):
    """(points, poses): the densified ring xy and a 4x4 TCP pose per point.
    Spray direction = DOWN rotated outward by `tilt_deg`; the nozzle is
    `standoff_mm` back along it from the aim point at height `z_aim`."""
    pts = densify_corners(ring, proc.track_width_mm)
    n = outward_normals(pts, region, proc.track_width_mm)
    t = np.radians(tilt_deg)
    poses = []
    for p, ni in zip(pts, n):
        spray = np.cos(t) * DOWN + np.sin(t) * np.array([ni[0], ni[1], 0.0])
        aim = np.array([p[0], p[1], z_aim])
        poses.append(spray_frame(aim - proc.standoff_mm * spray, spray, ALONG_X))
    return pts, poses
