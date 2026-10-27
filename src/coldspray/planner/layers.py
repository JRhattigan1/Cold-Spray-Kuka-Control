"""Turn sliced layers into spray passes (contours + raster infill).

Per layer: `contour_passes` perimeter loops, then raster lines filling the
rest at `step_over_mm` spacing, rotated by `raster_angle_step_deg` each layer.
Nozzle height is the top of the previous layer + standoff. The spray stays on
through each layer (powder feed can't switch quickly) and is switched off for
the move to the next layer.

Neighbouring strokes (gap up to 1.5 x step-over) are joined at spray height.
Longer gaps are "jumps": the gun lifts by `jump_height_mm` and moves at rapid
speed, so little powder deposits on the way across holes or between islands.

The gun points straight down, except contour passes with `contour_tilt_deg > 0`
(see contour_tilt.py).
"""

import numpy as np

from coldspray.cad.slicer import Layer
from coldspray.models import ProcessConfig
from coldspray.planner.contour_tilt import tilted_ring_poses
from coldspray.planner.turns import RASTER_PATTERNS, extend_strokes, parse_plane
from coldspray.planner.patterns import (  # noqa: F401  (re-exported for old imports)
    PATTERNS, SIMPLIFY_MM, _order, _polygons, contour_loops, raster_segments)
from coldspray.toolpath.model import Move, Toolpath, spray_frame

DOWN = (0.0, 0.0, -1.0)
ALONG_X = (1.0, 0.0, 0.0)


def plan_layers(layers: list[Layer], proc: ProcessConfig, name="part", table_axes: dict | None = None) -> Toolpath:
    """`table_axes` fixes the turntable for the whole program, e.g. {"E2": 30, "E5": 0};
    the gun then sprays along the (tilted) faceplate normal."""
    path = Toolpath(name)

    def at(xy, z):
        return spray_frame((xy[0], xy[1], z), DOWN, ALONG_X)

    for layer in layers:
        z = layer.z_bottom + proc.standoff_mm
        infill_area = layer.region.buffer(-proc.contour_passes * proc.step_over_mm)
        strokes = contour_loops(layer.region, proc)
        n_contour = len(strokes)
        start = strokes[0][0] if strokes else np.zeros(2)
        pattern = PATTERNS[getattr(proc, "infill_pattern", "zigzag")]
        infill = pattern(infill_area, proc, layer.index, start=start)
        extended = (getattr(proc, "turn_mode", "inside") != "inside"
                    and getattr(proc, "infill_pattern", "zigzag") in RASTER_PATTERNS)
        if extended:
            infill = extend_strokes(infill, infill_area, proc, parse_plane(getattr(proc, "turn_plane", "")))
        strokes += infill
        if not strokes:
            continue
        tilted = {}                                             # stroke index -> poses at height z
        if getattr(proc, "contour_tilt_deg", 0) > 0:
            for k in range(n_contour):
                strokes[k], tilted[k] = tilted_ring_poses(strokes[k], layer.region, layer.z_bottom, proc,
                                                          proc.contour_tilt_deg)

        def pose(k, i, h):
            if k not in tilted:
                return at(strokes[k][i], h)
            t = tilted[k][i].copy()
            t[2, 3] += h - z
            return t

        first = strokes[0][0]
        kind = "PTP" if not path.moves else "LIN"
        path.moves.append(Move(pose(0, 0, z + proc.approach_height_mm), proc.rapid_speed_mm_s, kind=kind,
                               axes=dict(table_axes or {}) if kind == "PTP" else {}, tag="approach"))
        path.moves.append(Move(pose(0, 0, z), proc.rapid_speed_mm_s, tag="approach"))
        here, last_ki = first, (0, 0)                           # aim point of the last move
        for k, stroke in enumerate(strokes):
            if np.linalg.norm(stroke[0] - here) > 1.5 * proc.step_over_mm:
                up = z + proc.jump_height_mm
                for ki, h in ((last_ki, up), ((k, 0), up), ((k, 0), z)):
                    path.moves.append(Move(pose(*ki, h), proc.rapid_speed_mm_s, gun_on=True, tag="jump"))
                here, last_ki = stroke[0], (k, 0)
            tag = "contour" if k < n_contour else "infill"
            for i, p in enumerate(stroke):
                if np.allclose(here, p):
                    continue                                    # the join to a stroke's start is a turn
                path.moves.append(Move(pose(k, i, z), proc.traverse_speed_mm_s, gun_on=True,
                                       tag="turn" if i == 0 or (extended and k >= n_contour
                                                                and len(stroke) == 4 and i in (1, 3)) else tag))
                here, last_ki = p, (k, i)
        path.moves.append(Move(pose(len(strokes) - 1, -1, z + proc.approach_height_mm), proc.rapid_speed_mm_s, tag="approach"))
    return path
