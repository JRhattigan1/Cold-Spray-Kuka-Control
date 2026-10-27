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

from coldspray.cad.slicer import Layer
from coldspray.models import ProcessConfig
from coldspray.planner.patterns import (  # noqa: F401  (re-exported for old imports)
    SIMPLIFY_MM, _order, _polygons, contour_loops, raster_segments)
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
        angle = (layer.index * proc.raster_angle_step_deg) % 180
        infill_area = layer.region.buffer(-proc.contour_passes * proc.step_over_mm)
        strokes = contour_loops(layer.region, proc)
        n_contour = len(strokes)
        start = strokes[0][0] if strokes else np.zeros(2)
        strokes += _order(raster_segments(infill_area, proc.step_over_mm, angle), start)
        if not strokes:
            continue

        first = strokes[0][0]
        kind = "PTP" if not path.moves else "LIN"
        path.moves.append(Move(at(first, z + proc.approach_height_mm), proc.rapid_speed_mm_s, kind=kind,
                               axes=dict(table_axes or {}) if kind == "PTP" else {}, tag="approach"))
        path.moves.append(Move(at(first, z), proc.rapid_speed_mm_s, tag="approach"))
        for k, stroke in enumerate(strokes):
            here = path.moves[-1].pose[:2, 3]
            if np.linalg.norm(stroke[0] - here) > 1.5 * proc.step_over_mm:
                up = z + proc.jump_height_mm
                for p, h in ((here, up), (stroke[0], up), (stroke[0], z)):
                    path.moves.append(Move(at(p, h), proc.rapid_speed_mm_s, gun_on=True, tag="jump"))
            tag = "contour" if k < n_contour else "infill"
            for i, p in enumerate(stroke):
                if np.allclose(path.moves[-1].pose[:2, 3], p):
                    continue                                    # the join to a stroke's start is a turn
                path.moves.append(Move(at(p, z), proc.traverse_speed_mm_s, gun_on=True,
                                       tag="turn" if i == 0 else tag))
        last = strokes[-1][-1]
        path.moves.append(Move(at(last, z + proc.approach_height_mm), proc.rapid_speed_mm_s, tag="approach"))
    return path
