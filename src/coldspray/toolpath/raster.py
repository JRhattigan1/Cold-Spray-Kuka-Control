"""Simple raster over a flat rectangle - the Phase 1 test path."""

import numpy as np

from coldspray.models import ProcessConfig
from coldspray.toolpath.model import Move, Toolpath, spray_frame

DOWN = (0.0, 0.0, -1.0)
ALONG_X = (1.0, 0.0, 0.0)


def plate_raster(proc: ProcessConfig, width: float, length: float, name="plate_raster") -> Toolpath:
    """Zig-zag passes along X over a `width` x `length` plate whose corner is
    the BASE origin and top surface is BASE z = 0. Spray stays on through the
    turnarounds, which sit `overrun_mm` beyond the plate edges.
    """
    z = proc.standoff_mm
    x0, x1 = -proc.overrun_mm, width + proc.overrun_mm
    ys = np.arange(0.0, length + 1e-9, proc.step_over_mm)

    def at(x, y, dz=0.0):
        return spray_frame((x, y, z + dz), DOWN, ALONG_X)

    moves = [
        Move(at(x0, ys[0], proc.approach_height_mm), proc.rapid_speed_mm_s, kind="PTP"),
        Move(at(x0, ys[0]), proc.rapid_speed_mm_s),
    ]
    x_end = x0
    for i, y in enumerate(ys):
        xa, xb = (x0, x1) if i % 2 == 0 else (x1, x0)
        if i > 0:
            moves.append(Move(at(xa, y), proc.traverse_speed_mm_s, gun_on=True))
        moves.append(Move(at(xb, y), proc.traverse_speed_mm_s, gun_on=True))
        x_end = xb
    moves.append(Move(at(x_end, ys[-1], proc.approach_height_mm), proc.rapid_speed_mm_s))
    return Toolpath(name, moves)
