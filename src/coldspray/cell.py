"""Frames of the cell: robot root on the gantry, and the work base.

WORLD is the cell CAD coordinate system. The robot model works in its own
root frame, so a work base given in WORLD is converted with base_in_robot().
"""

import numpy as np

from coldspray.models import CellConfig
from coldspray.transforms import frame


def gantry_position(cell: CellConfig, e1: float | None = None) -> float:
    if cell.gantry is None:
        return 0.0
    e1 = cell.gantry.position_mm if e1 is None else e1
    lo, hi = cell.gantry.limits_mm
    if not lo <= e1 <= hi:
        raise ValueError(f"gantry position {e1} mm outside limits {lo}..{hi} mm")
    return e1


def robroot(cell: CellConfig, e1: float | None = None) -> np.ndarray:
    """Robot root in WORLD with the gantry at `e1` (default: configured position)."""
    root = frame(**cell.robroot.model_dump())
    if cell.gantry is not None:
        root[:3, 3] += np.asarray(cell.gantry.direction) * gantry_position(cell, e1)
    return root


def base_world(cell: CellConfig) -> np.ndarray:
    return frame(**cell.base.model_dump())


def base_in_robot(cell: CellConfig, e1: float | None = None) -> np.ndarray:
    return np.linalg.inv(robroot(cell, e1)) @ base_world(cell)
