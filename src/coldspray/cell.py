"""Frames of the cell: robot root on the gantry, turntable, and the work base.

WORLD is the cell CAD coordinate system. The robot model works in its own
root frame, so a work base given in WORLD is converted with base_in_robot().

External axis values are passed around as a dict, e.g. {"E2": 30.0, "E5": -90.0};
axes not in the dict take their configured position.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from coldspray.models import CellConfig, RotaryAxisConfig
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


def rotation_about(axis: RotaryAxisConfig, angle_deg: float) -> np.ndarray:
    """4x4 rotation by `angle_deg` about a rotary axis (WORLD, all axes at 0)."""
    d = np.asarray(axis.direction, float)
    m = np.eye(4)
    m[:3, :3] = Rotation.from_rotvec(np.radians(angle_deg) * d / np.linalg.norm(d)).as_matrix()
    p = np.asarray(axis.point, float)
    m[:3, 3] = p - m[:3, :3] @ p
    return m


def turntable_axes(cell: CellConfig, ext: dict | None = None) -> tuple[float, float]:
    """(tilt, rotation) in degrees: from `ext` where given, else the configured positions."""
    tt = cell.turntable
    if tt is None:
        return 0.0, 0.0
    ext = ext or {}
    tilt = ext.get(tt.tilt.axis, tt.tilt.position_deg)
    rot = ext.get(tt.rotation.axis, tt.rotation.position_deg)
    for ax, v in ((tt.tilt, tilt), (tt.rotation, rot)):
        if ax.endless:
            continue
        lo, hi = ax.limits_deg
        if not lo - 1e-9 <= v <= hi + 1e-9:
            raise ValueError(f"{ax.axis} = {v:.2f} deg outside limits {lo}..{hi} deg")
    return tilt, rot


def turntable_motion(cell: CellConfig, ext: dict | None = None) -> tuple[np.ndarray, np.ndarray]:
    """WORLD transforms of (tilting body, faceplate) relative to their CAD pose."""
    if cell.turntable is None:
        return np.eye(4), np.eye(4)
    tilt, rot = turntable_axes(cell, ext)
    t = rotation_about(cell.turntable.tilt, tilt)
    return t, t @ rotation_about(cell.turntable.rotation, rot)


def base_world(cell: CellConfig, ext: dict | None = None) -> np.ndarray:
    """Work base (faceplate) in WORLD with the turntable at `ext`."""
    return turntable_motion(cell, ext)[1] @ frame(**cell.base.model_dump())


def base_worlds(cell: CellConfig, axes_list) -> np.ndarray:
    """base_world for every axes dict in `axes_list`: (n, 4, 4). Computed once per
    distinct table position (a job's table rarely moves); other axes (E1) don't matter."""
    cache, out = {}, np.empty((len(axes_list), 4, 4))
    for i, axes in enumerate(axes_list):
        key = turntable_axes(cell, axes)
        if key not in cache:
            cache[key] = base_world(cell, axes)
        out[i] = cache[key]
    return out


def base_in_robot(cell: CellConfig, ext: dict | None = None, e1: float | None = None) -> np.ndarray:
    return np.linalg.inv(robroot(cell, e1)) @ base_world(cell, ext)


def table_axis_names(cell: CellConfig) -> list[str]:
    tt = cell.turntable
    return [tt.tilt.axis, tt.rotation.axis] if tt else []


def axis_defaults(cell: CellConfig) -> dict[str, float]:
    """External axis values before a path first sets them, as the KRL has them
    (post/krl.py): the cell's gantry and turntable positions. For Toolpath.sample(initial=)."""
    out = {}
    if cell.gantry is not None:
        out[cell.gantry.axis] = float(cell.gantry.position_mm)
    if cell.turntable is not None:
        tt = cell.turntable
        out |= {tt.tilt.axis: float(tt.tilt.position_deg), tt.rotation.axis: float(tt.rotation.position_deg)}
    return out
