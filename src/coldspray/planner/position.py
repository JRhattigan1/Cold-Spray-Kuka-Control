"""Table tilt, gun roll and gantry position for a job: gun as level as allowed,
whole path reachable.

The gun must not point far down (bad for the equipment), so Layers builds are
tilted towards the gun: at tilt t the gun points (90 - t) degrees below
horizontal. Turning the gun about its own spray axis ("roll") doesn't change
the spray but decides whether the wrist can reach, so it is searched together
with the gantry position E1.
"""

from dataclasses import dataclass

import numpy as np

from coldspray.cell import base_world, robroot, table_axis_names
from coldspray.models import CellConfig
from coldspray.transforms import frame

ROLLS = tuple(range(-180, 180, 45))
COARSE_POINTS = 150      # path points checked per candidate during the search
FINE_POINTS = 1500       # ... and for the final check (the viewer checks every sample after Generate)


@dataclass
class Position:
    tilt_deg: float
    roll_deg: float
    e1_mm: float | None      # None: no gantry
    reach: float             # fraction of the path's move end points reachable


def gun_down_deg(world_poses) -> np.ndarray:
    """Spray direction (tool X) below horizontal in degrees, per WORLD pose (negative = upwards)."""
    return np.degrees(np.arcsin(np.clip(-np.asarray(world_poses)[:, 2, 0], -1.0, 1.0)))


def layer_tilts(cell: CellConfig, gun_dir=(0.0, 0.0, -1.0), rotation_deg=0.0, step=15.0) -> list[float]:
    """Table tilts to try for Layers, most level gun first, keeping the gun
    within the angle limit. `gun_dir`: spray direction in BASE (default:
    straight down onto the faceplate)."""
    if not cell.turntable:
        return [0.0]
    lo, hi = cell.turntable.tilt.limits_deg
    tilt_axis, rot_axis = table_axis_names(cell)
    tilts = sorted({*np.arange(lo, hi + 1e-9, step), hi})

    def down(t):
        d = base_world(cell, {tilt_axis: t, rot_axis: rotation_deg})[:3, :3] @ np.asarray(gun_dir, float)
        return float(np.degrees(np.arcsin(np.clip(-d[2], -1.0, 1.0))))

    level = sorted(tilts, key=lambda t: (round(abs(down(t)), 6), t))
    allowed = [float(t) for t in level if down(t) <= cell.gun_max_down_deg + 1e-6]
    return allowed or [float(level[0])]


def world_poses(path, cell: CellConfig) -> np.ndarray:
    """TCP of every move end point in WORLD, with the table where that move puts it."""
    return np.array([base_world(cell, axes) @ m.pose for m, axes in zip(path.moves, path.axis_values())])


def _spread(poses, n) -> np.ndarray:
    return poses[np.linspace(0, len(poses) - 1, min(n, len(poses))).astype(int)]


def _reaches_all(robot, poses, seed) -> bool:
    for p in poses:
        sols = robot.ik(p, seed)
        if not sols:
            return False
        seed = sols[0]
    return True


def _margin(ok, r, e) -> int:
    """Reachable neighbours of grid cell (roll r, E1 e); rolls wrap round, E1 doesn't."""
    n_roll, n_e1 = ok.shape
    return sum(bool(ok[(r + dr) % n_roll, e + de]) for dr in (-1, 0, 1) for de in (-1, 0, 1)
               if 0 <= e + de < n_e1)


def _reach(robot, cell, world, roll, e1, seed) -> float:
    q = robot.ik_path(np.linalg.inv(robroot(cell, e1)) @ world @ frame(c=roll), seed_deg=seed)
    return float((~np.isnan(q[:, 0])).mean())


def auto_position(plan, robot, cell: CellConfig, tilts, e1_now=None, e1_step=500.0, log=print) -> Position:
    """`plan(tilt)` -> Toolpath in BASE (roll 0). For each tilt in order, find the
    gun roll and E1 that reach the whole path, preferring settings whose
    neighbours reach too (margin) and then E1 near `e1_now`. Returns the first
    fully reachable setting, or the best partial one found."""
    seed = cell.home_deg
    if cell.gantry:
        lo, hi = cell.gantry.limits_mm
        e1s = list(np.arange(lo, hi + 1e-6, e1_step))
        e1_now = cell.gantry.position_mm if e1_now is None else e1_now
    else:
        e1s, e1_now = [None], 0.0
    best = Position(tilts[0], 0.0, e1_now if cell.gantry else None, -1.0)
    for tilt in tilts:
        log(f"Auto position: trying tilt {tilt:.0f}°...")
        path = plan(tilt)
        world = _spread(world_poses(path, cell), FINE_POINTS)
        coarse = _spread(world, COARSE_POINTS)
        ok = np.array([[_reaches_all(robot, np.linalg.inv(robroot(cell, e1)) @ coarse @ frame(c=r), seed)
                        for e1 in e1s] for r in ROLLS])
        order = sorted(zip(*np.nonzero(ok)),
                       key=lambda rc: (-_margin(ok, *rc), abs((e1s[rc[1]] or 0.0) - e1_now), abs(ROLLS[rc[0]])))
        for r, e in order[:4]:
            reach = _reach(robot, cell, world, ROLLS[r], e1s[e], seed)
            cand = Position(tilt, float(ROLLS[r]), None if e1s[e] is None else float(e1s[e]), reach)
            if reach > best.reach:
                best = cand
            if reach == 1.0:
                return cand
    return best
