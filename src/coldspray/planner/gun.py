"""Hold the gun at a fixed angle about its spray axis: no spinning.

Planners choose the spray point and direction; how the gun is turned about
that axis is ours to pick. The gun has cables and a powder line out of the
back that are not in the CAD, so it must never wind round: the gun keeps one
attitude and only tilts as far as the spray direction makes it.

The first pose is turned so the wrist side of the gun points as near WORLD up
as it can (the usual set-up: horizontal spray, wrist and robot above the gun),
or, spraying within ~25 degrees of vertical, towards the robot. Every later
pose carries the previous pose's wrist direction over to its own spray
direction by the smallest tilt (parallel transport), so the gun never turns
with the travel direction: loops and zig-zags keep one attitude, and a path
that keeps one spray direction (Layers, Revolve) stays wrist-up throughout.
Then every pose is turned by the user's gun angle.
"""

from dataclasses import replace

import numpy as np

from coldspray.cell import base_world, robroot
from coldspray.toolpath.model import Toolpath
from coldspray.transforms import frame

VERTICAL = 0.9          # |spray . up| above this: too near vertical to level


def wrist_side(tool: np.ndarray) -> np.ndarray:
    """Unit vector in the tool frame, square to the spray axis (tool X), from
    the TCP towards the flange - the side the wrist and arm are on."""
    back = -tool[:3, :3].T @ tool[:3, 3]
    back[0] = 0.0
    return back / np.linalg.norm(back)


def level_gun(path: Toolpath, cell, wrist, angle_deg: float = 0.0) -> Toolpath:
    """Copy of `path` (BASE) with every gun turned about its spray axis as the
    module doc says, then turned `angle_deg` further. Spray points and
    directions are unchanged."""
    w = np.asarray(wrist, float)
    up = np.array([0.0, 0.0, 1.0])
    side = robroot(cell)[:3, 3] - base_world(cell)[:3, 3]           # towards the robot, WORLD
    side[2] = 0.0
    side = side / np.linalg.norm(side) if np.linalg.norm(side) > 1e-9 else np.array([1.0, 0.0, 0.0])
    moves, held = [], None                                          # held: wrist direction, WORLD
    for m, axes in zip(path.moves, path.axis_values()):
        bw = base_world(cell, axes)[:3, :3]
        r = bw @ m.pose[:3, :3]                                     # tool axes in WORLD
        x = r[:, 0]
        options = ([held] if held is not None else []) + ([up] if held is not None or abs(x @ up) <= VERTICAL
                                                           else []) + [side]
        for want in options:                                        # first one not along the spray
            d = want - (want @ x) * x
            if np.linalg.norm(d) > 0.1:
                break
        d /= np.linalg.norm(d)
        cur = r @ w
        turn = np.degrees(np.arctan2(x @ np.cross(cur, d), cur @ d))
        held = d
        moves.append(replace(m, pose=m.pose @ frame(c=turn + angle_deg), axes=dict(m.axes)))
    return replace(path, moves=moves)
