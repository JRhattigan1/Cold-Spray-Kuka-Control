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

from coldspray.cell import base_world, base_worlds, robroot
from coldspray.toolpath.model import Toolpath

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
    n = len(path.moves)
    poses = np.empty((n, 4, 4))
    for i, m in enumerate(path.moves):
        poses[i] = m.pose
    r_all = base_worlds(cell, path.axis_values())[:, :3, :3] @ poses[:, :3, :3]   # tool axes in WORLD
    xs, cur_all = r_all[:, :, 0], r_all @ w
    turns = np.empty(n)
    held = None                                                     # wrist direction, WORLD
    for i in range(n):                                              # sequential: each pose carries the last
        x, cur = xs[i], cur_all[i]
        options = ([held] if held is not None else []) + ([up] if held is not None or abs(x @ up) <= VERTICAL
                                                           else []) + [side]
        for want in options:                                        # first one not along the spray
            d = want - (want @ x) * x
            nd = np.sqrt(d @ d)
            if nd > 0.1:
                break
        d = d / nd
        c = cur @ d
        s = x @ (np.array([cur[1] * d[2] - cur[2] * d[1], cur[2] * d[0] - cur[0] * d[2],
                           cur[0] * d[1] - cur[1] * d[0]]))
        turns[i] = np.arctan2(s, c)
        held = d
    turns += np.radians(angle_deg)
    cs, sn = np.cos(turns), np.sin(turns)
    new = poses[:, :3, :3].copy()                                   # pose @ rotation about tool X
    y, z = poses[:, :3, 1], poses[:, :3, 2]
    new[:, :, 1] = y * cs[:, None] + z * sn[:, None]
    new[:, :, 2] = -y * sn[:, None] + z * cs[:, None]
    poses[:, :3, :3] = new
    moves = [replace(m, pose=poses[i], axes=dict(m.axes)) for i, m in enumerate(path.moves)]
    return replace(path, moves=moves)
