"""Hold the gun at a fixed angle about its spray axis: wrist above the gun.

Planners choose the spray point and direction; how the gun is turned about
that axis is ours to pick. The usual set-up sprays horizontally with the wrist
(and the robot) above the gun, so each pose is turned until the wrist side of
the gun points as near WORLD up as it can, then by the user's gun angle. With
the table where each move puts it, "up" is the same for the whole job, so the
gun keeps one attitude instead of following the path direction.

Poses spraying within ~25 degrees of vertical have no clear "up" for the
wrist (it would swing round with every change of direction), so they keep
the planner's roll and only get the gun angle.
"""

from dataclasses import replace

import numpy as np

from coldspray.cell import base_world
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
    """Copy of `path` (BASE) with every gun turned about its spray axis so
    `wrist` (tool frame, square to tool X) points up in WORLD, then turned
    `angle_deg` further. Spray points and directions are unchanged."""
    w = np.asarray(wrist, float)
    moves = []
    for m, axes in zip(path.moves, path.axis_values()):
        up = base_world(cell, axes)[:3, :3].T @ [0.0, 0.0, 1.0]      # WORLD up, in BASE
        r = m.pose[:3, :3]
        x = r[:, 0]
        turn = 0.0
        if abs(x @ up) <= VERTICAL:
            d, cur = up - (up @ x) * x, r @ w
            turn = np.degrees(np.arctan2(x @ np.cross(cur, d), cur @ d))
        moves.append(replace(m, pose=m.pose @ frame(c=turn + angle_deg), axes=dict(m.axes)))
    return replace(path, moves=moves)
