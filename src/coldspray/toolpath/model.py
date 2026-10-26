"""Toolpath: the format shared by planners, the simulator and post-processors.

Poses are 4x4 TCP frames in the work BASE frame (mm). The tool X axis is the
spray direction; tool Y follows the travel direction.
"""

from dataclasses import dataclass, field
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass
class Move:
    pose: np.ndarray
    speed: float                      # mm/s (ignored for PTP)
    gun_on: bool = False              # spray state during this move
    kind: Literal["LIN", "PTP"] = "LIN"


@dataclass
class Toolpath:
    name: str
    moves: list[Move] = field(default_factory=list)

    def sample(self, step: float = 5.0):
        """Poses every ~`step` mm along the path, for simulation and checks.

        Returns (poses, gun_on, move_index). PTP moves are interpolated in
        Cartesian space here, which is only an approximation of the robot.
        """
        poses, gun, index = [self.moves[0].pose], [self.moves[0].gun_on], [0]
        for i in range(1, len(self.moves)):
            a, b = self.moves[i - 1].pose, self.moves[i].pose
            n = max(1, int(np.ceil(np.linalg.norm(b[:3, 3] - a[:3, 3]) / step)))
            slerp = Slerp([0, 1], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))
            for t in np.linspace(0, 1, n + 1)[1:]:
                p = np.eye(4)
                p[:3, :3] = slerp(t).as_matrix()
                p[:3, 3] = (1 - t) * a[:3, 3] + t * b[:3, 3]
                poses.append(p)
                gun.append(self.moves[i].gun_on)
                index.append(i)
        return np.array(poses), np.array(gun), np.array(index)


def spray_frame(point, spray_dir, travel_dir) -> np.ndarray:
    """TCP frame at `point` with tool X along `spray_dir`, Y towards `travel_dir`."""
    x = np.asarray(spray_dir, float)
    x /= np.linalg.norm(x)
    y = np.asarray(travel_dir, float) - np.dot(travel_dir, x) * x
    y /= np.linalg.norm(y)
    t = np.eye(4)
    t[:3, :3] = np.column_stack([x, y, np.cross(x, y)])
    t[:3, 3] = point
    return t
