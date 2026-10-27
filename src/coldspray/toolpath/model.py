"""Toolpath: the format shared by planners, the simulator and post-processors.

Poses are 4x4 TCP frames in the work BASE frame (mm): the turntable faceplate,
origin at its top centre. The tool X axis is the spray direction; tool Y
follows the travel direction. A move may also set external axes (turntable
tilt and rotation, degrees); axes it doesn't mention keep their previous value.
"""

from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from coldspray.transforms import frame


@dataclass
class Move:
    pose: np.ndarray
    speed: float                      # mm/s along the part surface (ignored for PTP)
    gun_on: bool = False              # spray state during this move
    kind: Literal["LIN", "PTP"] = "LIN"
    axes: dict[str, float] = field(default_factory=dict)  # e.g. {"E2": 0.0, "E5": 90.0}
    tag: str = ""                     # what the move is for: contour, infill, turn, jump, purge, approach
    dwell_s: float = 0.0              # wait this long at the point after arriving


@dataclass
class Samples:
    poses: np.ndarray                 # (n, 4, 4) in BASE
    gun_on: np.ndarray                # (n,)
    move_index: np.ndarray            # (n,) which move each sample belongs to
    axes: dict[str, np.ndarray]       # external axis values per sample

    def __len__(self):
        return len(self.poses)

    def axes_at(self, i: int) -> dict[str, float]:
        return {k: float(v[i]) for k, v in self.axes.items()}


@dataclass
class Toolpath:
    name: str
    moves: list[Move] = field(default_factory=list)
    skipped_points: int = 0           # points a planner dropped (e.g. surface tilt limit)
    joint_hint: np.ndarray | None = None   # (n_moves, 6) joints to seed IK from (solver; NaN = none)

    def hint_for(self, move_index) -> np.ndarray | None:
        """`joint_hint` per sample (rows by move), or None if there is none or it no longer fits."""
        h = self.joint_hint
        return None if h is None or len(h) != len(self.moves) else h[np.asarray(move_index)]

    def axis_values(self) -> list[dict[str, float]]:
        """External axis values in force at each move (carried forward)."""
        state, out = {}, []
        for m in self.moves:
            state = state | m.axes
            out.append(dict(state))
        return out

    def transformed(self, t: np.ndarray) -> "Toolpath":
        """Copy with every pose replaced by `t @ pose` (everything else kept)."""
        return Toolpath(self.name, [replace(m, pose=t @ m.pose, axes=dict(m.axes)) for m in self.moves],
                        self.skipped_points, self.joint_hint)

    def rolled(self, deg: float) -> "Toolpath":
        """Copy with the gun turned `deg` about its spray axis (tool X) at every pose."""
        r = frame(c=deg)
        return Toolpath(self.name, [replace(m, pose=m.pose @ r, axes=dict(m.axes)) for m in self.moves],
                        self.skipped_points, self.joint_hint)

    def sample(self, step: float = 5.0, angle_step: float = 2.0) -> Samples:
        """Points every ~`step` mm (and every ~`angle_step` degrees of external
        axis motion) along the path, for simulation and checks.

        PTP moves are interpolated in Cartesian space here, which is only an
        approximation of the robot.
        """
        axes_per_move = self.axis_values()
        names = sorted({k for a in axes_per_move for k in a})
        first = {k: axes_per_move[-1].get(k, 0.0) for k in names}  # value before an axis is first set
        values = [{k: a.get(k, first[k]) for k in names} for a in axes_per_move]

        poses, gun, index = [self.moves[0].pose], [self.moves[0].gun_on], [0]
        ax = {k: [values[0][k]] for k in names}
        for i in range(1, len(self.moves)):
            a, b = self.moves[i - 1].pose, self.moves[i].pose
            dist = np.linalg.norm(b[:3, 3] - a[:3, 3])
            turn = max((abs(values[i][k] - values[i - 1][k]) for k in names), default=0.0)
            n = max(1, int(np.ceil(dist / step)), int(np.ceil(turn / angle_step)))
            slerp = Slerp([0, 1], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))
            for t in np.linspace(0, 1, n + 1)[1:]:
                p = np.eye(4)
                p[:3, :3] = slerp(t).as_matrix()
                p[:3, 3] = (1 - t) * a[:3, 3] + t * b[:3, 3]
                poses.append(p)
                gun.append(self.moves[i].gun_on)
                index.append(i)
                for k in names:
                    ax[k].append((1 - t) * values[i - 1][k] + t * values[i][k])
        return Samples(np.array(poses), np.array(gun), np.array(index),
                       {k: np.array(v) for k, v in ax.items()})


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
