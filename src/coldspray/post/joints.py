"""Joint angles the program should command: exactly what Simulate checked.

Simulate (sim/viewer.py set_path, as called from gui/steps/simulate.py) samples
the path every 2 mm / 5 deg, solves `robot.ik_path` from the cell's home joints
with the path's joint hints, and collision-checks those joints. A PTP in the
KRL must go to the same configuration, so it takes its joints from here instead
of a fresh `robot.ik` seeded from the last PTP (which may pick another branch).
"""

import numpy as np

from coldspray.planner.solver import robot_targets

STEP_MM = 2.0          # Simulate's sampling (gui/steps/simulate.py)
ANGLE_STEP_DEG = 5.0


def program_joints(path, robot, cell) -> np.ndarray:
    """(n_moves, 6) joints at the end of every move, as Simulate solves them
    (NaN where unreachable)."""
    s = path.sample(STEP_MM, ANGLE_STEP_DEG)
    joints = robot.ik_path(robot_targets(s, cell), seed_deg=cell.home_deg, hint=path.hint_for(s.move_index))
    idx = np.asarray(s.move_index)
    moves = np.arange(len(path.moves))
    last = np.searchsorted(idx, moves, side="right") - 1      # samples run in move order
    out = np.full((len(path.moves), 6), np.nan)
    ok = (last >= 0) & (idx[last.clip(0)] == moves)
    out[ok] = joints[last[ok]]
    return out
