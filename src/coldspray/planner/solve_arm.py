"""Arm lever: pick the arm/wrist configuration along the whole path.

Greedy IK (`Robot.ik_path`) follows the nearest solution sample by sample and
can end up in a poor configuration (or flip late). Here every in-limit
solution of every sample (`Robot.ik_all`) is a node, and dynamic programming
finds the chain with the least joint motion. Inside a spraying run a joint
jump above `JUMP_DEG` between neighbouring samples is heavily penalised, so it
only happens when no alternative exists. Solutions the checker reports as
colliding are penalised too, and the chain is re-solved (a few rounds).

The path is not changed geometrically. The result carries `joint_hint`, an
(n_moves, 6) array of the joints at the end of every move, for
`Robot.ik_path(..., hint=)` to seed from.
"""

from dataclasses import replace

import numpy as np

from coldspray.cell import axis_defaults
from coldspray.planner.solver import robot_targets

JUMP_DEG = 20.0          # per sample step, while spraying
JUMP_PENALTY = 1e4
COLLISION_PENALTY = 1e6
ROUNDS = 4


def _chain(sols, spraying, bad):
    """Cheapest chain of solution indices. sols[i] is a (k,6) array or None (unreachable)."""
    n = len(sols)
    cost = prev_cost = None
    prev_i = None
    back = [None] * n
    prev_of = [None] * n
    for i in range(n):
        if sols[i] is None:
            continue
        node = np.array([COLLISION_PENALTY if (i, j) in bad else 0.0 for j in range(len(sols[i]))])
        if prev_i is None:
            cost = node
        else:
            d = np.abs(sols[i][None, :, :] - sols[prev_i][:, None, :]).max(axis=2)   # (kp, k)
            jump = JUMP_PENALTY if spraying[i] and spraying[prev_i] else 0.0
            tot = prev_cost[:, None] + d + np.where(d > JUMP_DEG, jump, 0.0)
            back[i] = tot.argmin(axis=0)
            cost = tot.min(axis=0) + node
            prev_of[i] = prev_i
        prev_cost, prev_i = cost, i
    pick = [None] * n
    if prev_i is None:
        return pick
    i, j = prev_i, int(prev_cost.argmin())
    while i is not None:
        pick[i] = j
        if prev_of[i] is not None:
            j = int(back[i][j])
        i = prev_of[i]
    return pick


def fix_arm(path, issues, ctx):
    if not issues:
        return path
    o = ctx.opts
    s = path.sample(o.sample_mm, o.angle_step_deg, axis_defaults(ctx.cell))
    sols = []
    for t in robot_targets(s, ctx.cell):
        v = ctx.robot.ik_all(t)
        sols.append(np.array(v) if v else None)
    if all(x is None for x in sols):
        return path
    bad: set = set()
    pick = _chain(sols, s.gun_on, bad)
    for _ in range(ROUNDS if ctx.checker is not None else 0):
        new = {(i, pick[i]) for i in range(len(sols))
               if pick[i] is not None and ctx.checker.check(sols[i][pick[i]], s.axes_at(i))}
        if not new:
            break
        bad |= new
        pick = _chain(sols, s.gun_on, bad)
    hint = np.full((len(path.moves), 6), np.nan)
    for i in range(len(sols)):
        if pick[i] is not None:
            hint[s.move_index[i]] = sols[i][pick[i]]       # last sample of a move wins
    return replace(path, joint_hint=hint)
