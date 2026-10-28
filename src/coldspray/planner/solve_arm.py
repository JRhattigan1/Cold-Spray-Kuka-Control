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


def _steps(sols, spraying):
    """Joint-jump cost matrix into every sample from the previous reachable one (kept
    between rounds: it does not depend on the collisions)."""
    step = [None] * len(sols)
    prev_i = None
    for i, s in enumerate(sols):
        if s is None:
            continue
        if prev_i is not None:
            d = np.abs(s[None, :, :] - sols[prev_i][:, None, :]).max(axis=2)   # (kp, k)
            jump = JUMP_PENALTY if spraying[i] and spraying[prev_i] else 0.0
            step[i] = (prev_i, d + np.where(d > JUMP_DEG, jump, 0.0))
        prev_i = i
    return step


def _chain(sols, step, bad):
    """Cheapest chain of solution indices. sols[i] is a (k,6) array or None (unreachable)."""
    n = len(sols)
    cost = prev_cost = None
    prev_i = None
    back = [None] * n
    prev_of = [None] * n
    for i in range(n):
        if sols[i] is None:
            continue
        node = np.zeros(len(sols[i]))
        for j in bad.get(i, ()):
            node[j] = COLLISION_PENALTY
        if prev_i is None:
            cost = node
        else:
            tot = prev_cost[:, None] + step[i][1]
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
    targets = robot_targets(s, ctx.cell)
    sols = [None] * len(targets)
    if len(targets):
        cands = ctx.robot._candidates(targets)                 # all samples in one batch
        for i, c in enumerate(cands):
            c = c[~np.isnan(c[:, 0])]
            if len(c):
                sols[i] = c
    if all(x is None for x in sols):
        return path
    step = _steps(sols, s.gun_on)
    bad: dict = {}                                            # sample -> colliding solution indices
    pick = _chain(sols, step, bad)
    seen: dict = {}                                           # (sample, solution) -> collides; check once
    for _ in range(ROUNDS if ctx.checker is not None else 0):
        new = False
        for i in range(len(sols)):
            j = pick[i]
            if j is None:
                continue
            if (i, j) not in seen:
                seen[i, j] = bool(ctx.checker.check(sols[i][j], s.axes_at(i)))
            if seen[i, j] and j not in bad.get(i, ()):
                bad.setdefault(i, set()).add(j)
                new = True
        if not new:
            break
        pick = _chain(sols, step, bad)
    hint = np.full((len(path.moves), 6), np.nan)
    for i in range(len(sols)):
        if pick[i] is not None:
            hint[s.move_index[i]] = sols[i][pick[i]]       # last sample of a move wins
    return replace(path, joint_hint=hint)
