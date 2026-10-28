"""Lever `gantry`: one gantry position (E1) for the whole program.

Every E1 inside cell.gantry.limits_mm is scored with find_issues, coarse
(COARSE_MM steps) then fine (FINE_MM steps round the best coarse one); the one
with fewest problem points wins (nearest the current E1 on a tie). It is written
to `axes["E1"]` of the first move, which must be a gun-off PTP (else a gun-off
PTP copy of the first move is put in front), and any per-move E1 is dropped.
The value is left in `ctx.suggested["gantry_e1"]` for the SolveReport. With a
collision checker, E1 stays within solve_e1.MAX_SHIFT_MM of the one it was built for.
"""

from dataclasses import replace

import numpy as np

from coldspray.cell import axis_defaults
from coldspray.planner import solver as _solver
from coldspray.planner.solve_e1 import MAX_SHIFT_MM
from coldspray.planner.solver import (Context, Issue, e1_of, ptp_in_joint_space, ptp_moves,
                                      robot_targets)

COARSE_MM = 250.0
FINE_MM = 50.0


def with_gantry(path, axis, e1):
    """`path` driven at gantry `e1` throughout: set on the first move (a gun-off PTP)."""
    moves = [replace(m, axes={k: v for k, v in m.axes.items() if k != axis}) for m in path.moves]
    first = moves[0]
    if first.gun_on:
        moves.insert(0, replace(first, gun_on=False, kind="PTP", tag=first.tag or "approach",
                                dwell_s=0.0, axes=dict(first.axes)))
        moves[1] = replace(moves[1], axes={})
    moves[0] = replace(moves[0], kind="PTP", axes=dict(moves[0].axes) | {axis: float(e1)})
    return replace(path, moves=moves)


def count_issues(path, ctx: Context, limit=None) -> list:
    """find_issues(path, ctx)[2] - but once more than `limit` problem points are found the
    rest is not collision-checked (the list then stops at limit + 1). Candidates are only
    compared, so a candidate already worse than the best needn't be counted out. IK runs
    over the whole path as in find_issues and ctx.last is set the same way."""
    sampled = getattr(_solver, "_sampled", None)          # S8F: per-path sample cache
    if sampled is not None:
        s, targets, hint = sampled(path, ctx)
    else:
        o = ctx.opts
        s = path.sample(o.sample_mm, o.angle_step_deg, axis_defaults(ctx.cell))
        targets, hint = robot_targets(s, ctx.cell), path.hint_for(s.move_index)
    seed = ctx.cell.home_deg if ctx.seed is None else ctx.seed
    joints = ctx.robot.ik_path(targets, seed_deg=seed, hint=hint)
    joints = ptp_in_joint_space(ptp_moves(path, ctx.cell), s.move_index, joints)
    ctx.last = (s, joints)
    bad = np.isnan(joints[:, 0])
    issues = [Issue(int(i), int(s.move_index[i]), "unreachable") for i in np.flatnonzero(bad)]
    if limit is not None and len(issues) > limit:
        return issues[:limit + 1]
    if ctx.checker is not None:
        check = getattr(_solver, "_check", None)          # S8F: cached collision results
        for i in np.flatnonzero(~bad):
            if check is not None:
                detail = check(ctx, joints[i], s.axes_at(i))
            else:
                pairs = ctx.checker.check(joints[i], s.axes_at(i))
                detail = ", ".join(f"{a}/{b}" for a, b in sorted(pairs)) if pairs else None
            if detail is not None:
                issues.append(Issue(int(i), int(s.move_index[i]), "collision", detail))
                if limit is not None and len(issues) > limit:
                    break
    issues.sort(key=lambda x: x.sample)
    return issues


def fix_gantry(path, issues, ctx: Context):
    g = ctx.cell.gantry
    if g is None or not issues:
        return path
    lo, hi = g.limits_mm
    cur = e1_of(ctx.cell, path.axis_values()[0])
    centre, span = cur, None
    if ctx.checker is not None:            # it was cropped round the program's E1: stay near it
        built = getattr(ctx.checker, "e1", None)
        centre = g.position_mm if built is None else built
        span = MAX_SHIFT_MM
    scores = {}

    def score(e):
        e = float(min(max(round(float(e), 6), lo), hi))      # inside limits_mm (no 5500.000000000001)
        if span is not None and abs(e - centre) > span + 1e-9:
            return e, None
        if e not in scores:              # worse than the best so far: stop counting (never chosen)
            limit = min(scores.values()) if scores else None
            scores[e] = len(count_issues(with_gantry(path, g.axis, e), ctx, limit))
        return e, scores[e]

    def best():
        return min(scores, key=lambda e: (scores[e], abs(e - cur)))

    score(cur)
    coarse = np.arange(np.ceil(lo / COARSE_MM) * COARSE_MM, hi + 1e-9, COARSE_MM)
    for e in sorted(coarse, key=lambda e: abs(e - cur)):     # nearest first; stop once clear
        if scores[best()] == 0:
            break
        score(e)
    b = best()
    if scores[b] > 0:
        for e in np.arange(b - COARSE_MM + FINE_MM, b + COARSE_MM, FINE_MM):
            score(e)
    b = best()
    if b == cur or scores[b] >= len(issues):
        return path
    ctx.suggested = dict(getattr(ctx, "suggested", None) or {}) | {"gantry_e1": b}
    return with_gantry(path, g.axis, b)
