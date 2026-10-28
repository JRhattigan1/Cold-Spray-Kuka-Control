"""Lever `purge`: move the purge point in X/Y/Z (WORLD), spray direction kept.

Only when problem points lie on purge moves (tag "purge"). Offsets of up to
+-STEP_MM * N per axis are tried, smallest first (planner.purge.move_purge puts
the purge poses there and redoes the clearance vias); the one with fewest
problem points is kept, stopping once no purge move has any left. The new point
goes to `ctx.suggested["purge_point"]` for the SolveReport (`ctx.cell` is not changed).
"""

import itertools

import numpy as np

from coldspray.planner.purge import move_purge
from coldspray.planner.solver import Context, find_issues

STEP_MM = 200.0
N = 1                     # offsets -200, 0, +200 mm per axis (26 tries)


def offsets():
    steps = np.arange(-N, N + 1) * STEP_MM
    out = [np.array(o) for o in itertools.product(steps, steps, steps) if any(o)]
    return sorted(out, key=lambda o: (np.linalg.norm(o), tuple(np.abs(o))))


def _on_purge(path, issues):
    return sum(path.moves[i.move].tag == "purge" for i in issues)


def fix_purge(path, issues, ctx: Context):
    purge = getattr(ctx.cell, "purge", None)
    if purge is None or not getattr(purge, "enabled", False) or not _on_purge(path, issues):
        return path
    old = np.asarray(purge.point, float)
    best, best_n, best_point = path, len(issues), None
    for off in offsets():
        point = old + off
        trial = move_purge(path, ctx.cell, point)
        found = find_issues(trial, ctx)[2]
        if len(found) < best_n:
            best, best_n, best_point = trial, len(found), point
            if not _on_purge(trial, found):
                break
    if best_point is None:
        return path
    ctx.suggested = dict(getattr(ctx, "suggested", None) or {}) | {
        "purge_point": tuple(float(x) for x in best_point)}
    return best
