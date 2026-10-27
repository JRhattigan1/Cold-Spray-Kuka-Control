"""Solver lever "reroute": lift or go round on the non-spraying moves.

Only moves tagged approach/jump/purge, or with the gun off, are touched; spray
moves are never changed (jumps and purge transits keep the spray on, and so do
their via points - the spray state never toggles). For each such move that has a
problem sample, in order:
  1. lift its end point further along the spray axis (away from the surface),
     if the move after it is not a spray move that starts there;
  2. add a via point: go up from where the move starts, across at that height,
     then come down to the original end point.
The first height that clears the stretch wins. If none does, the path is returned
unchanged.
"""

from dataclasses import replace

import numpy as np

from coldspray.planner.solver import Context, Issue, find_issues, issues_in
from coldspray.toolpath.model import Toolpath

TAGS = {"approach", "jump", "purge"}
LIFTS_MM = (25.0, 50.0, 100.0, 150.0, 200.0, 300.0, 400.0)


def _free(m) -> bool:
    return m.tag in TAGS or not m.gun_on     # approach, jump, purge, or any gun-off move


def _up(pose, h):
    p = pose.copy()
    p[:3, 3] -= h * pose[:3, 0]          # tool X points at the surface: away from it is -X
    return p


def _bad_near(issues: list[Issue], lo: int, hi: int) -> int:
    return sum(lo <= i.move <= hi for i in issues)


def fix_route(path: Toolpath, issues: list[Issue], ctx: Context) -> Toolpath:
    cur, cur_issues, changed = path, issues, False
    for k in sorted({i.move for i in issues}, reverse=True):
        moves = cur.moves
        if k == 0 or k >= len(moves) or not _free(moves[k]):
            continue
        before = _bad_near(cur_issues, k, k + 1)
        if before == 0:
            continue
        nxt_ok = k + 1 >= len(moves) or _free(moves[k + 1])
        cands = []
        if nxt_ok:
            for h in LIFTS_MM:
                ms = list(moves)
                ms[k] = replace(moves[k], pose=_up(moves[k].pose, h), axes=dict(moves[k].axes))
                cands.append((Toolpath(cur.name, ms, cur.skipped_points), k, k + 1))
        for h in LIFTS_MM:
            a, b = moves[k - 1], moves[k]
            va = replace(b, pose=_up(a.pose, h), tag=b.tag or "jump", axes={})
            vb = replace(b, pose=_up(b.pose, h), tag=b.tag or "jump", axes={})
            ms = list(moves[:k]) + [va, vb] + list(moves[k:])
            cands.append((Toolpath(cur.name, ms, cur.skipped_points), k, k + 3))
        for cand, lo, hi in cands:
            if not issues_in(cand, ctx, k, hi):             # fast: only the changed moves
                _, _, cur_issues = find_issues(cand, ctx)
                cur, changed = cand, True
                break
    return cur if changed else path
