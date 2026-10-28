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

from coldspray.planner.solver import (Context, Issue, _check, _sampled, find_issues, ptp_in_joint_space,
                                      ptp_moves)
from coldspray.toolpath.model import Toolpath

TAGS = {"approach", "jump", "purge"}
LIFTS_MM = (25.0, 50.0, 100.0, 150.0, 200.0, 300.0, 400.0)


def count_in(path: Toolpath, ctx: Context, lo: int, hi: int, limit: int | None = None, stride: int = 8) -> int:
    """Number of problem points in moves lo..hi - what len(issues_in(path, ctx, lo, hi)) says -
    but it stops once the count reaches `limit` (then the result is only >= limit). The
    expensive collision checks run on every `stride`-th reachable sample first, so a bad
    candidate is usually dropped after a fraction of them. IK and seeds are those of issues_in."""
    s, all_targets, hint = _sampled(path, ctx)
    sel = np.flatnonzero((s.move_index >= lo) & (s.move_index <= hi))
    if not len(sel):
        return 0
    seed = ctx.cell.home_deg if ctx.seed is None else ctx.seed
    last = getattr(ctx, "last", None)
    if last is not None:
        before = np.flatnonzero(last[0].move_index < lo)
        if len(before) and not np.isnan(last[1][before[-1], 0]):
            seed = last[1][before[-1]]
    targets = all_targets[sel]
    joints = ctx.robot.ik_path(targets, seed_deg=seed, hint=None if hint is None else hint[sel])
    joints = ptp_in_joint_space(ptp_moves(path, ctx.cell), s.move_index[sel], joints,
                                before=seed if last is not None or lo == 0 else None)
    reach = ~np.isnan(joints[:, 0])
    n = int((~reach).sum())
    if ctx.checker is None or (limit is not None and n >= limit):
        return n
    rows = np.flatnonzero(reach)
    order = np.concatenate([rows[::stride], np.setdiff1d(rows, rows[::stride])]) if stride > 1 else rows
    for r in order:
        if _check(ctx, joints[r], s.axes_at(sel[r])) is not None:
            n += 1
            if limit is not None and n >= limit:
                break
    return n


def prime_seed(path: Toolpath, ctx: Context, lo: int) -> None:
    """Leave ctx.last as a full find_issues would for what `issues_in(.., lo, ..)` reads of it:
    the joints of the samples before move `lo` (IK only, no collision checks, nothing after)."""
    from types import SimpleNamespace
    s, targets, hint = _sampled(path, ctx)
    n = int(np.searchsorted(s.move_index, lo, side="left"))     # move_index is non-decreasing
    seed = ctx.cell.home_deg if ctx.seed is None else ctx.seed
    joints = ctx.robot.ik_path(targets[:n], seed_deg=seed, hint=None if hint is None else hint[:n])
    joints = ptp_in_joint_space(ptp_moves(path, ctx.cell), s.move_index[:n], joints)
    ctx.last = (SimpleNamespace(move_index=s.move_index[:n]), joints)


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
    ks = sorted({i.move for i in issues}, reverse=True)
    for n, k in enumerate(ks):
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
            if not count_in(cand, ctx, k, hi, limit=1):     # fast: only the changed moves, stop at the first bad point
                cur, changed = cand, True
                if n + 1 < len(ks):                         # the next move's check needs the full state
                    _, _, cur_issues = find_issues(cand, ctx)
                break
    return cur if changed else path
