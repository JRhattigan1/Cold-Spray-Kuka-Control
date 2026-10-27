"""Solver lever "angle": tip the gun on the stretches that cannot be reached or collide.

For each bad stretch (a run of moves with problem samples) try, smallest first,
a gun angle offset (turn about the spray axis, tool X) up to +-angle_band_deg,
then a lean (turn about the axis square to spray and travel, tool Z) up to
+-lean_max_deg, in step_deg steps. The offset is full over the stretch and
blends to nothing over ramp_mm of path length either side. Positions are never
touched, so the TCP point and the spray footprint centre stay where they were.
"""

from dataclasses import replace

import numpy as np

from coldspray.planner.solver import find_issues, issues_in
from coldspray.transforms import frame


def _candidates(opts):
    """(angle, lean) pairs: angle offsets first, then leans, each smallest first."""
    out = []
    for limit, kind in ((opts.angle_band_deg, 0), (opts.lean_max_deg, 1)):
        step = max(opts.step_deg, 1e-6)
        for k in range(1, int(limit / step + 1e-9) + 1):
            for sign in (1, -1):
                v = sign * k * step
                out.append((v, 0.0) if kind == 0 else (0.0, v))
    return out


def _stretches(bad: list[int]) -> list[list[int]]:
    runs = []
    for m in sorted(bad):
        if runs and m - runs[-1][-1] <= 1:
            runs[-1].append(m)
        else:
            runs.append([m])
    return runs


def _weights(path, run, ramp_mm) -> np.ndarray:
    """Per-move weight: 1 over the run, falling to 0 within ramp_mm of path length outside it."""
    pos = np.array([m.pose[:3, 3] for m in path.moves])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))])
    lo, hi = s[run[0]], s[run[-1]]
    d = np.maximum(np.maximum(lo - s, s - hi), 0.0)
    if ramp_mm <= 0:
        return (d == 0).astype(float)
    return np.clip(1.0 - d / ramp_mm, 0.0, 1.0)


def _apply(path, w, angle, lean):
    moves = []
    for m, wi in zip(path.moves, w):
        if wi <= 0:
            moves.append(m)
            continue
        moves.append(replace(m, pose=m.pose @ frame(a=lean * wi, c=angle * wi), axes=dict(m.axes)))
    return replace(path, moves=moves)


def fix_angle(path, issues, ctx):
    opts = ctx.opts
    if not issues or (opts.angle_band_deg <= 0 and opts.lean_max_deg <= 0):
        return path
    cands = _candidates(opts)
    cur, cur_issues, changed = path, issues, False
    for run in _stretches([i.move for i in issues]):
        w = _weights(cur, run, opts.ramp_mm)
        region = {i for i in range(len(w)) if w[i] > 0}
        if not any(i.move in region for i in cur_issues):
            continue                                  # an earlier stretch's change already fixed it
        lo, hi = min(region), max(region) + 1          # the move after the ramp feels it too
        for angle, lean in cands:
            trial = _apply(cur, w, angle, lean)
            if not issues_in(trial, ctx, lo, hi):        # fast: only the changed stretch
                _, _, cur_issues = find_issues(trial, ctx)
                cur, changed = trial, True
                break
    return cur if changed else path
