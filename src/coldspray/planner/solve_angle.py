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

from coldspray.planner.solver import find_issues, ptp_in_joint_space, ptp_moves, robot_targets
from coldspray.cell import axis_defaults
from coldspray.toolpath.model import Toolpath
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


def _apply_range(path, w, angle, lean, a, b):
    """Moves a..b (inclusive) of `path` with the offset applied (others untouched)."""
    moves = []
    for k in range(a, b + 1):
        m, wi = path.moves[k], w[k]
        if wi <= 0 and (m.aux is None or k == 0 or w[k - 1] <= 0):
            moves.append(m)
            continue
        aux = m.aux
        if aux is not None:         # CIRC via point: midway along the arc, so midway weight
            wa = 0.5 * (wi + (w[k - 1] if k else wi))
            aux = aux @ frame(a=lean * wa, c=angle * wa)
        moves.append(replace(m, pose=m.pose @ frame(a=lean * wi, c=angle * wi), aux=aux, axes=dict(m.axes)))
    return moves


def _apply(path, w, angle, lean):
    return replace(path, moves=_apply_range(path, w, angle, lean, 0, len(path.moves) - 1))


def _full_axes(path, initial):
    """Axis values of every move, as Toolpath.sample fills them in."""
    per_move = path.axis_values()
    names = sorted({k for a in per_move for k in a})
    first = {k: initial.get(k, per_move[-1].get(k, 0.0)) for k in names}
    return [{k: a.get(k, first[k]) for k in names} for a in per_move]


def _clears(trial_moves, a, base, ctx, lo, hi, axes, ptp):
    """True if moves lo..hi have no problem point: the check issues_in makes, on just the
    window of moves a..hi (the sampling of a segment only needs its two end moves)."""
    o = ctx.opts
    win = Toolpath(base.name, [replace(m, axes=axes[a + k]) for k, m in enumerate(trial_moves)])
    s = win.sample(o.sample_mm, o.angle_step_deg)
    mi = s.move_index + a
    sel = np.flatnonzero((mi >= lo) & (mi <= hi))
    if not len(sel):
        return True
    seed = ctx.cell.home_deg if ctx.seed is None else ctx.seed
    last = getattr(ctx, "last", None)
    if last is not None:
        before = np.flatnonzero(last[0].move_index < lo)
        if len(before) and not np.isnan(last[1][before[-1], 0]):
            seed = last[1][before[-1]]
    targets = robot_targets(s, ctx.cell)[sel]
    hint = base.joint_hint
    hint = None if hint is None or len(hint) != len(base.moves) else hint[mi]
    joints = ctx.robot.ik_path(targets, seed_deg=seed, hint=None if hint is None else hint[sel])
    joints = ptp_in_joint_space(ptp, mi[sel], joints, before=seed if last is not None or lo == 0 else None)
    if np.isnan(joints[:, 0]).any():
        return False
    if ctx.checker is not None:
        for i, q in zip(sel, joints):
            if ctx.checker.check(q, s.axes_at(i)):
                return False
    return True


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
        n = len(cur.moves)
        a, b = max(lo - 1, 0), min(hi, n - 1)
        axes = _full_axes(cur, axis_defaults(ctx.cell))
        ptp = ptp_moves(cur, ctx.cell)
        for angle, lean in cands:
            part = _apply_range(cur, w, angle, lean, a, b)
            if _clears(part, a, cur, ctx, lo, hi, axes, ptp):   # fast: only the changed stretch
                trial = replace(cur, moves=cur.moves[:a] + part + cur.moves[b + 1:])
                _, _, cur_issues = find_issues(trial, ctx)
                cur, changed = trial, True
                break
    return cur if changed else path
