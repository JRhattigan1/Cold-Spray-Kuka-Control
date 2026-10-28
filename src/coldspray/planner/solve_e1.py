"""Lever `e1`: move the gantry (E1) per layer to a position where the layer is clear.

For each layer (planner.purge.layer_ends) that has problem points, other E1 values
inside cell.gantry.limits_mm are tried in 250 mm steps, nearest the current one
first; the first that clears the layer is kept (else the one with fewest problems,
if it helps). The value is written to `axes["E1"]` of the layer's first move with
the gun off (the KRL makes an E1 change a PTP, and a PTP must never spray), and
E1 is stated on every layer from then on so it never leaks into the next layer.
A layer that sprays on every move gets no E1 of its own (it keeps the last one). A path with nothing to fix, or a cell without a gantry, comes back as is.
"""

from dataclasses import replace

import numpy as np

from coldspray.planner.purge import layer_ends
from coldspray.planner.solver import Context, e1_of, find_issues, issues_in

STEP_MM = 250.0
MAX_SHIFT_MM = 1000.0     # with a collision checker: its cropped cell covers the robot only this far


def layer_starts(path) -> list[int]:
    return [0] + [i + 1 for i in layer_ends(path) if i + 1 < len(path.moves)]


def e1_carriers(path, starts) -> list[int | None]:
    """Per layer: its first move with the gun off (where an E1 change may go), or None."""
    ends = starts[1:] + [len(path.moves)]
    return [next((i for i in range(a, b) if not path.moves[i].gun_on), None) for a, b in zip(starts, ends)]


def _with_e1(path, carriers, values, axis):
    moves = list(path.moves)
    for s, v in zip(carriers, values):
        if s is not None:
            moves[s] = replace(moves[s], axes=dict(moves[s].axes) | {axis: float(v)})
    return replace(path, moves=moves)


def fix_e1(path, issues, ctx: Context):
    g = ctx.cell.gantry
    if g is None or not issues:
        return path
    starts = layer_starts(path)
    ends = starts[1:] + [len(path.moves)]
    carriers = e1_carriers(path, starts)
    axes = path.axis_values()
    cur = [e1_of(ctx.cell, axes[s if c is None else c]) for s, c in zip(starts, carriers)]
    lo, hi = g.limits_mm
    cands = np.arange(np.ceil(lo / STEP_MM) * STEP_MM, hi + 1e-9, STEP_MM)
    if ctx.checker is not None:            # it was cropped round the program's E1: stay near it
        cands = cands[np.abs(cands - (g.position_mm or 0.0)) <= MAX_SHIFT_MM]
    values = list(cur)
    changed = False

    def bad_in(p, k):
        return len(issues_in(p, ctx, starts[k], ends[k] - 1))     # fast: only layer k

    bad_layers = sorted({k for x in issues for k in range(len(starts)) if starts[k] <= x.move < ends[k]})
    for k in bad_layers:
        if carriers[k] is None:                 # sprays throughout: no gun-off move to change E1 on
            continue
        base = _with_e1(path, carriers, values, g.axis)
        find_issues(base, ctx)                  # joints up to this layer, with earlier layers' new E1
        best, best_n = values[k], bad_in(base, k)
        for e in sorted(cands, key=lambda e: abs(e - cur[k])):
            if e == values[k] or best_n == 0:
                continue
            trial = values[:k] + [float(e)] + values[k + 1:]
            n = bad_in(_with_e1(path, carriers, trial, g.axis), k)
            if n < best_n:
                best, best_n = float(e), n
        if best != values[k]:
            values[k], changed = best, True
    return _with_e1(path, carriers, values, g.axis) if changed else path
