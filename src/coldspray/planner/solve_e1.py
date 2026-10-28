"""Lever `e1`: move the gantry (E1) per layer to a position where the layer is clear.

For each layer (planner.purge.layer_ends) that has problem points, other E1 values
inside cell.gantry.limits_mm are tried in 250 mm steps, nearest the current one
first; the first that clears the layer is kept (else the one with fewest problems,
if it helps). The value is written to `axes["E1"]` of the layer's first move with
the gun off (the KRL makes an E1 change a PTP, and a PTP must never spray), and
E1 is stated on every layer from then on so it never leaks into the next layer.
A layer that sprays on every move gets no E1 of its own (it keeps the last one), so it is
scored with the layer whose E1 it keeps; moves before a layer's carrier are not scored for it. A path with nothing to fix, or a cell without a gantry, comes back as is.
"""

from dataclasses import replace

import numpy as np

from coldspray.planner.purge import layer_ends
from coldspray.planner.solve_route import count_in, prime_seed
from coldspray.planner.solver import Context, e1_of

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
    carriers = e1_carriers(path, starts)
    axes = path.axis_values()
    cur = [e1_of(ctx.cell, axes[s if c is None else c]) for s, c in zip(starts, carriers)]
    lo, hi = g.limits_mm
    cands = np.arange(np.ceil(lo / STEP_MM) * STEP_MM, hi + 1e-9, STEP_MM)
    if ctx.checker is not None:            # it was cropped round the program's E1: stay near it
        built = getattr(ctx.checker, "e1", None)
        cands = cands[np.abs(cands - (g.position_mm if built is None else built)) <= MAX_SHIFT_MM]
    values = list(cur)
    changed = False

    # Layer k's E1 holds from its carrier until the next layer that has one (a layer that sprays
    # throughout keeps it); moves before the carrier run on the previous E1, which k can't change.
    span = {}
    for k, c in enumerate(carriers):
        if c is not None:
            span[k] = (c, next((x for x in carriers[k + 1:] if x is not None), len(path.moves)) - 1)

    def bad_in(p, k, limit=None):
        return count_in(p, ctx, *span[k], limit=limit)   # fast: only the moves layer k's E1 governs; stops at `limit`

    bad_layers = sorted({k for x in issues for k, (a, b) in span.items() if a <= x.move <= b})
    for k in bad_layers:
        base = _with_e1(path, carriers, values, g.axis)
        prime_seed(base, ctx, span[k][0])        # joints up to this layer, with earlier layers' new E1
        best, best_n = values[k], bad_in(base, k)
        for e in sorted(cands, key=lambda e: abs(e - cur[k])):
            if e == values[k] or best_n == 0:
                continue
            trial = values[:k] + [float(e)] + values[k + 1:]
            n = bad_in(_with_e1(path, carriers, trial, g.axis), k, best_n)      # >= best_n: not better, whatever it is
            if n < best_n:
                best, best_n = float(e), n
        if best != values[k]:
            values[k], changed = best, True
    return _with_e1(path, carriers, values, g.axis) if changed else path
