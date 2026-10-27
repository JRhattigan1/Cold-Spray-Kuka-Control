"""Warm-up / powder purge point: a fixed spot in free space where the gun
switches on and dwells until the powder flow settles, before it travels to
the part with the spray still on. It comes back at the end (and every N
layers) and switches off there, so the spray never starts or stops over the
part.

The point is fixed in the cell (WORLD) while the toolpath is in BASE, which
moves with the turntable, so each purge pose is converted with the table axes
in force at that point of the path.
"""

from dataclasses import replace

import numpy as np

from coldspray.cell import base_world
from coldspray.toolpath.model import Move, Toolpath, spray_frame


def purge_world(purge, back_mm: float = 0.0) -> np.ndarray:
    """TCP frame in WORLD at the purge point (or `back_mm` back along the spray)."""
    d = np.asarray(purge.spray_dir, float)
    d /= np.linalg.norm(d)
    side = (0.0, 0.0, 1.0) if abs(d[2]) < 0.9 else (1.0, 0.0, 0.0)
    return spray_frame(np.asarray(purge.point, float) - back_mm * d, d, side)


def layer_ends(path: Toolpath) -> list[int]:
    """Index of the retract that ends each layer: the first `approach` move
    after spraying that is followed by the next layer's approach, or ends the
    path (layers.py tags). A lone lift between strokes is not a layer end, and
    Surface coats are not layers (joined by turns). Paths without tags have none."""
    m = path.moves
    return [i for i in range(1, len(m))
            if m[i].tag == "approach" and m[i - 1].tag not in ("approach", "purge")
            and (i + 1 == len(m) or m[i + 1].tag == "approach")
            and any(x.gun_on for x in m[:i])]


def add_purge(path: Toolpath, cell, speed: float | None = None) -> Toolpath:
    """Copy of `path` with purge visits added (see the module docstring).
    `cell.purge` (if any, and enabled) gives point, spray_dir, dwell_s,
    at_start, at_end, every_n_layers, approach_mm. All moves from the first
    purge visit to the last keep the spray on."""
    purge = getattr(cell, "purge", None) if cell is not None else None
    if purge is None or not purge.enabled or not path.moves:
        return path
    axes = path.axis_values()
    speed = speed or max(m.speed for m in path.moves)
    at, back = purge_world(purge), purge_world(purge, purge.approach_mm)

    def to_base(world, i):
        return np.linalg.inv(base_world(cell, axes[i])) @ world

    def visit(i, dwell):
        return Move(to_base(at, i), speed, gun_on=True, tag="purge", dwell_s=dwell)

    moves = [replace(m, axes=dict(m.axes)) for m in path.moves]
    out, span = [], []
    if purge.at_start:
        out += [Move(to_base(back, 0), speed, kind="PTP", axes=dict(axes[0]), tag="purge"),
                visit(0, purge.dwell_s)]
        span.append(len(out) - 1)
    ends = set(layer_ends(path)[:-1]) if purge.every_n_layers else set()
    n = 0
    for i, m in enumerate(moves):
        out.append(m)
        if i in ends:
            n += 1
            if n % purge.every_n_layers == 0:
                out.append(visit(i, purge.dwell_s))
                span.append(len(out) - 1)
    if purge.at_end:
        out.append(visit(len(moves) - 1, 0.0))
        span.append(len(out) - 1)
    if span:
        first, last = min(span), max(span)
        if not purge.at_end:
            last = max(last, max((k for k, m in enumerate(out) if m.gun_on), default=last))
        if not purge.at_start:
            first = min(first, min((k for k, m in enumerate(out) if m.gun_on), default=first))
        for k in range(first, last + 1):
            out[k].gun_on = True
            if out[k].kind == "PTP":
                out[k].kind = "LIN"             # spray on: keep the gun on a straight line
    return Toolpath(path.name, out, path.skipped_points)
