"""Fix unreachable and colliding points after planning.

`find_issues` samples the path the way Simulate does, solves IK (joint limits
are always respected: a pose with no in-limit solution is unreachable) and,
given a collision checker, checks every reachable sample. `solve` then tries
the levers in LEVERS order, cheapest first, re-checking after each; it stops
as soon as nothing is left. What no lever can fix is reported and left as
planned - the export stays blocked until it is dealt with.

A lever is `fn(path, issues, ctx) -> Toolpath`: it returns a changed copy, or
the same path if it can't help. Each lives in its own module:
  arm      planner/solve_arm.py     arm/wrist configuration along the path (no A4/A6 wind-up)
  angle    planner/solve_angle.py   gun angle within +-angle_band_deg, and lean along travel
  reroute  planner/solve_route.py   lift / go round on approach, jump and purge moves
  e1       planner/solve_e1.py      another gantry E1 for the layers that need it

Moves may carry the gantry in `axes` ("E1"); without it the cell's E1 is used.
"""

from dataclasses import dataclass, field
from typing import Callable, Literal

import numpy as np

from coldspray.cell import base_world, robroot
from coldspray.toolpath.model import Toolpath


@dataclass
class SolveOptions:
    angle_band_deg: float = 30.0     # gun angle may move this far from the set angle
    lean_max_deg: float = 10.0       # gun lean forward/back along travel
    step_deg: float = 5.0            # search step for angle and lean
    ramp_mm: float = 20.0            # changes blend in/out over this path length
    reroute: bool = True
    e1_per_layer: bool = True
    sample_mm: float = 5.0           # path sampling, as Simulate
    angle_step_deg: float = 5.0


@dataclass
class Issue:
    sample: int                      # index into path.sample(...)
    move: int                        # path move the sample belongs to
    kind: Literal["unreachable", "collision"]
    detail: str = ""                 # e.g. colliding pairs


@dataclass
class SolveReport:
    fixed: list[str] = field(default_factory=list)      # one line per lever that changed the path
    unfixed: list[Issue] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.unfixed


@dataclass
class Context:
    robot: object
    cell: object
    checker: object | None = None    # sim.collision.CollisionChecker, or None (reach only)
    opts: SolveOptions = field(default_factory=SolveOptions)
    seed: np.ndarray | None = None   # joints to start from (default: cell home)
    log: Callable[[str], None] = print


def e1_of(cell, axes: dict) -> float | None:
    """Gantry position for a sample: from its axes if set there, else the cell's."""
    if cell.gantry is None:
        return None
    return axes.get(cell.gantry.axis, cell.gantry.position_mm)


def robot_targets(samples, cell) -> np.ndarray:
    """TCP of every sample in the robot root frame (gantry and table where each sample puts them)."""
    out = []
    for i in range(len(samples)):
        axes = samples.axes_at(i)
        out.append(np.linalg.inv(robroot(cell, e1_of(cell, axes))) @ base_world(cell, axes) @ samples.poses[i])
    return np.array(out)


def find_issues(path: Toolpath, ctx: Context):
    """(samples, joints, issues): joints NaN where unreachable; one Issue per bad sample."""
    o = ctx.opts
    s = path.sample(o.sample_mm, o.angle_step_deg)
    seed = ctx.cell.home_deg if ctx.seed is None else ctx.seed
    joints = ctx.robot.ik_path(robot_targets(s, ctx.cell), seed_deg=seed)
    issues = [Issue(i, int(s.move_index[i]), "unreachable") for i in np.flatnonzero(np.isnan(joints[:, 0]))]
    if ctx.checker is not None:
        for i in np.flatnonzero(~np.isnan(joints[:, 0])):
            pairs = ctx.checker.check(joints[i], s.axes_at(i))
            if pairs:
                issues.append(Issue(int(i), int(s.move_index[i]), "collision",
                                    ", ".join(f"{a}/{b}" for a, b in sorted(pairs))))
    issues.sort(key=lambda x: x.sample)
    return s, joints, issues


def _levers():
    """(name, fn, enabled(opts)) in the order they are tried; modules are imported
    lazily so a lever that isn't written yet is simply skipped."""
    out = []
    for name, module, attr, enabled in (
            ("arm", "solve_arm", "fix_arm", lambda o: True),
            ("angle", "solve_angle", "fix_angle", lambda o: o.angle_band_deg > 0 or o.lean_max_deg > 0),
            ("reroute", "solve_route", "fix_route", lambda o: o.reroute),
            ("e1", "solve_e1", "fix_e1", lambda o: o.e1_per_layer)):
        try:
            mod = __import__(f"coldspray.planner.{module}", fromlist=[attr])
        except ImportError:
            continue
        out.append((name, getattr(mod, attr), enabled))
    return out


def solve(path: Toolpath, ctx: Context) -> tuple[Toolpath, SolveReport]:
    """Try each lever in turn on what is still wrong; returns the (maybe changed)
    path and a report. With nothing wrong the path comes back unchanged."""
    report = SolveReport()
    _, _, issues = find_issues(path, ctx)
    for name, fn, enabled in _levers():
        if not issues or not enabled(ctx.opts):
            continue
        before = len(issues)
        new = fn(path, issues, ctx)
        if new is path:
            continue
        _, _, after = find_issues(new, ctx)
        if len(after) < before:          # keep a lever's change only if it helped
            report.fixed.append(f"{name}: {before - len(after)} of {before} problem points fixed")
            ctx.log(f"Solver: {report.fixed[-1]}")
            path, issues = new, after
    report.unfixed = issues
    return path, report
