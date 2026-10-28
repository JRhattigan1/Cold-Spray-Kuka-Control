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
  purge    planner/solve_purge.py   move the purge point in X/Y/Z (spray direction kept)
  gantry   planner/solve_gantry.py  one gantry E1 for the whole program (suggested in the report)
  e1       planner/solve_e1.py      another gantry E1 for the layers that need it

Moves may carry the gantry in `axes` ("E1"); without it the cell's E1 is used.
"""

from dataclasses import dataclass, field
from typing import Callable, Literal

import numpy as np

from coldspray.cell import axis_defaults, base_world, robroot
from coldspray.toolpath.model import Toolpath, log_rot, slerp_mats


@dataclass
class SolveOptions:
    angle_band_deg: float = 30.0     # gun angle may move this far from the set angle
    lean_max_deg: float = 10.0       # gun lean forward/back along travel
    step_deg: float = 5.0            # search step for angle and lean
    ramp_mm: float = 20.0            # changes blend in/out over this path length
    reroute: bool = True
    e1_per_layer: bool = True
    gantry: bool = True              # one whole-program E1 (tried before e1 per layer)
    purge: bool = True               # move the purge point (not its spray direction)
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
    suggested: dict = field(default_factory=dict)          # e.g. {"gantry_e1": mm, "purge_point": (x, y, z)}

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
    names = sorted(samples.axes)
    values = np.column_stack([samples.axes[k] for k in names]) if names else np.zeros((len(samples), 0))
    keys, inverse = np.unique(values, axis=0, return_inverse=True)     # the table rarely moves: few distinct
    frames = []
    for row in keys:
        axes = dict(zip(names, map(float, row)))
        frames.append(np.linalg.inv(robroot(cell, e1_of(cell, axes))) @ base_world(cell, axes))
    return np.array(frames)[inverse.ravel()] @ samples.poses


def ptp_moves(path: Toolpath, cell) -> np.ndarray:
    """Per move: True if the KRL drives it as a PTP (post/krl.py): kind PTP, or E1 changes there."""
    out = np.array([m.kind == "PTP" for m in path.moves], bool)
    if cell.gantry is not None and len(out) > 1:
        e1 = [a.get(cell.gantry.axis, cell.gantry.position_mm) for a in path.axis_values()]
        out[1:] |= np.array(e1[1:]) != np.array(e1[:-1])
    return out


def ptp_in_joint_space(ptp, move_index, joints, before=None) -> np.ndarray:
    """`joints` (n, 6) of samples (`move_index` per sample) with the samples of PTP moves
    (`ptp` per move) put on a straight line in joint space - how the robot moves a PTP - from
    the joints before the move to its end joints (the end joints, which the KRL commands, are
    kept). `before`: joints before the first sample (else that segment is left as is).
    A segment with an unknown (NaN) end is left as is."""
    joints = np.array(joints, float)
    idx = np.asarray(move_index)
    for m in np.unique(idx[np.asarray(ptp)[idx]]):
        sel = np.flatnonzero(idx == m)
        q0 = joints[sel[0] - 1] if sel[0] > 0 else before
        q1 = joints[sel[-1]]
        if q0 is None or np.isnan(q0).any() or np.isnan(q1).any() or len(sel) < 2:
            continue
        t = (np.arange(1, len(sel) + 1) / len(sel))[:, None]
        joints[sel] = (1 - t) * np.asarray(q0, float) + t * q1
    return joints


def joint_sweep(q0, q1, step_deg: float = 5.0) -> np.ndarray:
    """Joints along a PTP from `q0` to `q1` (both ends excluded), at most `step_deg` apart:
    e.g. the KRL's HOME -> first point and last point -> HOME."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    if np.isnan(q0).any() or np.isnan(q1).any():
        return np.zeros((0, 6))
    n = max(1, int(np.ceil(np.abs(q1 - q0).max() / step_deg)))
    t = (np.arange(1, n) / n)[:, None]
    return (1 - t) * q0 + t * q1


def home_first_leg(robot, home, target0, q0, ptp: bool, step_mm: float = 5.0,
                   step_deg: float = 5.0) -> np.ndarray:
    """Joints along the KRL's HOME -> first point (both ends excluded), moved as the KRL
    issues it (post/krl.py): a PTP (joint line to `q0`) if move 0 is a PTP, else a LIN
    (TCP straight from HOME to `target0`, robot root frame; IK chained from HOME, NaN where
    unreachable). Empty if `q0` is unknown (NaN)."""
    if ptp:
        return joint_sweep(home, q0, step_deg)
    if np.isnan(np.asarray(q0, float)).any():
        return np.zeros((0, 6))
    p0, p1 = robot.fk(np.asarray(home, float)), np.asarray(target0, float)
    turn = np.degrees(np.linalg.norm(log_rot(p0[:3, :3].T @ p1[:3, :3])))
    n = max(1, int(np.ceil(max(np.linalg.norm(p1[:3, 3] - p0[:3, 3]) / step_mm, turn / step_deg))))
    t = np.arange(1, n) / n
    if not len(t):
        return np.zeros((0, 6))
    poses = np.tile(np.eye(4), (len(t), 1, 1))
    poses[:, :3, 3] = (1 - t)[:, None] * p0[:3, 3] + t[:, None] * p1[:3, 3]
    poses[:, :3, :3] = slerp_mats(p0[:3, :3], p1[:3, :3], t)
    return robot.ik_path(poses, seed_deg=home)


def _home_issues(path: Toolpath, ctx: Context, s, targets, joints) -> list[Issue]:
    """Issues on the KRL's HOME -> first point and last point -> HOME legs, reported at
    the first / last sample (one per leg and kind; detail names the leg)."""
    home = np.asarray(ctx.cell.home_deg, float)
    n, last_move = len(joints), int(s.move_index[-1])
    legs = [(0, 0, "HOME -> first" + ("" if path.moves[0].kind == "PTP" else " (LIN)"),
             home_first_leg(ctx.robot, home, targets[0], joints[0], path.moves[0].kind == "PTP",
                            ctx.opts.sample_mm, ctx.opts.angle_step_deg)),
            (n - 1, last_move, "last -> HOME", joint_sweep(joints[-1], home, ctx.opts.angle_step_deg))]
    out = []
    for i, move, name, qs in legs:
        if not len(qs):
            continue
        bad = np.isnan(qs[:, 0])
        if bad.any():
            out.append(Issue(i, move, "unreachable", name))
        if ctx.checker is not None:
            axes = s.axes_at(i)
            pairs = set()
            for q in qs[~bad]:
                d = _check(ctx, q, axes)
                if d is not None:
                    pairs |= set(d.split(", "))
            if pairs:
                out.append(Issue(i, move, "collision", f"{name}: {', '.join(sorted(pairs))}"))
    return out


def _sampled(path: Toolpath, ctx: Context):
    """(samples, robot targets, hint) of `path`, cached on ctx for the last few paths:
    levers and the after-check often sample the same path object more than once."""
    cache = ctx.__dict__.setdefault("_sample_cache", [])
    for p, val in cache:
        if p is path:
            return val
    o = ctx.opts
    s = path.sample(o.sample_mm, o.angle_step_deg, axis_defaults(ctx.cell))
    val = (s, robot_targets(s, ctx.cell), path.hint_for(s.move_index))
    cache.append((path, val))
    del cache[:-4]
    return val


def _check(ctx: Context, q, axes) -> str | None:
    """Colliding pairs at `q` / `axes` as Issue detail text (None: clear), cached on ctx:
    after a lever changes a few moves most samples keep exactly the same joints."""
    cache = ctx.__dict__.setdefault("_check_cache", {})
    if cache.get("__checker__") is not ctx.checker:
        cache.clear()
        cache["__checker__"] = ctx.checker
    key = (np.asarray(q, float).tobytes(), tuple(sorted(axes.items())))
    if key not in cache:
        pairs = ctx.checker.check(q, axes)
        cache[key] = ", ".join(f"{a}/{b}" for a, b in sorted(pairs)) if pairs else None
    return cache[key]


def find_issues(path: Toolpath, ctx: Context, limit: int | None = None):
    """(samples, joints, issues): joints NaN where unreachable; one Issue per bad sample,
    plus the KRL's HOME -> first and last -> HOME legs (at the first / last sample, detail
    naming the leg; only when the path starts from HOME, i.e. ctx.seed is None).
    With `limit`, collision checks stop once more than `limit` problem points are found
    (issues then holds limit + 1 of them): enough for levers that only compare counts.
    IK runs over the whole path and ctx.last is set either way."""
    s, targets, hint = _sampled(path, ctx)
    seed = ctx.cell.home_deg if ctx.seed is None else ctx.seed
    joints = ctx.robot.ik_path(targets, seed_deg=seed, hint=hint)
    joints = ptp_in_joint_space(ptp_moves(path, ctx.cell), s.move_index, joints)
    ctx.last = (s, joints)
    issues = [Issue(int(i), int(s.move_index[i]), "unreachable") for i in np.flatnonzero(np.isnan(joints[:, 0]))]
    if limit is not None and len(issues) > limit:
        return s, joints, issues[:limit + 1]
    if ctx.checker is not None:
        for i in np.flatnonzero(~np.isnan(joints[:, 0])):
            detail = _check(ctx, joints[i], s.axes_at(i))
            if detail is not None:
                issues.append(Issue(int(i), int(s.move_index[i]), "collision", detail))
                if limit is not None and len(issues) > limit:
                    break
    if ctx.seed is None and len(joints) and (limit is None or len(issues) <= limit):
        issues += _home_issues(path, ctx, s, targets, joints)       # the KRL's HOME legs
    issues.sort(key=lambda x: x.sample)
    return s, joints, issues


def issues_in(path: Toolpath, ctx: Context, lo: int, hi: int) -> list[Issue]:
    """Problem points of moves lo..hi only - the fast re-check levers use while
    searching. IK starts from the joints the last `find_issues` had just before
    move `lo`, so moves before `lo` must be unchanged since then (moves after
    `hi` may be added or removed). `solve` still re-checks the whole path."""
    s, all_targets, hint = _sampled(path, ctx)
    sel = np.flatnonzero((s.move_index >= lo) & (s.move_index <= hi))
    if not len(sel):
        return []
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
    out = []
    for i, q in zip(sel, joints):
        if np.isnan(q[0]):
            out.append(Issue(int(i), int(s.move_index[i]), "unreachable"))
        elif ctx.checker is not None:
            detail = _check(ctx, q, s.axes_at(i))
            if detail is not None:
                out.append(Issue(int(i), int(s.move_index[i]), "collision", detail))
    return out


LEVER_LABELS = {"arm": "arm / elbow choice", "angle": "gun angle band and lean",
                "reroute": "rerouting approach and jump moves", "e1": "E1 position per layer",
                "gantry": "one gantry E1 for the whole program", "purge": "moving the purge point"}


def _levers():
    """(name, fn, enabled(opts)) in the order they are tried; modules are imported
    lazily so a lever that isn't written yet is simply skipped."""
    out = []
    for name, module, attr, enabled in (
            ("arm", "solve_arm", "fix_arm", lambda o: True),
            ("angle", "solve_angle", "fix_angle", lambda o: o.angle_band_deg > 0 or o.lean_max_deg > 0),
            ("reroute", "solve_route", "fix_route", lambda o: o.reroute),
            ("purge", "solve_purge", "fix_purge", lambda o: o.purge),
            ("gantry", "solve_gantry", "fix_gantry", lambda o: o.gantry),
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
        ctx.log(f"Solver: trying {LEVER_LABELS.get(name, name)}...")
        ctx.suggested = {}               # a lever may leave a suggestion for the cell set-up here
        new = fn(path, issues, ctx)
        if new is path:
            continue
        _, _, after = find_issues(new, ctx)
        if len(after) < before:          # keep a lever's change only if it helped
            report.fixed.append(f"{name}: {before - len(after)} of {before} problem points fixed")
            report.suggested.update(getattr(ctx, "suggested", None) or {})
            ctx.log(f"Solver: {report.fixed[-1]}")
            path, issues = new, after
    report.unfixed = issues
    return path, report
