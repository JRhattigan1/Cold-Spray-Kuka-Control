"""Time planner.solver.solve (collision checking on, headless) on small parts that have real problems.
Usage: python tools/bench_solve.py [--only NAME ...] [--profile] [--json out.json] [--scan]
Prints wall time, time per lever, and call counts (find_issues/issues_in/ik_path/checker.check).
--json saves SolveReport + output path poses (compare with tools/bench_solve.py --compare a.json b.json).
--scan lists issue counts per scenario before solving (to tune SCENARIOS). Env: QT_QPA_PLATFORM=offscreen."""
import argparse, cProfile, json, os, pstats, sys, time
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import numpy as np
from coldspray.cad.part import build_mesh, other_mesh
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner import solver
from coldspray.planner.gun import wrist_side
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_plan import _repair_patch, _tube_outer  # noqa: E402

# name: (part, mode, tilt_deg, roll_deg, face picker, E1 mm or None (cell default))
SCENARIOS = {
    "block_layers": ("test_block", "layers", 90.0, -135.0, None, None),
    "tube_surface": ("test_tube", "surface", 0.0, -135.0, _tube_outer, None),
    "repair_surface": ("test_repair", "surface", 0.0, -135.0, _repair_patch, None),
}
COUNTS = {}


def _count(name, fn):
    def w(*a, **k):
        COUNTS[name] = COUNTS.get(name, 0) + 1
        return fn(*a, **k)
    return w


def _install_counters():
    """Patch find_issues/issues_in everywhere they were imported, plus Robot.ik_path and the checker."""
    import importlib, pkgutil, coldspray.planner as pk
    from coldspray.sim.collision import CollisionChecker
    for m in pkgutil.iter_modules(pk.__path__):
        mod = importlib.import_module(f"coldspray.planner.{m.name}")
        for n in ("find_issues", "issues_in"):
            if hasattr(mod, n) and not getattr(getattr(mod, n), "_counted", False):
                f = _count(n, getattr(mod, n)); f._counted = True; setattr(mod, n, f)
    Robot.ik_path = _count("ik_path", Robot.ik_path)
    CollisionChecker.check = _count("checker.check", CollisionChecker.check)
    orig = solver._levers
    def levers():
        out = []
        for name, fn, en in orig():
            def timed(*a, _n=name, _f=fn, **k):
                t = time.perf_counter()
                try:
                    return _f(*a, **k)
                finally:
                    LEVER_T[_n] = LEVER_T.get(_n, 0.0) + time.perf_counter() - t
            out.append((name, timed, en))
        return out
    solver._levers = levers


LEVER_T = {}
_ASSETS = {}


def build(name):
    from coldspray.sim.collision import CollisionChecker, part_for_mode
    from coldspray.sim.scene import load_assets, make_gun
    pname, mode, tilt, roll, pick, e1 = SCENARIOS[name]
    cell, robot, proc = load_cell(), Robot(load_robot(), load_tool()), load_process()
    if e1 is not None:
        cell.gantry.position_mm = e1
    part = load_part(pname)
    if pick:
        part.coat_faces = pick(part)
    path = plan_part(part, proc, mode, tilt_deg=tilt, cell=cell, roll_deg=roll, wrist=wrist_side(robot.tool))[1]
    if "a" not in _ASSETS:
        _ASSETS["a"] = load_assets(log=lambda _: None)
    a = _ASSETS["a"]
    checker = CollisionChecker(robot, cell, a.meshes, make_gun(a.tool, a.gun_mesh), part_for_mode(mode, build_mesh(part)),
                               cell.robot_cad_pose_deg, fixture=other_mesh(part))
    return path, solver.Context(robot, cell, checker, log=lambda _: None)


def digest(path, report):
    return {"fixed": report.fixed, "unfixed": [[i.sample, i.move, i.kind, i.detail] for i in report.unfixed],
            "suggested": {k: list(v) if isinstance(v, (tuple, list, np.ndarray)) else v for k, v in report.suggested.items()},
            "poses": [m.pose.tolist() for m in path.moves], "gun_on": [bool(m.gun_on) for m in path.moves]}


def run(name, profile=False):
    COUNTS.clear(); LEVER_T.clear()
    path, ctx = build(name)
    pr = cProfile.Profile() if profile else None
    t = time.perf_counter()
    if pr: pr.enable()
    out, rep = solver.solve(path, ctx)
    if pr: pr.disable()
    dt = time.perf_counter() - t
    print(f"{name:16s} {dt:7.2f} s  moves={len(path.moves)} fixed={len(rep.fixed)} unfixed={len(rep.unfixed)}")
    print("   levers: " + ", ".join(f"{k} {v:.2f}s" for k, v in LEVER_T.items()) + "   calls: " + ", ".join(f"{k}={v}" for k, v in sorted(COUNTS.items())))
    if pr: pstats.Stats(pr).sort_stats("cumulative").print_stats(25)
    return dt, digest(out, rep)


def compare(fa, fb):
    a, b = json.load(open(fa)), json.load(open(fb)); bad = 0
    for k in a:
        x, y = a[k], b.get(k)
        ok = (y is not None and x["fixed"] == y["fixed"] and x["unfixed"] == y["unfixed"] and x["suggested"] == y["suggested"]
              and x["gun_on"] == y["gun_on"] and len(x["poses"]) == len(y["poses"])
              and np.allclose(np.array(x["poses"]), np.array(y["poses"]), atol=1e-9, rtol=0))
        print(k, "IDENTICAL" if ok else "DIFFERENT"); bad += not ok
    sys.exit(1 if bad else 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*"); ap.add_argument("--profile", action="store_true")
    ap.add_argument("--json"); ap.add_argument("--scan", action="store_true"); ap.add_argument("--compare", nargs=2)
    a = ap.parse_args()
    if a.compare: compare(*a.compare)
    _install_counters()
    names = a.only or list(SCENARIOS)
    if a.scan:
        for n in names:
            p, ctx = build(n); _, _, iss = solver.find_issues(p, ctx)
            print(n, len(iss), {k: sum(i.kind == k for i in iss) for k in ("unreachable", "collision")})
        return
    res, total = {}, 0.0
    for n in names:
        dt, res[n] = run(n, a.profile); total += dt
    print(f"TOTAL {total:.2f} s")
    if a.json: json.dump(res, open(a.json, "w"), default=lambda o: o.item() if hasattr(o, "item") else str(o))


if __name__ == "__main__":
    main()
