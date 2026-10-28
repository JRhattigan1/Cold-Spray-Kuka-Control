"""Time pipeline.plan_part on the test parts; --save/--check a pickled baseline of
every move (pose, flags, axes) so speed-ups can be checked for identical output.
Usage: python tools/bench_plan.py [--save F | --check F] [--profile CASE]"""
import argparse, cProfile, pickle, pstats, sys, time
import numpy as np
from coldspray.models import load_cell, load_process
from coldspray.pipeline import load_part, plan_part


def _repair_patch(part):
    f = {k: p for k, p in part.planar.items() if part.body_of_face(k) == part.build_body}
    return set(f) - {min(f, key=lambda k: f[k].normal[2])}


def _tube_outer(part):
    out = {}
    for f in set(part.face_ids.tolist()) - set(part.planar):
        v = part.vertices[np.unique(part.faces[part.face_ids == f])]
        out[f] = np.hypot(v[:, 0] - v[:, 0].mean(), v[:, 1] - v[:, 1].mean()).mean()
    r = max(out.values())
    return {f for f, d in out.items() if abs(d - r) < 1}


def _block_top(part):
    return {max(part.planar, key=lambda k: part.planar[k].normal[2])}


CASES = {  # name: (part, mode, tilt, face picker, layer thickness scale)
    "tube_layers": ("test_tube", "layers", 0.0, None, 1.0),
    "block_layers": ("test_block", "layers", 90.0, None, 1.0),
    "repair_layers": ("test_repair", "layers", 90.0, None, 1.0),
    "block_layers_tall": ("test_block", "layers", 0.0, None, 0.25),
    "tube_surface": ("test_tube", "surface", 0.0, _tube_outer, 1.0),
    "block_surface": ("test_block", "surface", 0.0, _block_top, 1.0),
    "repair_surface": ("test_repair", "surface", 0.0, _repair_patch, 1.0),
}


def run(name):
    pname, mode, tilt, pick, scale = CASES[name]
    part, proc, cell = load_part(pname), load_process(), load_cell()
    proc.layer_thickness_mm *= scale
    if pick:
        part.coat_faces = pick(part)
    return plan_part(part, proc, mode, tilt_deg=tilt, cell=cell, roll_deg=-135.0)[1]


def digest(path):
    return [(m.pose.copy(), m.speed, m.gun_on, m.kind, dict(m.axes), m.tag, m.dwell_s) for m in path.moves]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save"); ap.add_argument("--check"); ap.add_argument("--profile")
    a = ap.parse_args()
    if a.profile:
        run(a.profile)  # warm caches
        cProfile.run(f"run({a.profile!r})", "prof.out")
        pstats.Stats("prof.out").sort_stats("cumulative").print_stats(35)
        return
    out, bad = {}, 0
    ref = pickle.load(open(a.check, "rb")) if a.check else None
    for name in CASES:
        run(name)  # warm (mesh/STEP caches)
        t = time.perf_counter(); p = run(name); dt = time.perf_counter() - t
        out[name] = digest(p)
        same = ""
        if ref is not None:
            r, d = ref[name], out[name]
            ok = len(r) == len(d) and all(np.array_equal(x[0], y[0]) and x[1:] == y[1:] for x, y in zip(r, d))
            same = "IDENTICAL" if ok else "DIFFERENT"; bad += not ok
        print(f"{name:20s} {dt:7.3f} s  {len(p.moves):7d} moves  {same}")
    if a.save:
        pickle.dump(out, open(a.save, "wb"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
