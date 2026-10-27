import time

import numpy as np
import pytest

from coldspray.cad.constraints import (Axis, PointRef, align, angle, centre, centre_between, datum, distance,
                                       faceplate, fix, parallel, part_axis, part_plane, perpendicular, solve,
                                       reference, substrate_top, table_axis, touch)
from coldspray.cad.part import Substrate, constrain
from coldspray.pipeline import load_part
from coldspray.transforms import frame

TOP = Substrate().top


@pytest.fixture(scope="module")
def block():
    return load_part("test_block")   # 40 x 30 x 5 box at (10, 10, 0) with a 10 mm hole


@pytest.fixture(scope="module")
def tube():
    return load_part("test_tube")    # OD 120, axis Z through (0, 0) in part CAD, 150 long


@pytest.fixture(scope="module")
def repair():
    return load_part("test_repair")


def face(part, n):
    return next(i for i, f in part.planar.items() if np.allclose(f.normal, n, atol=1e-9))


def base(t, e):
    """Plane `e` (part coords) in BASE under placement t: (point, normal)."""
    return t[:3, :3] @ e.point + t[:3, 3], t[:3, :3] @ e.normal


def tilted(part, a=20, b=15, c=10):
    return part.placement @ frame(5, -3, 8, a, b, c)


def test_touch_floor(block):
    bottom = part_plane(block, face(block, (0, 0, -1)))
    r = solve(tilted(block), [touch(bottom, substrate_top(Substrate()))])
    assert r.status == "ok" and r.residuals[0] < 1e-6
    p, n = base(r.placement, bottom)
    assert np.allclose(n, (0, 0, -1), atol=1e-6) and p[2] == pytest.approx(TOP, abs=1e-6)
    assert r.dof_left == 3   # slide in XY, spin about Z


def test_align_and_distance(block):
    top = part_plane(block, face(block, (0, 0, 1)))
    r = solve(tilted(block), [align(top, faceplate(50))])
    p, n = base(r.placement, top)
    assert np.allclose(n, (0, 0, 1), atol=1e-6) and p[2] == pytest.approx(50, abs=1e-6)
    r = solve(tilted(block), [distance(top, datum("XY"), 30)])
    p, n = base(r.placement, top)
    assert abs(n[2]) == pytest.approx(1, abs=1e-6) and p[2] == pytest.approx(30, abs=1e-5)


def test_parallel_perpendicular_angle(block):
    side = part_plane(block, face(block, (1, 0, 0)))
    r = solve(tilted(block), [parallel(side, datum("YZ"))])
    assert abs(base(r.placement, side)[1][0]) == pytest.approx(1, abs=1e-6) and r.dof_left == 4
    r = solve(tilted(block), [perpendicular(side, datum("XY"))])
    assert base(r.placement, side)[1][2] == pytest.approx(0, abs=1e-6) and r.dof_left == 5
    r = solve(tilted(block), [angle(side, datum("XY"), 60)])
    assert base(r.placement, side)[1][2] == pytest.approx(0.5, abs=1e-6)


def test_centre_concentric(tube):
    axis = Axis((0, 0, 0), (0, 0, 1), moving=True)   # the tube's axis, part CAD
    bottom = part_plane(tube, face(tube, (0, 0, -1)))
    start = tube.placement @ frame(30, -20, 0, 0, 10, 5)
    r = solve(start, [centre(axis, table_axis()), touch(bottom, substrate_top(Substrate()))])
    assert r.status == "ok"
    t = r.placement
    assert np.allclose(t[:3, :3] @ axis.dir, (0, 0, 1), atol=1e-6)
    assert np.allclose(t[:2, 3], 0, atol=1e-5)
    assert r.dof_left == 1   # spin about its own axis


def test_centre_point_on_axis(block):
    hole_centre = PointRef((30, 25, 0), moving=True)
    r = solve(tilted(block), [centre(hole_centre, table_axis())])
    p = r.placement[:3, :3] @ hole_centre.point + r.placement[:3, 3]
    assert np.allclose(p[:2], 0, atol=1e-6) and r.dof_left == 4


def test_part_axis_uses_cylinders(tube):
    cyl = getattr(tube, "cylinders", None)
    if not cyl:
        pytest.skip("Part.cylinders not available yet (WP0)")
    outer = max(cyl, key=lambda k: cyl[k].radius)
    r = solve(tube.placement @ frame(20, 0, 0, 0, 10, 0), [centre(part_axis(tube, outer), table_axis())])
    assert r.status == "ok" and r.dof_left == 2


def test_centre_between(repair):
    # the component (120 long) and the patch both have -X / +X faces; take the outermost
    ids = [i for i, f in repair.planar.items() if abs(f.normal[0]) > 0.99]
    lo = min(ids, key=lambda i: repair.planar[i].point[0])
    hi = max(ids, key=lambda i: repair.planar[i].point[0])
    left, right = part_plane(repair, lo), part_plane(repair, hi)
    r = solve(repair.placement @ frame(17, 0, 0, 0, 0, 0), [centre_between(left, right, datum("YZ"))])
    assert r.status == "ok"
    xl, xr = base(r.placement, left)[0][0], base(r.placement, right)[0][0]
    assert (xl + xr) / 2 == pytest.approx(0, abs=1e-5) and xr - xl == pytest.approx(120, abs=1e-5)


def test_fix_locks_everything(block):
    r = solve(tilted(block), [fix(block.placement)])
    assert np.allclose(r.placement, block.placement, atol=1e-6) and r.dof_left == 0


def test_conflict(block):
    bottom = part_plane(block, face(block, (0, 0, -1)))
    r = solve(block.placement, [touch(bottom, faceplate()), touch(bottom, faceplate(20))])
    assert r.status == "conflicting" and max(r.residuals) > 1e-3


def test_over_constrained_but_consistent(block):
    bottom = part_plane(block, face(block, (0, 0, -1)))
    top = part_plane(block, face(block, (0, 0, 1)))
    r = solve(tilted(block), [touch(bottom, faceplate(TOP)), align(top, faceplate(TOP + 5))])
    assert r.status == "ok" and r.redundant and r.dof_left == 3


def test_suppressed_ignored(block):
    bottom = part_plane(block, face(block, (0, 0, -1)))
    k = touch(bottom, faceplate())
    k.suppressed = True
    r = solve(block.placement, [k])
    assert np.allclose(r.placement, block.placement) and r.dof_left == 6 and r.residuals == [0.0]


def test_keep_on_plate_matches_constrain(block):
    block = load_part("test_block")
    block.base_face = face(block, (0, 0, -1))
    start = tilted(block)
    want = constrain(block, start, TOP)
    r = solve(start, [touch(part_plane(block, block.base_face), substrate_top(Substrate()))])
    assert np.allclose(r.placement, want, atol=1e-4)


def test_fast(repair):
    bottom = part_plane(repair, face(repair, (0, 0, -1)))
    side = part_plane(repair, face(repair, (1, 0, 0)))
    ks = [touch(bottom, substrate_top(Substrate())), parallel(side, datum("YZ")),
          centre(PointRef((60, 40, 0), moving=True), table_axis())]
    start = repair.placement @ frame(10, 5, 3, 10, 5, 5)
    solve(start, ks)
    t0 = time.perf_counter()
    r = solve(start, ks)
    assert time.perf_counter() - t0 < 0.05 and r.status == "ok" and r.dof_left == 0


def test_reference_names(block):
    fid = face(block, (0, 0, -1))
    assert reference(f"face:{fid}", block).moving
    assert np.allclose(reference("datum:XZ", offset=5).point, (0, 5, 0))
    assert reference("substrate", substrate=Substrate()).point[2] == TOP
    assert reference("faceplate").point[2] == 0 and np.allclose(reference("table_axis").dir, (0, 0, 1))
    with pytest.raises(ValueError):
        reference("nope")


def test_touch_from_180_degree_flip(block):
    """R2: the top face already pointing up must still be turned over onto the floor."""
    top = part_plane(block, face(block, (0, 0, 1)))
    r = solve(block.placement, [touch(top, substrate_top(Substrate()))])
    assert r.status == "ok"
    assert np.allclose(base(r.placement, top)[1], (0, 0, -1), atol=1e-4)


def test_reference_rejects_curved_non_cylinder_face(block):
    import dataclasses
    fid = int(block.face_ids[0])
    bare = dataclasses.replace(block, planar={}, cylinders={})
    with pytest.raises(ValueError, match=f"face {fid} is not flat or cylindrical"):
        reference(f"face:{fid}", bare)


def test_wrong_entity_type_is_a_clear_error():
    ax, pl = table_axis(), faceplate()
    with pytest.raises(ValueError, match="touch.*plane"):
        touch(ax, pl)
    with pytest.raises(ValueError, match="centre_between.*plane"):
        centre_between(pl, ax, pl)
    with pytest.raises(ValueError, match="centre.*axis"):
        centre(pl, ax)
    with pytest.raises(ValueError, match="parallel.*plane or axis"):
        parallel(PointRef((0, 0, 0)), pl)
    k = touch(pl, pl)
    k.a = ax                                  # changed after building: caught by solve
    with pytest.raises(ValueError, match="touch.*plane"):
        solve(np.eye(4), [k])
