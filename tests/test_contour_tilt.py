import numpy as np
import pytest
from shapely.geometry import Point, box

from coldspray.models import load_process
from coldspray.planner.contour_tilt import densify_corners, outward_normals, tilted_ring_poses
from coldspray.planner.layers import contour_loops, plan_layers
from coldspray.cad.slicer import Layer

TILT = 20.0


@pytest.fixture
def proc():
    return load_process().model_copy(update={"contour_tilt_deg": TILT})


def block_with_hole():
    return box(0, 0, 40, 30).difference(Point(20, 15).buffer(5, 64))


def _layer(region):
    return Layer(0, 10.0, 10.5, region)


def test_densify_adds_points_near_corners():
    ring = np.array([[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]], float)
    out = densify_corners(ring, 2.0)
    assert all(any(np.allclose(q, v) for q in out) for v in ring[:4])
    assert any(np.allclose(q, [1, 0]) for q in out) and any(np.allclose(q, [9, 0]) for q in out)
    assert not any(1.01 < q[0] < 8.99 for q in out if q[1] == 0)   # straight runs stay sparse


@pytest.mark.parametrize("width", [2.0, 5.0])
def test_normal_turns_at_most_10deg_per_point(width):
    region = box(0, 0, 40, 30).difference(Point(20, 15).buffer(6))
    for ring in [region.exterior.coords, *[r.coords for r in region.interiors]]:
        pts = densify_corners(np.array(ring), width)
        n = outward_normals(pts, region, width)
        turn = np.degrees(np.arccos(np.clip((n[:-1] * n[1:]).sum(1), -1, 1)))
        assert turn.max() <= 10.0


def test_normals_point_away_from_material(proc):
    region = block_with_hole()
    for ring in contour_loops(region, proc):
        n = outward_normals(densify_corners(ring, proc.track_width_mm), region, proc.track_width_mm)
        pts = densify_corners(ring, proc.track_width_mm)
        assert np.allclose(np.linalg.norm(n, axis=1), 1)
        edge = region.boundary
        assert all(edge.distance(Point(p + 0.5 * ni)) < edge.distance(Point(p)) for p, ni in zip(pts, n))


def test_poses_tilt_and_standoff(proc):
    region = block_with_hole()
    for ring in contour_loops(region, proc):
        pts, poses = tilted_ring_poses(ring, region, 10.0, proc, TILT)
        for p, T in zip(pts, poses):
            x = T[:3, 0]
            assert np.degrees(np.arccos(-x[2])) == pytest.approx(TILT, abs=0.5)
            aim = T[:3, 3] + proc.standoff_mm * x
            assert np.allclose(aim, [p[0], p[1], 10.0])
            # leans outward: the spray's horizontal part moves towards the nearest edge
            h = x[:2] / np.linalg.norm(x[:2])
            assert region.boundary.distance(Point(p + h)) < region.boundary.distance(Point(p))


def test_plan_layers_contour_tilted_infill_vertical(proc):
    path = plan_layers([_layer(block_with_hole())], proc)
    contour = [m for m in path.moves if m.tag == "contour"]
    infill = [m for m in path.moves if m.tag == "infill"]
    assert contour and infill
    for m in contour:
        assert np.degrees(np.arccos(-m.pose[2, 0])) == pytest.approx(TILT, abs=0.5)
    for m in infill:
        assert m.pose[2, 0] == pytest.approx(-1)


def test_zero_tilt_unchanged():
    base = load_process()
    zero = base.model_copy(update={"contour_tilt_deg": 0.0})
    a = plan_layers([_layer(block_with_hole())], base)
    b = plan_layers([_layer(block_with_hole())], zero)
    assert len(a.moves) == len(b.moves)
    assert all(np.array_equal(m.pose, n.pose) for m, n in zip(a.moves, b.moves))


def test_block_tilted_contours_reachable(proc):
    """Acceptance: test_block at 20 deg, contour poses 20 +- 0.5 deg from the table
    normal (the plan is in BASE, possibly with a tilted table: compare to infill),
    and auto-position still reaches the whole path."""
    from coldspray.kinematics.robot import Robot
    from coldspray.models import load_cell, load_robot, load_tool
    from coldspray.pipeline import load_part, plan_part
    from coldspray.planner.position import auto_position, layer_tilts
    cell = load_cell()
    part = load_part("test_block")
    plan = lambda tilt: plan_part(part, proc, "layers", tilt_deg=tilt, cell=cell)[1]
    path = plan(0.0)
    vertical = next(m.pose[:3, 0] for m in path.moves if m.tag == "infill")
    angles = [np.degrees(np.arccos(np.clip(m.pose[:3, 0] @ vertical, -1, 1))) for m in path.moves if m.tag == "contour"]
    assert angles and np.allclose(angles, TILT, atol=0.5)
    pos = auto_position(plan, Robot(load_robot(), load_tool()), cell, layer_tilts(cell), log=lambda _: None)
    assert pos.reach == pytest.approx(1.0)
