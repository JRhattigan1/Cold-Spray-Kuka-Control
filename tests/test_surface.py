import numpy as np
import pytest
import trimesh

from coldspray.cad.part import placed_vertices
from coldspray.models import load_process
from coldspray.pipeline import load_part
from coldspray.planner.surface import plan_surface


def _face_mask(part, pick):
    """Triangles of the CAD faces chosen by `pick(centroid_xyz, normal)`."""
    v = placed_vertices(part)
    tri = v[part.faces]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    hit = pick(tri.mean(1), n)
    chosen = set(np.unique(part.face_ids[hit]))
    return v, np.isin(part.face_ids, list(chosen))


def test_block_top_is_a_flat_plate_raster():
    proc = load_process()
    part = load_part("test_block")
    v = placed_vertices(part)
    top = v[:, 2].max()
    v, mask = _face_mask(part, lambda c, n: (n[:, 2] > 0.99) & np.isclose(c[:, 2], top))
    path = plan_surface(v, part.faces, mask, proc)
    spray = [m for i, m in enumerate(path.moves) if m.gun_on
             and not (m.tag == "turn" and path.moves[i - 1].tag != "turn")]   # skip lifted mid-turns
    pts = np.array([m.pose[:3, 3] for m in spray])
    assert np.allclose(pts[:, 2], top + proc.standoff_mm, atol=1e-6)
    assert all(np.allclose(m.pose[:3, 0], [0, 0, -1], atol=1e-6) for m in spray)
    # 40 x 30 top: strokes along X (longest), passes step_over apart across Y, overrun past both edges
    sel = v[part.faces[mask]].reshape(-1, 3)
    xmin, xmax = sel[:, 0].min(), sel[:, 0].max()
    assert np.isclose(pts[:, 0].min(), xmin - proc.overrun_mm, atol=1e-6)
    assert np.isclose(pts[:, 0].max(), xmax + proc.overrun_mm, atol=1e-6)
    ys = np.unique(np.round(pts[[m.tag == "infill" for m in spray], 1], 6))
    assert np.allclose(np.diff(ys), proc.step_over_mm) and len(ys) == 15
    assert path.skipped_points == 0
    assert path.moves[0].kind == "PTP" and path.moves[0].tag == "approach"


def test_tube_outer_cylinder_at_standoff_along_normal():
    proc = load_process()
    part = load_part("test_tube")
    v = placed_vertices(part)
    axis = v[:, :2].mean(0)
    v, mask = _face_mask(part, lambda c, n: (np.abs(n[:, 2]) < 0.1)
                         & (np.linalg.norm(c[:, :2] - axis, axis=1) > 50))
    path = plan_surface(v, part.faces, mask, proc)
    spray = [m for m in path.moves if m.gun_on and m.tag == "infill"]
    assert spray
    for m in spray:
        p, x = m.pose[:3, 3], m.pose[:3, 0]
        r = p[:2] - axis
        assert np.linalg.norm(r) == pytest.approx(60 + proc.standoff_mm, abs=0.3)
        assert np.dot(x[:2], -r / np.linalg.norm(r)) > 0.999 and abs(x[2]) < 1e-3


def test_box_edge_strokes_cross_continuously_with_smooth_normals():
    proc = load_process()
    box = trimesh.creation.box([40, 30, 20])
    n = box.face_normals
    mask = (n[:, 2] > 0.9) | (n[:, 0] > 0.9)           # top and +X side share an edge
    path = plan_surface(box.vertices, box.faces, mask, proc, max_tilt_deg=90.0)
    moves = path.moves
    stroke = []
    for m in moves[2:]:
        if m.tag != "infill":
            break
        stroke.append(m)
    x = np.array([m.pose[:3, 0] for m in stroke])
    angles = np.degrees(np.arccos(np.clip(np.sum(x[1:] * x[:-1], axis=1), -1, 1)))
    assert angles.max() < 15                         # no snap at the edge
    assert x[:, 2].min() < -0.99 and x[:, 0].min() < -0.99   # spans top (down) and side (-X)
    p = np.array([m.pose[:3, 3] for m in stroke])
    assert np.linalg.norm(np.diff(p, axis=0), axis=1)[:-1].max() < 10   # last step is the overrun


def test_coats_and_tilt_limit():
    proc = load_process()
    box = trimesh.creation.box([40, 30, 20])
    mask = (box.face_normals[:, 2] > 0.9) | (box.face_normals[:, 0] > 0.9)
    one = plan_surface(box.vertices, box.faces, mask, proc, max_tilt_deg=90.0)
    two = plan_surface(box.vertices, box.faces, mask, proc, coats=2, max_tilt_deg=90.0)
    n1 = sum(m.gun_on for m in one.moves)
    assert sum(m.gun_on for m in two.moves) == 2 * n1 + 2   # one extra (lifted, 2-move) turn between coats
    tight = plan_surface(box.vertices, box.faces, mask, proc, max_tilt_deg=20.0)
    assert tight.skipped_points > 0
    assert sum(m.gun_on for m in tight.moves) < n1


def test_turns_lift_off_convex_surface_by_standoff():
    proc = load_process()
    part = load_part("test_tube")
    v = placed_vertices(part)
    axis = v[:, :2].mean(0)
    v, mask = _face_mask(part, lambda c, n: (np.abs(n[:, 2]) < 0.1)
                         & (np.linalg.norm(c[:, :2] - axis, axis=1) > 50))
    path = plan_surface(v, part.faces, mask, proc)
    moves = path.moves
    mids = [m for i, m in enumerate(moves[1:-1], 1)
            if m.tag == "turn" and moves[i - 1].tag != "turn"]
    assert mids
    for m in mids:
        r = np.linalg.norm(m.pose[:3, 3][:2] - axis)
        assert r >= 60 + 2 * proc.standoff_mm - 0.5      # lifted a further standoff mid-turn
