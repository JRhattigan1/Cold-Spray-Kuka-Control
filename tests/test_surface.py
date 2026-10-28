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


def test_turns_lift_by_jump_height():
    """Mid-turn the gun lifts a further jump_height_mm off the surface; at 0 there is no lift."""
    part = load_part("test_tube")
    v = placed_vertices(part)
    axis = v[:, :2].mean(0)
    v, mask = _face_mask(part, lambda c, n: (np.abs(n[:, 2]) < 0.1)
                         & (np.linalg.norm(c[:, :2] - axis, axis=1) > 50))
    for jump in (20.0, 0.0):
        proc = load_process().model_copy(update={"jump_height_mm": jump})
        moves = plan_surface(v, part.faces, mask, proc).moves
        turns = [m for m in moves if m.tag == "turn"]
        assert turns
        r = [np.linalg.norm(m.pose[:3, 3][:2] - axis) for m in turns]
        assert max(r) >= 60 + proc.standoff_mm + jump - 0.5     # the highest turn point
        assert max(r) <= 60 + proc.standoff_mm + jump + 0.5


def test_separate_faces_are_separate_regions_with_a_jump():
    """The U-block's two prong tops, 20 mm apart, are coated one after the other: no
    stroke runs across the slot, and the gun jumps between them."""
    from coldspray.models import load_process
    from coldspray.pipeline import load_part, plan_part
    from coldspray.planner.surface import coat_regions
    part = load_part("test_u_block")
    up = part.placement[:3, :3].T @ [0.0, 0.0, 1.0]
    top = max(f.point @ up for f in part.planar.values())
    prongs = {k for k, f in part.planar.items() if f.normal @ up > 0.99 and np.isclose(f.point @ up, top)}
    assert len(prongs) == 2
    part.coat_faces = prongs
    mask = np.isin(part.face_ids, sorted(prongs))
    assert len(coat_regions(part.vertices, part.faces, mask)) == 2
    proc = load_process().model_copy(update={"overrun_mm": 0.0})   # stroke ends: no deliberate overrun
    path = plan_part(part, proc, "surface")[1]
    tags = [m.tag for m in path.moves]
    assert "jump" in tags
    # No infill stroke (merged LINs are long) crosses the slot: every point along every
    # infill segment lies over a prong top (in plan view, 0.5 mm margin at the edges).
    v = placed_vertices(part)
    tri = v[part.faces[mask]][:, :, :2]                                   # (T, 3, 2) prong triangles
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    det = (b - a)[:, 0] * (c - a)[:, 1] - (b - a)[:, 1] * (c - a)[:, 0]

    def over_prong(p):
        d = p[:, None, :] - a[None]
        u = (d[..., 0] * (c - a)[:, 1] - d[..., 1] * (c - a)[:, 0]) / det
        w = ((b - a)[:, 0] * d[..., 1] - (b - a)[:, 1] * d[..., 0]) / det
        return ((u >= -0.05) & (w >= -0.05) & (u + w <= 1.05)).any(axis=1)

    segs = [(a_.pose[:3, 3], b_.pose[:3, 3]) for a_, b_ in zip(path.moves, path.moves[1:])
            if a_.tag == b_.tag == "infill"]
    assert segs
    t = np.linspace(0, 1, 50)[:, None]
    for p0, p1 in segs:
        assert over_prong((p0 + t * (p1 - p0))[:, :2]).all()


def _tube_end(part):
    v = placed_vertices(part)
    top = v[:, 2].max()
    return _face_mask(part, lambda c, n: (n[:, 2] > 0.99) & np.isclose(c[:, 2], top)) + (top,)


@pytest.mark.parametrize("pattern", ["oneway", "concentric", "spiral", "islands"])
def test_patterns_fill_a_flat_face(pattern):
    """Any infill pattern on the tube's annular end: contour then pattern, gun straight
    down at the standoff, and no spray over the bore (only overruns and jumps cross it)."""
    proc = load_process().model_copy(update={"infill_pattern": pattern})
    part = load_part("test_tube")
    v, mask, top = _tube_end(part)
    axis = v[:, :2].mean(0)
    path = plan_surface(v, part.faces, mask, proc)
    assert path.zigzag_regions == 0 and path.skipped_points == 0
    tags = [m.tag for m in path.moves]
    assert "contour" in tags and "infill" in tags and tags.index("contour") < tags.index("infill")
    sprayed = [m for m in path.moves if m.tag in ("contour", "infill")]
    assert all(np.allclose(m.pose[:3, 0], [0, 0, -1], atol=1e-6) for m in sprayed)
    assert all(abs(m.pose[2, 3] - top - proc.standoff_mm) < 1e-6 for m in sprayed)
    r = np.array([np.linalg.norm(m.pose[:2, 3] - axis) for m in sprayed])
    if pattern in ("concentric", "spiral", "islands"):          # no overrun: every point on the face
        assert r.min() > 40 and r.max() < 60
    else:                                                       # raster: only the overrun leaves it
        on = r[(r > 40) & (r < 60)]
        assert len(on) > 0.8 * len(r)


def test_concentric_rings_on_the_tube_end():
    proc = load_process().model_copy(update={"infill_pattern": "concentric"})
    part = load_part("test_tube")
    v, mask, _ = _tube_end(part)
    axis = v[:, :2].mean(0)
    path = plan_surface(v, part.faces, mask, proc)
    r = np.array([np.linalg.norm(m.pose[:2, 3] - axis) for m in path.moves if m.tag in ("contour", "infill")])
    rings = np.unique(np.round(r))       # contour 2 mm in, then step-over apart from both edges
    assert {42, 44, 46, 48, 52, 54, 56, 58} <= set(rings.astype(int))
    assert sum(t == "jump" for t in (m.tag for m in path.moves)) < 30    # no slivers at the centre line


def test_wrapping_region_falls_back_to_zigzag():
    """The whole tube side can't be flattened: it is zigzagged as before, and counted."""
    part = load_part("test_tube")
    v = placed_vertices(part)
    axis = v[:, :2].mean(0)
    v, mask = _face_mask(part, lambda c, n: (np.abs(n[:, 2]) < 0.1)
                         & (np.linalg.norm(c[:, :2] - axis, axis=1) > 50))
    zig = plan_surface(v, part.faces, mask, load_process())
    conc = plan_surface(v, part.faces, mask, load_process().model_copy(update={"infill_pattern": "concentric"}))
    assert conc.zigzag_regions == 1 and zig.zigzag_regions == 0
    assert len(conc.moves) == len(zig.moves)


@pytest.mark.parametrize("angle", [0.0, 30.0, 90.0])
def test_raster_start_angle(angle):
    """raster_start_deg turns the first raster from the default direction, in Layers
    patterns (from BASE X) and in the Surface zigzag (from the longest side)."""
    from shapely.geometry import box as rect
    from coldspray.planner.patterns import zigzag
    proc = load_process().model_copy(update={"raster_start_deg": angle})
    seg = zigzag(rect(-50, -40, 50, 40), proc, 0)
    d = seg[0][-1] - seg[0][0]
    assert np.isclose(abs(np.degrees(np.arctan2(d[1], d[0]))) % 180, angle % 180, atol=1e-6) or \
        np.isclose(180 - abs(np.degrees(np.arctan2(d[1], d[0]))), angle, atol=1e-6)
    top = trimesh.creation.box([40, 30, 20])
    path = plan_surface(top.vertices, top.faces, top.face_normals[:, 2] > 0.9,
                        proc.model_copy(update={"overrun_mm": 0.0}))
    y = np.array([m.pose[:3, 1] for m in path.moves if m.tag == "infill"])     # gun Y: along the stroke
    a = np.degrees(np.arctan2(y[:, 1], y[:, 0])) % 180
    assert np.allclose(np.minimum(a, 180 - a), min(angle % 180, 180 - angle % 180), atol=1e-6)


def _tube_side_and_end(part):
    v = placed_vertices(part)
    axis, top = v[:, :2].mean(0), v[:, 2].max()
    return _face_mask(part, lambda c, n: ((np.abs(n[:, 2]) < 0.1) & (np.linalg.norm(c[:, :2] - axis, axis=1) > 50))
                      | ((n[:, 2] > 0.99) & np.isclose(c[:, 2], top)))


def _blocks(path):
    """Runs of gun-on moves (raster blocks), split at gun-off moves."""
    out, cur = [], []
    for m in path.moves:
        if m.gun_on:
            cur.append(m)
        elif cur:
            out.append(cur)
            cur = []
    return out + ([cur] if cur else [])


@pytest.mark.parametrize("pattern", ["zigzag", "concentric"])
def test_wrapping_pick_is_rastered_face_by_face(pattern):
    """The tube's outer side and its end face picked together (one connected region that
    wraps round): one raster block per CAD face, gun off between them, and no stroke
    runs over the edge from the side onto the end."""
    from coldspray.pipeline import same_surface_ids
    part = load_part("test_tube")
    v, mask = _tube_side_and_end(part)
    assert len(np.unique(part.face_ids[mask])) == 2
    proc = load_process().model_copy(update={"infill_pattern": pattern})
    path = plan_surface(v, part.faces, mask, proc, face_ids=same_surface_ids(part))
    blocks = _blocks(path)
    if pattern == "zigzag":
        assert len(blocks) == 2
    kinds = []
    for b in blocks:
        z = np.array([m.pose[2, 0] for m in b if m.tag in ("infill", "contour")])   # spray direction, Z
        if not len(z):
            continue
        end, side = np.all(z < -0.999), np.all(np.abs(z) < 1e-3)
        assert end or side                         # every block on one face: no stroke over the edge
        kinds.append("end" if end else "side")
    assert set(kinds) == {"end", "side"}
    assert kinds == sorted(kinds, key=kinds.index) and kinds.count(kinds[0]) + kinds.count(kinds[-1]) == len(kinds) \
        or kinds[0] != kinds[-1]
    # each face once, in one stretch: the sequence changes face exactly once
    assert sum(a != b for a, b in zip(kinds, kinds[1:])) == 1
    sw = [i for i, m in enumerate(path.moves) if not m.gun_on and m.tag == "jump"]
    assert sw and all(m.tag == "jump" for m in (path.moves[i] for i in sw))
    # without face ids the old behaviour: strokes blend over the edge in one zigzag
    if pattern == "zigzag":
        old = plan_surface(v, part.faces, mask, proc)
        assert len(_blocks(old)) == 1


def test_single_face_pick_unchanged_by_face_ids():
    """Picks on one CAD face plan exactly as without face ids."""
    from coldspray.pipeline import same_surface_ids
    part = load_part("test_tube")
    ids = same_surface_ids(part)
    axis = placed_vertices(part)[:, :2].mean(0)
    side = _face_mask(part, lambda c, n: (np.abs(n[:, 2]) < 0.1) & (np.linalg.norm(c[:, :2] - axis, axis=1) > 50))
    for v, mask in (_tube_end(part)[:2], side):
        a = plan_surface(v, part.faces, mask, load_process())
        b = plan_surface(v, part.faces, mask, load_process(), face_ids=ids)
        assert len(a.moves) == len(b.moves)
        assert all(np.array_equal(x.pose, y.pose) and x.gun_on == y.gun_on and x.tag == y.tag
                   for x, y in zip(a.moves, b.moves))


def test_oneway_overrun_measured_from_the_face_edge():
    """A oneway infill sits inside the contour passes, but its strokes run on to the
    face's edge and then overrun_mm past it (not overrun_mm past the inset)."""
    proc = load_process().model_copy(update={"infill_pattern": "oneway", "contour_passes": 2,
                                             "raster_start_deg": 0.0})
    top = trimesh.creation.box([40, 30, 20])
    path = plan_surface(top.vertices, top.faces, top.face_normals[:, 2] > 0.9, proc)
    first = [m.tag for m in path.moves].index("infill")
    pts = np.array([m.pose[:3, 3] for m in path.moves[first - 1:]
                    if m.gun_on and np.isclose(m.pose[2, 3], 10 + proc.standoff_mm)])   # stroke starts too
    assert len(pts)
    assert np.isclose(pts[:, 0].min(), -20 - proc.overrun_mm, atol=0.01)
    assert np.isclose(pts[:, 0].max(), 20 + proc.overrun_mm, atol=0.01)


def _flat_plus_fillet(r=20.0, flat=40.0, sweep_deg=60.0, length=120.0):
    """Flat strip (y in [0, flat], z = 0) running into a fillet of radius r turning down
    by `sweep_deg`; returns (vertices, faces, surface distance across of a point)."""
    th = np.radians(np.linspace(0, sweep_deg, 31))
    prof = np.vstack([np.column_stack([np.linspace(0, flat, 21)[:-1], np.zeros(20)]),
                      np.column_stack([flat + r * np.sin(th), -r * (1 - np.cos(th))])])
    xs = np.linspace(0, length, 13)
    v = np.array([[x, y, z] for x in xs for y, z in prof])
    k = len(prof)
    f = []
    for i in range(len(xs) - 1):
        for j in range(k - 1):
            a, b = i * k + j, (i + 1) * k + j
            f += [[a, b, a + 1], [a + 1, b, b + 1]]

    def across(p):
        y, z = p[..., 1], p[..., 2]
        return np.where(y <= flat + 1e-9, y, flat + r * np.arctan2(y - flat, r + z))
    return v, np.array(f), across


@pytest.mark.parametrize("pattern", ["zigzag", "oneway"])
def test_step_over_measured_along_the_surface_over_a_fillet(pattern):
    """Passes stay step_over apart along the surface where a flat face runs into a curve
    (they used to be spaced in the plane-cut / flattened frame and bunch on the curve)."""
    proc = load_process()
    proc.infill_pattern = pattern
    proc.contour_passes = 0
    v, f, across = _flat_plus_fillet()
    path = plan_surface(v, f, np.ones(len(f), bool), proc)
    pts = np.array([m.pose[:3, 3] + m.pose[:3, 0] * proc.standoff_mm for m in path.moves
                    if m.gun_on and m.tag == "infill"])
    inside = (pts[:, 0] > 1) & (pts[:, 0] < 119)              # not the overrun past the ends
    s = np.unique(np.round(across(pts[inside]), 1))
    s = s[np.concatenate([[True], np.diff(s) > proc.step_over_mm / 2])]   # one value per pass
    gaps = np.diff(s)
    total = across(np.array([0, 40 + 20 * np.sin(np.radians(60)), -20 * (1 - np.cos(np.radians(60)))]))
    assert len(s) == pytest.approx(total / proc.step_over_mm, abs=1.5)
    assert np.allclose(gaps, proc.step_over_mm, rtol=0.05), gaps
