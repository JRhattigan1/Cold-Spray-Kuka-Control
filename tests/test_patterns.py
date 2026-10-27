import numpy as np
import pytest
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union

from coldspray.models import load_process
from coldspray.planner import layers, patterns
from coldspray.planner.patterns import PATTERNS, raster_segments, rotate_seam

PROC = load_process()

# a plate with a hole, an L-shape, and a two-island region
PLATE = box(0, 0, 40, 30).difference(Point(20, 15).buffer(5))
L_SHAPE = Polygon([(0, 0), (30, 0), (30, 10), (10, 10), (10, 30), (0, 30)])
TWO = MultiPolygon([box(0, 0, 15, 15), box(25, 0, 45, 12)])
REGIONS = {"plate": PLATE, "L": L_SHAPE, "two": TWO}


@pytest.mark.parametrize("name", list(PATTERNS))
@pytest.mark.parametrize("region", list(REGIONS))
@pytest.mark.parametrize("layer_index", [0, 1, 2])
def test_pattern_covers_and_stays_inside(name, region, layer_index):
    reg = REGIONS[region]
    strokes = PATTERNS[name](reg, PROC, layer_index)
    assert strokes and all(s.ndim == 2 and s.shape[1] == 2 and len(s) >= 2 for s in strokes)
    inside = reg.buffer(1e-6)
    assert all(inside.contains(LineString(s)) for s in strokes)
    sprayed = unary_union([LineString(s).buffer(PROC.track_width_mm / 2) for s in strokes])
    assert sprayed.intersection(reg).area >= 0.95 * reg.area


@pytest.mark.parametrize("name", list(PATTERNS))
def test_pattern_is_deterministic(name):
    a = PATTERNS[name](PLATE, PROC, 3, start=(1, 1))
    b = PATTERNS[name](PLATE, PROC, 3, start=(1, 1))
    assert len(a) == len(b) and all(np.array_equal(x, y) for x, y in zip(a, b))


def test_zigzag_matches_old_layers_behaviour():
    start = np.array([2.0, 2.0])
    for k in range(4):
        angle = (k * PROC.raster_angle_step_deg) % 180
        old = layers._order(raster_segments(PLATE, PROC.step_over_mm, angle), start)
        new = PATTERNS["zigzag"](PLATE, PROC, k, start=start)
        assert len(old) == len(new) and all(np.array_equal(x, y) for x, y in zip(old, new))


def test_old_layer_imports_still_work():
    assert layers.contour_loops is patterns.contour_loops
    assert layers.raster_segments is patterns.raster_segments


def test_oneway_strokes_all_point_the_same_way():
    dirs = [s[-1] - s[0] for s in PATTERNS["oneway"](box(0, 0, 40, 30), PROC, 0)]
    dirs = [d / np.linalg.norm(d) for d in dirs]
    assert all(np.allclose(d, dirs[0]) for d in dirs)


def test_concentric_loops_are_closed_and_nested():
    loops = PATTERNS["concentric"](box(0, 0, 40, 30), PROC, 0)
    assert len(loops) > 3 and all(np.allclose(s[0], s[-1]) for s in loops)
    areas = [Polygon(s).area for s in loops]
    assert areas == sorted(areas, reverse=True)


def test_spiral_joins_loops_into_fewer_strokes():
    reg = box(0, 0, 40, 30)
    assert len(PATTERNS["spiral"](reg, PROC, 0)) < len(PATTERNS["concentric"](reg, PROC, 0))


def test_crosshatch_alternates_perpendicular():
    a = PATTERNS["crosshatch"](box(0, 0, 40, 30), PROC, 0)[0]
    b = PATTERNS["crosshatch"](box(0, 0, 40, 30), PROC, 1)[0]
    da, db = a[-1] - a[0], b[-1] - b[0]
    assert abs(np.dot(da, db)) < 1e-6 * np.linalg.norm(da) * np.linalg.norm(db)


def test_islands_mix_directions():
    strokes = PATTERNS["islands"](box(0, 0, 60, 60), PROC, 0)
    horiz = sum(abs((s[-1] - s[0])[1]) < 1e-9 for s in strokes)
    assert 0 < horiz < len(strokes)


def test_rotate_seam_moves_start_but_keeps_loop():
    loop = np.asarray(box(0, 0, 10, 10).exterior.coords)
    starts = {tuple(rotate_seam(loop, k)[0]) for k in range(5)}
    assert len(starts) > 1
    r = rotate_seam(loop, 3)
    assert np.allclose(r[0], r[-1]) and Polygon(r).equals(Polygon(loop))
    assert np.array_equal(rotate_seam(loop, 0), loop)


def _points(strokes):
    return np.vstack(strokes)


@pytest.mark.parametrize("a, b", [("zigzag", "crosshatch"), ("concentric", "spiral")])
def test_patterns_differ_by_path_not_just_count(a, b):
    """Same stroke count is fine; the same sprayed points over the first layers is not."""
    reg = box(0, 0, 40, 30)
    same = [np.array_equal(_points(PATTERNS[a](reg, PROC, k)), _points(PATTERNS[b](reg, PROC, k)))
            for k in range(4)]
    assert not all(same)


def test_spiral_steps_in_without_closing_each_loop():
    """A spiral leaves each ring before closing it, heading straight for the next one."""
    reg = box(0, 0, 40, 30)
    stroke = max(PATTERNS["spiral"](reg, PROC, 0), key=len)
    # no point is visited twice (closed rings would return to their start)
    assert len({tuple(np.round(p, 9)) for p in stroke[:-1]}) == len(stroke) - 1
