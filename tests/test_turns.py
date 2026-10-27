"""G1: infill turning points off the component."""
import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from coldspray.models import load_process
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.patterns import zigzag
from coldspray.planner.turns import extend_strokes, parse_plane
from coldspray.cad.slicer import slice_mesh

BASE = load_process()


def _proc(**kw):
    return BASE.model_copy(update=kw)


def FILLED(reg):
    return unary_union([Polygon(g.exterior) for g in getattr(reg, "geoms", [reg])])


def _region():
    part = load_part("test_block")
    v, f = part.vertices, part.faces
    layers = slice_mesh(v - v.min(axis=0), f, BASE.layer_thickness_mm)
    return layers[0].region


def test_inside_is_unchanged():
    reg = _region()
    s = zigzag(reg, BASE, 0)
    assert extend_strokes(s, reg, _proc(turn_mode="inside")) is s


def test_overrun_turns_outside_region():
    reg = _region()
    o = 7.0
    proc = _proc(turn_mode="overrun", overrun_mm=o)
    for layer_index in (0, 1):
        base = zigzag(reg, proc, layer_index)
        ext = extend_strokes(base, reg, proc)
        for b, e in zip(base, ext):
            u = (b[-1] - b[0]) / np.linalg.norm(b[-1] - b[0])
            for end, turn, d in ((b[0], e[0], -u), (b[-1], e[-1], u)):
                if FILLED(reg).contains(Point(end + d * 0.01)):
                    np.testing.assert_allclose(turn, end)      # hole edge: not extended
                else:
                    assert reg.distance(Point(turn)) >= o - 1e-6
                    np.testing.assert_allclose(np.dot(turn - end, d), o)
            np.testing.assert_allclose(e[1:3], b)


def test_beyond_plane_turns_past_x60():
    reg = _region()
    proc = _proc(turn_mode="beyond_plane", overrun_mm=2.0, turn_plane="YZ:60")
    plane = parse_plane(proc.turn_plane)
    assert plane is not None
    strokes = extend_strokes(zigzag(reg, proc, 0), reg, proc, plane)   # layer 0: rasters along x
    outer = [e for e in strokes if max(e[1][0], e[2][0]) >= reg.bounds[2] - 1e-6]  # not ending at the hole
    assert outer
    for e in outer:
        assert max(e[0][0], e[-1][0]) >= 62 - 1e-6       # crosses x = 60, plus 2 mm overrun


def test_plan_tags_turns_and_default_unchanged():
    part = load_part("test_block")
    _, plain = plan_part(part, BASE)
    _, same = plan_part(part, _proc(turn_mode="inside"))
    assert len(plain.moves) == len(same.moves)
    _, over = plan_part(part, _proc(turn_mode="overrun", overrun_mm=5.0))
    assert sum(m.tag == "turn" for m in over.moves) > sum(m.tag == "turn" for m in plain.moves)
