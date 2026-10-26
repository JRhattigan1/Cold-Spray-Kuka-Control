import numpy as np
import pytest
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
from shapely.geometry import LineString, Point

from coldspray.cad.slicer import place_on_plate, slice_mesh
from coldspray.cad.step_io import load_step, tessellate
from coldspray.models import load_process
from coldspray.planner.layers import plan_layers, raster_segments

W, D, H, R = 40.0, 30.0, 5.0, 5.0  # block with a through hole at its centre


@pytest.fixture(scope="module")
def block_step(tmp_path_factory):
    box = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 10), W, D, H).Shape()  # sits at z = 10
    hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(W / 2, D / 2, 0), gp_Dir(0, 0, 1)), R, 50).Shape()
    writer = STEPControl_Writer()
    writer.Transfer(BRepAlgoAPI_Cut(box, hole).Shape(), STEPControl_AsIs)
    path = tmp_path_factory.mktemp("cad") / "block.step"
    writer.Write(str(path))
    return path


@pytest.fixture(scope="module")
def layers(block_step):
    v, f = tessellate(load_step(block_step), deflection=0.01)
    return slice_mesh(place_on_plate(v), f, layer_thickness=0.5)


def test_layer_count_and_heights(layers):
    assert len(layers) == 10
    assert layers[0].z_bottom == 0 and np.isclose(layers[-1].z_top, H)


def test_layer_region_has_hole(layers):
    region = layers[3].region
    assert np.isclose(region.area, W * D - np.pi * R**2, rtol=1e-3)
    assert not region.contains(Point(W / 2, D / 2))


def test_raster_spacing_and_clipping(layers):
    region = layers[0].region
    segs = raster_segments(region, step=2.0, angle_deg=0)
    ys = np.unique(np.round([s[0][1] for s in segs], 6))
    assert np.allclose(np.diff(ys), 2.0)
    # lines through the hole are split in two
    assert sum(1 for s in segs if abs(s[0][1] - D / 2) < 1.0) >= 2


def test_plan_stays_on_part(layers):
    proc = load_process()
    path = plan_layers(layers, proc, "block")
    spray = [m for m in path.moves if m.gun_on]
    assert spray
    grown = layers[0].region.buffer(0.01)
    for m in spray:
        assert grown.contains(Point(m.pose[0, 3], m.pose[1, 3]))
    sprayed = [m for m in spray if m.speed == proc.traverse_speed_mm_s]
    heights = sorted({round(m.pose[2, 3], 6) for m in sprayed})
    assert np.allclose(heights, [l.z_bottom + proc.standoff_mm for l in layers])
    # every move made at spray speed lies on the part (no stripes across the hole)
    for a, b in zip(path.moves, path.moves[1:]):
        if b.speed == proc.traverse_speed_mm_s and b.gun_on:
            seg = LineString([a.pose[:2, 3], b.pose[:2, 3]])
            assert grown.contains(seg)
    # no zero-length moves
    pts = np.array([m.pose[:3, 3] for m in path.moves])
    assert np.all(np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9)


def test_raster_rotates_between_layers(layers):
    proc = load_process()
    seg0 = raster_segments(layers[0].region, 2.0, 0)
    seg1 = raster_segments(layers[1].region, 2.0, proc.raster_angle_step_deg)
    d0, d1 = seg0[0][1] - seg0[0][0], seg1[0][1] - seg1[0][0]
    assert abs(np.dot(d0, d1)) < 1e-6 * np.linalg.norm(d0) * np.linalg.norm(d1)
