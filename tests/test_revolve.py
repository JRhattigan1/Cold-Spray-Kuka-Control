import numpy as np
import pytest
import trimesh

from coldspray.cell import base_world
from coldspray.models import load_cell, load_process
from coldspray.planner.revolve import plan_revolve, radius_profile

R, H = 40.0, 60.0


@pytest.fixture(scope="module")
def cylinder():
    m = trimesh.creation.cylinder(radius=R, height=H, sections=256)
    m.apply_translation([0, 0, H / 2])
    return np.asarray(m.vertices), np.asarray(m.faces)


def test_radius_profile(cylinder):
    r = radius_profile(*cylinder, [10, 30, 50, 70])
    assert np.allclose(r[:3], R, atol=0.1) and np.isnan(r[3])


def test_constant_standoff_and_aimed_at_axis(cylinder):
    proc = load_process()
    path = plan_revolve(*cylinder, proc, coats=1)
    spray = [m for m in path.moves if m.gun_on]
    for m in spray:
        p, x = m.pose[:3, 3], m.pose[:3, 0]
        assert np.isclose(np.linalg.norm(p[:2]), R + proc.standoff_mm, atol=0.1)
        assert np.allclose(x[:2] / np.linalg.norm(x[:2]), -p[:2] / np.linalg.norm(p[:2]))  # sprays at the axis


def test_nozzle_stays_put_while_table_turns(cylinder):
    cell, proc = load_cell(), load_process()
    path = plan_revolve(*cylinder, proc, coats=1)
    axes = path.axis_values()
    world = np.array([(base_world(cell, a) @ m.pose)[:3, 3] for m, a in zip(path.moves, axes) if m.gun_on])
    heights = np.round(world[:, 2], 3)
    for z in np.unique(heights):   # within one pass the nozzle only moves along the axis
        ring = world[heights == z]
        assert np.ptp(ring[:, 0]) < 1e-6 and np.ptp(ring[:, 1]) < 1e-6


def test_passes_alternate_and_stay_in_limits(cylinder):
    cell, proc = load_cell(), load_process()
    path = plan_revolve(*cylinder, proc, coats=2)
    e5 = np.array([a["E5"] for a in path.axis_values()])
    lo, hi = cell.turntable.rotation.limits_deg
    assert e5.min() >= lo and e5.max() <= hi
    ends = [m.axes["E5"] for m in path.moves if m.gun_on and abs(abs(m.axes.get("E5", 0)) - 180) < 1e-9]
    assert set(np.sign(ends)) == {-1.0, 1.0}


def test_surface_speed_is_traverse_speed(cylinder):
    proc = load_process()
    path = plan_revolve(*cylinder, proc, coats=1)
    a, b = path.moves[3], path.moves[4]          # two points on the first pass
    assert a.gun_on and b.gun_on and a.pose[2, 3] == b.pose[2, 3]
    turn = np.radians(abs(b.axes["E5"] - a.axes["E5"]))
    t = np.linalg.norm(b.pose[:3, 3] - a.pose[:3, 3]) / b.speed
    surface_arc = 2 * R * np.sin(turn / 2)
    assert np.isclose(surface_arc / t, proc.traverse_speed_mm_s, rtol=1e-3)


def test_coats_back_off_by_layer_thickness(cylinder):
    proc = load_process()
    path = plan_revolve(*cylinder, proc, coats=3)
    radii = sorted({round(float(np.linalg.norm(m.pose[:2, 3])), 3) for m in path.moves if m.gun_on})
    assert np.allclose(np.diff(radii), proc.layer_thickness_mm)
