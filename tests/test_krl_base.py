"""KRL base picked on the part: the frame from a face + edge click, and the program written in it."""

import re

import numpy as np
import pytest

from coldspray.cad.part import krl_base_frame
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part
from coldspray.post.krl import to_krl
from coldspray.toolpath.raster import plate_raster
from coldspray.transforms import frame


def _top(part):
    """(face id, its corner vertices) of the block's top face, in part-CAD."""
    fid = max(part.planar, key=lambda k: (part.planar[k].normal[2], part.planar[k].point[2]))
    v = part.vertices[np.unique(part.faces[part.face_ids == fid])]
    return fid, v


def test_frame_from_face_and_edge():
    part = load_part("test_block")
    fid, v = _top(part)
    lo = v.min(0)
    hi = v.max(0)
    corner = np.array([lo[0], lo[1], hi[2]])
    # near the (xmin, ymin) corner, beside the edge running along +X
    t = krl_base_frame(part, fid, corner + [5.0, 0.5, 0.0])
    assert np.allclose(t[:3, 3], corner, atol=1e-6)
    assert np.allclose(t[:3, 0], [1, 0, 0], atol=1e-6)
    assert np.allclose(t[:3, 2], [0, 0, 1], atol=1e-6)
    assert np.allclose(t[:3, 1], [0, 1, 0], atol=1e-6)            # right-handed, into the face
    # same corner, beside the edge along +Y: X follows that edge
    t = krl_base_frame(part, fid, corner + [0.5, 5.0, 0.0])
    assert np.allclose(t[:3, 3], corner, atol=1e-6) and np.allclose(t[:3, 0], [0, 1, 0], atol=1e-6)
    assert np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3)) and np.linalg.det(t[:3, :3]) > 0
    # the far end of the X edge: origin there, X pointing back along it
    t = krl_base_frame(part, fid, [hi[0] - 5.0, lo[1] + 0.5, hi[2]])
    assert np.allclose(t[:3, 3], [hi[0], lo[1], hi[2]], atol=1e-6) and np.allclose(t[:3, 0], [-1, 0, 0], atol=1e-6)


def test_without_keeps_the_base():
    part = load_part("test_block")
    fid, v = _top(part)
    part.krl_base, part.krl_base_face = krl_base_frame(part, fid, v[0]), fid
    copy = part.without([])
    assert copy.krl_base_face == fid and np.allclose(copy.krl_base, part.krl_base)


@pytest.fixture(scope="module")
def setup():
    return Robot(load_robot(), load_tool()), load_cell(), load_process()


def _lins(src):
    return [np.array([float(x) for x in re.findall(r"[XYZABC] (-?[\d.]+)", l)[:6]])
            for l in src.splitlines() if l.startswith("LIN")]


def test_program_relative_to_the_part_base(setup):
    robot, cell, proc = setup
    path = plate_raster(proc, 100, 100)
    base = frame(20.0, -10.0, 5.0, 30.0, 0.0, 0.0)
    plain, on_part = to_krl(path, robot, cell, proc), to_krl(path, robot, cell, proc, base)
    assert f"BAS(#BASE, {cell.base_number})" in plain
    assert f"BAS(#BASE, {cell.part_base_number})" in on_part
    assert f"BASE_DATA[{cell.part_base_number}] taught on the part" in on_part
    # the same points: moved into the part base
    for a, b in zip(_lins(plain), _lins(on_part)):
        assert np.allclose(np.linalg.inv(base) @ frame(*a), frame(*b), atol=2e-3)
    # joint (PTP) lines don't change
    ptp = lambda s: [l for l in s.splitlines() if l.startswith("PTP")]
    assert ptp(plain) == ptp(on_part)


# --- 3 planes, vertex, spin / flip ---------------------------------------------------
from coldspray.cad.part import PlanarFace, flip_z, krl_base_at_vertex, krl_base_from_planes, spin_frame, vertex_edges


def _ok(t):
    assert np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-9) and np.linalg.det(t[:3, :3]) > 0


def _face(part, normal):
    return max(part.planar, key=lambda k: part.planar[k].normal @ np.asarray(normal, float))


def test_frame_from_three_planes():
    part = load_part("test_block")
    lo, hi = part.vertices.min(0), part.vertices.max(0)
    top, front, left = (part.planar[_face(part, n)] for n in ([0, 0, 1], [0, -1, 0], [-1, 0, 0]))
    t = krl_base_from_planes([top, front, left], part.vertices.mean(0))
    _ok(t)
    assert np.allclose(t[:3, 3], [lo[0], lo[1], hi[2]], atol=1e-6)
    assert np.allclose(t[:3, 2], [0, 0, 1]) and np.allclose(t[:3, 0], [1, 0, 0])   # X into the part
    # 1st and 2nd swapped: Z out of the front, X along the same edge
    t = krl_base_from_planes([front, top, left], part.vertices.mean(0))
    assert np.allclose(t[:3, 2], [0, -1, 0]) and np.allclose(t[:3, 0], [1, 0, 0])
    with pytest.raises(ValueError):
        krl_base_from_planes([top, part.planar[_face(part, [0, 0, -1])], left], part.vertices.mean(0))
    tilted = PlanarFace(np.zeros(3), np.array([0.05, 0, 1.0]), 1.0)
    with pytest.raises(ValueError):
        krl_base_from_planes([top, front, tilted], part.vertices.mean(0))


def test_frame_at_vertex():
    part = load_part("test_block")
    lo, hi = part.vertices.min(0), part.vertices.max(0)
    corner = np.array([lo[0], lo[1], hi[2]])
    edges, normal = vertex_edges(part, corner)
    assert len(edges) == 3 and normal[2] > 0 and normal[0] < 0 and normal[1] < 0
    top = part.planar[_face(part, [0, 0, 1])].normal
    t = krl_base_at_vertex(corner, top, edges, toward=[1.0, 0.1, 0])
    _ok(t)
    assert np.allclose(t[:3, 3], corner) and np.allclose(t[:3, 0], [1, 0, 0]) and np.allclose(t[:3, 2], [0, 0, 1])
    t = krl_base_at_vertex(corner, top, edges, toward=[0.1, 1.0, 0])
    assert np.allclose(t[:3, 0], [0, 1, 0])
    _ok(krl_base_at_vertex(corner, normal, edges, toward=[1.0, 0, 0]))      # mean normal: still a frame
    assert np.allclose(krl_base_at_vertex(corner, [0, 0, 1], [], None)[:3, 0], [1, 0, 0])


def test_spin_and_flip():
    t = frame(10.0, 20.0, 30.0, 0.0, 0.0, 0.0)
    s = spin_frame(t, 90)
    _ok(s)
    assert np.allclose(s[:3, 3], t[:3, 3]) and np.allclose(s[:3, 2], t[:3, 2]) and np.allclose(s[:3, 0], [0, 1, 0])
    assert np.allclose(spin_frame(spin_frame(t, 30), -30), t)
    f = flip_z(t)
    _ok(f)
    assert np.allclose(f[:3, 0], t[:3, 0]) and np.allclose(f[:3, 2], -t[:3, 2]) and np.allclose(f[:3, 3], t[:3, 3])


def test_without_keeps_a_base_with_no_face():
    part = load_part("test_block")
    part.krl_base, part.krl_base_face = frame(1.0, 2.0, 3.0, 0, 0, 0), None
    assert np.allclose(part.without([]).krl_base, part.krl_base)
