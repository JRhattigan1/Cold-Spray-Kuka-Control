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
