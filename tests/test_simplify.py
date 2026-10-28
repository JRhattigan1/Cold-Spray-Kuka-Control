"""toolpath.simplify: merged LINs and fitted CIRC arcs reproduce the path they replace."""

import numpy as np
import pytest

from coldspray.toolpath.model import Move, Toolpath
from coldspray.toolpath.simplify import TOL_MM, simplify


def _base():
    from coldspray.models import load_process
    from coldspray.toolpath.raster import plate_raster
    return plate_raster(load_process(), 100, 100).moves[0].pose    # a reachable spray pose


def _path(points, base=None):
    """Gun-on LIN moves through `points` (local x, y in mm) at one orientation."""
    base = np.eye(4) if base is None else base
    moves = [Move(pose=base.copy(), speed=20.0, gun_on=False, tag="approach")]
    for x, y in points:
        p = base.copy()
        p[:3, 3] = base[:3, 3] + base[:3, 1] * x + base[:3, 2] * y
        moves.append(Move(pose=p, speed=20.0, gun_on=True, tag="infill"))
    return Toolpath("simplify_test", moves)


def _arc_points(r=30.0, deg=150.0, n=60):
    a = np.radians(np.linspace(0, deg, n))
    return np.c_[r * np.cos(a), r * np.sin(a)]


def _seg_dist(p, a, b):
    d = b - a
    t = np.clip(((p - a) @ d) / max(d @ d, 1e-12), 0, 1)
    return np.linalg.norm(a + t * d - p)


def test_known_arc_becomes_one_circ():
    out = simplify(_path(_arc_points()))
    kinds = [m.kind for m in out.moves[1:]]
    # The first point is reached by a LIN; the rest of the arc is one CIRC.
    assert kinds.count("CIRC") == 1 and len(kinds) <= 3
    circ = next(m for m in out.moves if m.kind == "CIRC")
    assert circ.aux is not None
    centre = out.moves[0].pose[:3, 3]
    assert abs(np.linalg.norm(circ.aux[:3, 3] - centre) - 30.0) < 1e-6   # the via point is on the circle


def test_rectangle_corners_are_not_an_arc():
    """The 4 corners of a rectangle lie on its circumcircle; their LINs must stay LINs."""
    out = simplify(_path([(5, 5), (45, 5), (45, 25), (5, 25), (5, 5)]))
    assert all(m.kind == "LIN" for m in out.moves[1:])
    assert len(out.moves) == 6


@pytest.mark.parametrize("pts", [
    _arc_points(),
    np.r_[np.c_[np.linspace(0, 50, 26), np.zeros(26)], _arc_points(20, 180, 40) + [50, 20],
          np.c_[np.linspace(50, 0, 26), np.full(26, 40.0)]],          # line, half turn, line
    np.c_[np.linspace(0, 60, 121), 8 * np.sin(np.linspace(0, 60, 121) / 6)],  # wavy: arcs of changing curvature
])
def test_every_point_within_tolerance_of_resampled_path(pts):
    path = _path(pts)
    out = simplify(path)
    assert len(out.moves) < len(path.moves)
    s = out.sample(step=0.2)
    q = s.poses[:, :3, 3]
    for m in path.moves:
        p = m.pose[:3, 3]
        k = int(np.argmin(np.linalg.norm(q - p, axis=1)))
        d = min(_seg_dist(p, q[j], q[j + 1]) for j in range(max(k - 1, 0), min(k + 1, len(q) - 1)))
        assert d <= TOL_MM + 1e-6


def test_circ_krl_syntax():
    from coldspray.kinematics.robot import Robot
    from coldspray.models import load_cell, load_process, load_robot, load_tool
    from coldspray.post.krl import to_krl
    proc = load_process()
    out = simplify(_path(_arc_points(), _base()))
    src = to_krl(out, Robot(load_robot(), load_tool()), load_cell(), proc).splitlines()
    assert "$CIRC_TYPE = #PATH" in src and "$ORI_TYPE = #VAR" in src
    circ = [l for l in src if l.startswith("CIRC ")]
    assert len(circ) == 1
    via, end = circ[0][len("CIRC "):].removesuffix(" C_DIS").split("}, ")
    for pos in (via + "}", end):
        assert pos.startswith("{X ") and pos.endswith("}") and ", A " in pos and ", C " in pos


def test_arc_time_and_transforms():
    """Time estimates use the arc length; transformed / rolled paths move the via point too."""
    from coldspray.gui.steps.toolpath import estimate_time_s
    from coldspray.gui.timeline import sample_times
    from coldspray.models import load_process
    from coldspray.transforms import frame
    proc = load_process()
    orig, out = _path(_arc_points()), simplify(_path(_arc_points()))
    t_orig, t_out = estimate_time_s(orig, proc), estimate_time_s(out, proc)
    assert abs(t_out - t_orig) < 0.01 * t_orig                     # 60 chords ~ the arc
    assert abs(sample_times(out, out.sample(1.0), proc)[-1] - t_out) < 0.01 * t_out
    t = frame(x=10, y=-5, z=3, c=30)
    moved = out.transformed(t).rolled(15)
    i = next(k for k, m in enumerate(out.moves) if m.kind == "CIRC")
    assert np.allclose(moved.moves[i].aux, t @ out.moves[i].aux @ frame(c=15))
