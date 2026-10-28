import numpy as np
from coldspray.toolpath.model import Move, Toolpath
from coldspray.transforms import frame


def _tp():
    rng = np.random.default_rng(0)
    mv = [Move(frame(x=float(i), y=2.0, z=3.0, a=10.0 * i, b=5.0, c=float(i)), 5.0, i % 2 == 0, "LIN", {"E2": float(i)}, "t", 0.1)
          for i in range(7)]
    return Toolpath("n", mv, 3, rng.normal(size=(7, 6)), 4)


def test_transformed_rolled_match_per_move_and_keep_fields():
    tp, t = _tp(), frame(x=10.0, z=-4.0, a=30.0)
    for out, ref in ((tp.transformed(t), [t @ m.pose for m in tp.moves]), (tp.rolled(-135.0), [m.pose @ frame(c=-135.0) for m in tp.moves])):
        assert all(np.array_equal(m.pose, r) for m, r in zip(out.moves, ref))
        assert (out.skipped_points, out.zigzag_regions) == (3, 4)
        assert out.joint_hint is tp.joint_hint
        assert [(m.speed, m.gun_on, m.kind, m.axes, m.tag, m.dwell_s) for m in out.moves] == \
               [(m.speed, m.gun_on, m.kind, m.axes, m.tag, m.dwell_s) for m in tp.moves]
        assert out.moves[0].axes is not tp.moves[0].axes


def test_empty():
    assert Toolpath("e").rolled(5.0).moves == []
