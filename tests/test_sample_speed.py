"""Vectorised Toolpath.sample must match the old per-move Slerp implementation."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from coldspray.toolpath.model import Move, Toolpath, Samples


def _old_sample(self, step=5.0, angle_step=2.0):
    axes_per_move = self.axis_values()
    names = sorted({k for a in axes_per_move for k in a})
    first = {k: axes_per_move[-1].get(k, 0.0) for k in names}
    values = [{k: a.get(k, first[k]) for k in names} for a in axes_per_move]
    poses, gun, index = [self.moves[0].pose], [self.moves[0].gun_on], [0]
    ax = {k: [values[0][k]] for k in names}
    for i in range(1, len(self.moves)):
        a, b = self.moves[i - 1].pose, self.moves[i].pose
        dist = np.linalg.norm(b[:3, 3] - a[:3, 3])
        turn = max((abs(values[i][k] - values[i - 1][k]) for k in names), default=0.0)
        n = max(1, int(np.ceil(dist / step)), int(np.ceil(turn / angle_step)))
        slerp = Slerp([0, 1], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))
        for t in np.linspace(0, 1, n + 1)[1:]:
            p = np.eye(4)
            p[:3, :3] = slerp(t).as_matrix()
            p[:3, 3] = (1 - t) * a[:3, 3] + t * b[:3, 3]
            poses.append(p)
            gun.append(self.moves[i].gun_on)
            index.append(i)
            for k in names:
                ax[k].append((1 - t) * values[i - 1][k] + t * values[i][k])
    return Samples(np.array(poses), np.array(gun), np.array(index),
                   {k: np.array(v) for k, v in ax.items()})


def _path(n, seed, with_axes=True):
    rng = np.random.default_rng(seed)
    moves = []
    R = Rotation.identity()
    for i in range(n):
        if i % 3:  # keep some identical orientations and repeated points
            R = Rotation.random(random_state=int(rng.integers(1e6)))
        pose = np.eye(4)
        pose[:3, :3] = R.as_matrix()
        pose[:3, 3] = rng.uniform(-100, 100, 3) if i % 4 else (0, 0, 0)
        axes = {"E2": float(rng.uniform(-90, 90))} if with_axes and i % 2 else {}
        if with_axes and i == 5:
            axes["E5"] = 40.0
        moves.append(Move(pose, 100.0, bool(i % 2), "LIN", axes, "t"))
    return Toolpath("t", moves)


def _same(tp):
    a, b = tp.sample(5.0, 2.0), _old_sample(tp, 5.0, 2.0)
    assert len(a) == len(b)
    assert np.allclose(a.poses, b.poses, atol=1e-6)
    assert (a.gun_on == b.gun_on).all() and (a.move_index == b.move_index).all()
    assert a.axes.keys() == b.axes.keys()
    for k in a.axes:
        assert np.allclose(a.axes[k], b.axes[k], atol=1e-9)


def test_matches_old_with_axes():
    _same(_path(40, 1))


def test_matches_old_without_axes():
    _same(_path(40, 2, with_axes=False))


def test_single_move():
    _same(_path(1, 3))


def test_half_turn_flip():
    mv = []
    for i, ang in enumerate([0, 180, 180, 0, 90]):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler("z", ang, degrees=True).as_matrix()
        pose[:3, 3] = (10 * i, 0, 0)
        mv.append(Move(pose, 100.0, True))
    _same(Toolpath("t", mv))
