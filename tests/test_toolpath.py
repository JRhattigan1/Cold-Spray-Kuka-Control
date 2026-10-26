import numpy as np

from coldspray.models import load_process
from coldspray.toolpath.model import spray_frame
from coldspray.toolpath.raster import plate_raster


def test_spray_frame_axes():
    t = spray_frame((1, 2, 3), (0, 0, -1), (1, 0, 0))
    assert np.allclose(t[:3, 0], [0, 0, -1])   # X = spray direction
    assert np.allclose(t[:3, 1], [1, 0, 0])    # Y = travel
    assert np.isclose(np.linalg.det(t[:3, :3]), 1)


def test_plate_raster_covers_plate():
    proc = load_process()
    path = plate_raster(proc, 100, 100)
    spray = np.array([m.pose[:3, 3] for m in path.moves if m.gun_on])

    assert np.allclose(spray[:, 2], proc.standoff_mm)
    assert spray[:, 0].min() == -proc.overrun_mm
    assert spray[:, 0].max() == 100 + proc.overrun_mm
    ys = np.unique(spray[:, 1])
    assert np.allclose(np.diff(ys), proc.step_over_mm)
    assert ys[0] == 0 and np.isclose(ys[-1], 100)
    assert path.moves[0].kind == "PTP" and not path.moves[0].gun_on
    assert not path.moves[-1].gun_on


def test_sample_spacing():
    path = plate_raster(load_process(), 100, 10)
    poses, gun, idx = path.sample(step=5)
    gaps = np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1)
    assert gaps.max() <= 5 + 1e-9
    assert len(poses) == len(gun) == len(idx)
