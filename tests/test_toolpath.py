import numpy as np

from coldspray.models import load_process
from coldspray.toolpath.model import Move, Toolpath, spray_frame
from coldspray.toolpath.raster import plate_raster
from coldspray.transforms import frame


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
    s = path.sample(step=5)
    gaps = np.linalg.norm(np.diff(s.poses[:, :3, 3], axis=0), axis=1)
    assert gaps.max() <= 5 + 1e-9
    assert len(s.poses) == len(s.gun_on) == len(s.move_index)


def test_sample_interpolates_external_axes():
    p = spray_frame((0, 0, 30), (0, 0, -1), (1, 0, 0))
    path = Toolpath("t", [Move(p, 100, axes={"E2": 0.0, "E5": 0.0}),
                          Move(p, 100, axes={"E5": 90.0}),         # rotate in place
                          Move(p, 100)])                           # E5 carried forward
    s = path.sample(step=5, angle_step=2)
    assert len(s) == 1 + 45 + 1
    assert np.allclose(s.axes["E5"][[0, 45, -1]], [0, 90, 90])
    assert np.allclose(s.axes["E2"], 0)
    assert path.axis_values()[-1] == {"E2": 0.0, "E5": 90.0}


def test_toolpath_transformed():
    p = spray_frame((1, 2, 3), (0, 0, -1), (1, 0, 0))
    path = Toolpath("t", [Move(p, 100, kind="PTP", axes={"E2": 5.0}), Move(p, 20, gun_on=True)])
    t = frame(10, 0, 4, a=90)
    moved = path.transformed(t)
    assert moved.name == "t" and len(moved.moves) == 2
    assert np.allclose(moved.moves[0].pose, t @ p) and np.allclose(path.moves[0].pose, p)
    assert [(m.speed, m.gun_on, m.kind, m.axes) for m in moved.moves] == \
        [(m.speed, m.gun_on, m.kind, m.axes) for m in path.moves]
    assert moved.moves[0].axes is not path.moves[0].axes


def test_transformed_and_rolled_keep_tag_and_dwell():
    p = Toolpath("t", [Move(frame(), 10.0, True, tag="purge", dwell_s=3.0)])
    for q in (p.transformed(frame(1, 2, 3)), p.rolled(30)):
        assert (q.moves[0].tag, q.moves[0].dwell_s) == ("purge", 3.0)
