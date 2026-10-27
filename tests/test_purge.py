from types import SimpleNamespace

import numpy as np
import pytest

from coldspray.cell import base_in_robot, base_world
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.planner.purge import add_purge, layer_ends, purge_world
from coldspray.post.krl import to_krl
from coldspray.toolpath.model import Move, Toolpath, spray_frame


@pytest.fixture(scope="module")
def setup():
    return Robot(load_robot(), load_tool()), load_cell(), load_process()


def _cell(cell, **kw):
    """Cell copy with a purge config (plain namespace until CellConfig.purge lands)."""
    c = cell.model_copy(deep=True)
    p = dict(enabled=True, point=(0.0, 0.0, 0.0), spray_dir=(0.0, 0.0, -1.0), dwell_s=5.0,
             at_start=True, at_end=True, every_n_layers=0, approach_mm=100.0) | kw
    object.__setattr__(c, "purge", SimpleNamespace(**p))
    return c


def _layers(n=3, tilt=0.0):
    """Toolpath shaped like layers.py: approach, spray strokes, retract, per layer."""
    at = lambda x, z: spray_frame((x, 0.0, z), (0, 0, -1), (1, 0, 0))
    moves = []
    for k in range(n):
        z = 30.0 + 2 * k
        moves += [Move(at(0, z + 100), 250, kind="PTP" if k == 0 else "LIN",
                       axes={"E2": tilt, "E5": 0.0} if k == 0 else {}, tag="approach"),
                  Move(at(0, z), 250, tag="approach"),
                  Move(at(50, z), 20, gun_on=True, tag="infill"),
                  Move(at(50, z + 100), 250, tag="approach")]
    return Toolpath("blk", moves)


def _world_point(cell, z=300.0):
    return tuple(base_world(cell)[:3, 3] + [150.0, 0.0, z])


@pytest.mark.parametrize("tilt", [0.0, 30.0, 60.0])
def test_purge_poses_fixed_in_world(setup, tilt):
    _, cell, _ = setup
    cell = _cell(cell, point=_world_point(cell), every_n_layers=1)
    path = add_purge(_layers(tilt=tilt), cell)
    axes = path.axis_values()
    want = purge_world(cell.purge)
    visits = [i for i, m in enumerate(path.moves) if m.tag == "purge" and m.kind == "LIN"]
    assert len(visits) == 1 + 2 + 1                      # start, after layers 1 and 2, end
    for i in visits:
        assert np.allclose(base_world(cell, axes[i]) @ path.moves[i].pose, want, atol=1e-6)
    approach = base_world(cell, axes[0]) @ path.moves[0].pose
    assert np.allclose(approach[:3, 3], np.add(cell.purge.point, [0, 0, 100.0]))


def test_gun_sequence(setup):
    _, cell, _ = setup
    cell = _cell(cell, point=_world_point(cell))
    path = add_purge(_layers(), cell)
    m = path.moves
    assert (m[0].tag, m[0].kind, m[0].gun_on) == ("purge", "PTP", False)
    assert (m[1].tag, m[1].gun_on, m[1].dwell_s) == ("purge", True, 5.0)
    assert all(x.gun_on for x in m[1:]) and all(x.kind == "LIN" for x in m[1:])
    assert (m[-1].tag, m[-1].dwell_s) == ("purge", 0.0)
    assert len(m) == len(_layers().moves) + 3


def test_layer_ends_and_disabled(setup):
    _, cell, _ = setup
    path = _layers()
    assert layer_ends(path) == [3, 7, 11]
    assert add_purge(path, cell) is path                 # no purge config
    assert add_purge(path, _cell(cell, enabled=False)) is path
    assert add_purge(path, None) is path


def test_krl_dwell_and_single_spray_toggle(setup):
    robot, cell, proc = setup
    # a reachable part path and a purge point beside it, gun pointing down
    p = robot.fk([0, -90, 90, 0, 60, 0])
    pose = np.linalg.inv(base_in_robot(cell)) @ p
    shift = lambda dx, dz=0.0: pose @ np.array([[1, 0, 0, dz], [0, 1, 0, dx], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    path = Toolpath("pp", [Move(shift(0, -100), 250, kind="PTP", axes={"E2": 0.0, "E5": 0.0}, tag="approach"),
                           Move(shift(0), 250, tag="approach"),
                           Move(shift(50), 20, gun_on=True, tag="infill"),
                           Move(shift(50, -100), 250, tag="approach")])
    w = base_world(cell) @ shift(-200, -50)
    cell = _cell(cell, point=tuple(w[:3, 3]), spray_dir=tuple(w[:3, 0]), dwell_s=4.0)
    src = to_krl(add_purge(path, cell), robot, cell, proc).splitlines()
    gun = f"$OUT[{cell.gun_output}]"
    assert "WAIT SEC 4  ;dwell" in src
    assert sum(l == f"{gun} = TRUE" for l in src) == 1
    assert sum(l == f"{gun} = FALSE" for l in src) == 2   # the init line and the final switch-off
    on = src.index(f"{gun} = TRUE")
    assert src.index("WAIT SEC 4  ;dwell") > on


def test_plan_part_rolls_purge_moves_too(setup):
    """auto_position checks roll on every pose (purge included), so plan_part must roll them too."""
    from coldspray.pipeline import load_part, plan_part
    from coldspray.transforms import frame
    _, cell, proc = setup
    c = _cell(cell, point=(1500.0, 0.0, 800.0), spray_dir=(0.0, -1.0, 0.0))
    part = load_part("test_block")
    _, p0 = plan_part(part, proc, tilt_deg=90, cell=c)
    _, p45 = plan_part(part, proc, tilt_deg=90, cell=c, roll_deg=45)
    for a, b in zip(p0.moves, p45.moves):
        assert np.allclose(a.pose @ frame(c=45), b.pose)


def test_layer_ends_only_where_a_new_layer_starts():
    """A lone `approach` lift followed straight by spraying is not a layer end."""
    path = _layers()
    at = lambda x, z: spray_frame((x, 0.0, z), (0, 0, -1), (1, 0, 0))
    moves = list(path.moves)
    moves[3:3] = [Move(at(50, 80), 250, tag="approach"), Move(at(60, 30), 20, gun_on=True, tag="infill")]
    assert layer_ends(Toolpath("x", moves)) == [5, 9, 13]


def test_surface_coats_get_no_every_n_purge(setup):
    """Surface coats are not layers: every-N adds nothing, only start and end visits."""
    from coldspray.pipeline import load_part, plan_part
    _, cell, proc = setup
    proc = proc.model_copy(update={"surface_coats": 3})
    part = load_part("test_block")
    part.coat_faces = {int(part.face_ids[np.argmax(part.vertices[part.faces].mean(1)[:, 2])])}
    c = _cell(cell, point=_world_point(cell), every_n_layers=1)
    _, path = plan_part(part, proc, mode="surface", cell=c)
    visits = [m for m in path.moves if m.tag == "purge" and m.kind == "LIN"]
    assert len(visits) == 2
