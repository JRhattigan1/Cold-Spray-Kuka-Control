"""Golden regression for solver.solve: report fields and output-path poses/axes/gun flags
must not change when the solver is made faster. Regenerate (on main only, deliberately):
    GOLDEN_UPDATE=1 pytest tests/test_solver_golden.py -m "slow or not slow"
Scenarios use test_square (layers, tilt 90, wrist_side), collision "on" through a deterministic
synthetic checker (A2 > 20 deg collides) so the result does not depend on the CAD cache."""

import json
import os
from pathlib import Path

import numpy as np

from coldspray.cell import base_world
from coldspray.kinematics.robot import Robot
from coldspray.models import PurgeConfig, load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.planner import solver
from coldspray.planner.gun import wrist_side

GOLDEN = Path(__file__).parent / "golden"
UPDATE = bool(os.environ.get("GOLDEN_UPDATE"))


class FakeChecker:
    def check(self, q, axes):
        return {("A3", "Wall")} if q[1] > 20 else set()


def _run(gantry_end=None, purge=False, checker=None):
    cell, robot = load_cell(), Robot(load_robot(), load_tool())
    cell.gantry.position_mm = cell.gantry.limits_mm[gantry_end] if gantry_end is not None else 2500.0
    if purge:
        pt = base_world(cell)[:3, 3] + [-200.0, 0.0, 30.0]
        cell.purge = PurgeConfig(enabled=True, point=tuple(pt), spray_dir=(0.0, 0.0, -1.0), every_n_layers=1)
    path = plan_part(load_part("test_square"), load_process(), "layers", tilt_deg=90.0, cell=cell,
                     wrist=wrist_side(robot.tool))[1]
    return solver.solve(path, solver.Context(robot, cell, checker, log=lambda _: None))


def _snap(out, rep):
    return {"fixed": rep.fixed, "unfixed": [[i.sample, i.move, i.kind, i.detail] for i in rep.unfixed],
            "suggested": json.loads(json.dumps(rep.suggested)), "n_moves": len(out.moves),
            "gun": [bool(m.gun_on) for m in out.moves], "kind": [m.kind for m in out.moves],
            "tag": [m.tag for m in out.moves], "axes": [m.axes for m in out.moves]}


def _check(name, out, rep):
    poses = np.array([m.pose for m in out.moves])
    hint = np.zeros(0) if out.joint_hint is None else np.nan_to_num(out.joint_hint, nan=-9999.0)
    js, npz = GOLDEN / f"solver_{name}.json", GOLDEN / f"solver_{name}.npz"
    snap = _snap(out, rep)
    if UPDATE:
        js.write_text(json.dumps(snap))
        np.savez_compressed(npz, poses=poses, hint=hint)
        return
    assert json.loads(js.read_text()) == snap
    g = np.load(npz)
    assert g["poses"].shape == poses.shape and np.allclose(g["poses"], poses, rtol=0, atol=1e-9)
    assert g["hint"].shape == hint.shape and np.allclose(g["hint"], hint, rtol=0, atol=1e-9)


def test_golden_gantry_far_end():
    """Gantry at its low limit: the part is out of reach; the gantry lever fixes it (reach only)."""
    out, rep = _run(gantry_end=0)
    assert rep.fixed and "gantry_e1" in rep.suggested
    _check("gantry", out, rep)


def test_golden_purge_with_collisions():
    """Purge visits + synthetic collisions: arm and angle levers both act."""
    out, rep = _run(purge=True, checker=FakeChecker())
    assert len(rep.fixed) >= 2
    _check("purge_arm_angle", out, rep)
