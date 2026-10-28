"""KRL PTP joints are the ones Simulate solves (post/joints.program_joints = viewer's ik_path + joint_hint)."""
import re

import numpy as np

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.post.joints import program_joints
from coldspray.post.krl import KRLOptions, to_krl

PTP = re.compile(r"^PTP \{A1 ([-\d.]+), A2 ([-\d.]+), A3 ([-\d.]+), A4 ([-\d.]+), A5 ([-\d.]+), A6 ([-\d.]+)")


def _plan():
    cell, proc = load_cell(), load_process()
    cell.gantry.position_mm = 2000.0
    _, path = plan_part(load_part("test_block"), proc, "layers", tilt_deg=90.0, cell=cell, roll_deg=-135.0)
    return path, Robot(load_robot(), load_tool()), cell, proc


def test_ptp_joints_equal_simulate():
    path, robot, cell, proc = _plan()
    text = to_krl(path, robot, cell, proc)
    written = [np.array(m.groups(), float) for l in text.split("\n") if (m := PTP.match(l))][1:-1]  # drop home
    pj = program_joints(path, robot, cell)
    ptp_rows = [pj[i] for i, m in enumerate(path.moves)
                if m.kind == "PTP" or (i > 0 and m.axes.get("E1") != path.moves[i - 1].axes.get("E1"))]
    assert written and len(written) == len(ptp_rows)
    for w, q in zip(written, ptp_rows):
        if not np.isnan(q).any():
            assert np.allclose(w, q, atol=1e-3)


def test_preflight_off_by_default():
    path, robot, cell, proc = _plan()
    assert "HALT" not in to_krl(path, robot, cell, proc)
    text = to_krl(path, robot, cell, proc, options=KRLOptions(preflight=True))
    lines = text.split("\n")
    assert lines[lines.index(f"BAS(#BASE, {cell.base_number})") + 1].startswith("IF ($ACT_TOOL<>")
