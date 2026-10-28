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


def test_nan_ptp_seeds_from_last_valid_joints_and_warns(monkeypatch, caplog):
    """S7K/20: a PTP Simulate couldn't solve is seeded from the last joints it did solve (any
    move, not the last PTP), and the fallback is logged."""
    import coldspray.post.krl as krl
    from dataclasses import replace
    path, robot, cell, proc = _plan()
    k = len(path.moves) // 2
    path = replace(path, moves=[replace(m, kind="PTP", gun_on=False) if i == k else m
                                for i, m in enumerate(path.moves)])
    pj = program_joints(path, robot, cell)
    pj[k] = np.nan
    assert not np.isnan(pj[k - 1]).any()
    monkeypatch.setattr(krl, "program_joints", lambda *a: pj)
    seeds, ik = [], robot.ik
    monkeypatch.setattr(robot, "ik", lambda pose, seed: seeds.append(np.array(seed)) or ik(pose, seed))
    with caplog.at_level("WARNING", logger="coldspray.post.krl"):
        to_krl(path, robot, cell, proc)
    assert len(seeds) == 1 and np.allclose(seeds[0], pj[k - 1])
    assert f"move {k}" in caplog.text


def test_manual_spray_comments_the_outputs_only():
    """spray_output=False (spray switched by hand): every $OUT line becomes a comment and
    nothing else changes, so stops and settle waits keep the program's timing."""
    path, robot, cell, proc = _plan()
    auto = to_krl(path, robot, cell, proc, options=KRLOptions())
    manual = to_krl(path, robot, cell, proc, options=KRLOptions(spray_output=False))
    out = f"$OUT[{cell.gun_output}]"
    assert out in auto and not any(l.startswith(out) for l in manual.split("\n"))
    assert manual == auto.replace(f"\n{out}", f"\n;{out}")
