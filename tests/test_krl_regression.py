"""to_krl output for the built-in parts must not change unless a change means it to.

The hashes were recorded on unchanged code (main at 453e4e3). A deliberate
output change updates them and says so in the commit. Layers hashes updated
2026-10-27: moves between layers lift by jump_height_mm, not approach_height_mm.
ARC_CASES: ProcessConfig.arcs on (merged LINs + CIRC, S7A); test_block's round hole is
24 CIRCs (LIN 909 -> 595). test_repair and test_tube come out the same as with arcs off."""

import hashlib

import pytest

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.post.krl import to_krl

CASES = {  # part: (mode, table tilt, gun roll, E1, sha256 of the program); positions from auto_position
    "test_block": ("layers", 90.0, -135.0, 2000.0,
                   "71ecd0f87f7f2a0c3b32eed07619478bd1a9427f5ea2557c6449c93b9531fa76"),
    "test_repair": ("layers", 90.0, -135.0, 2000.0,
                    "89c900db1c213f9ce0d3728e98b1917ad136ec8606df47bb83e01896d7eb9cfc"),
    "test_tube": ("revolve", 0.0, -45.0, 2500.0,
                  "dcf903640c58a4c359fdc13c94474513e8b141a8fb4994e364faa0c11d943a28"),
}
ARC_CASES = {  # part: sha256 with proc.arcs = True (otherwise as CASES)
    "test_block": "0ee6266a267ac532220ad3af7847a12c0c0dce7557c9e3dad712952a929cc8f3",
}


def krl_text(name: str, arcs: bool = False) -> str:
    mode, tilt, roll, e1, _ = CASES[name]
    cell, proc = load_cell(), load_process()
    proc.arcs = arcs
    cell.gantry.position_mm = e1
    _, path = plan_part(load_part(name), proc, mode, tilt_deg=tilt, cell=cell, roll_deg=roll)
    return to_krl(path, Robot(load_robot(), load_tool()), cell, proc)


@pytest.mark.parametrize("name", CASES)
def test_krl_output_unchanged(name):
    assert hashlib.sha256(krl_text(name).encode()).hexdigest() == CASES[name][-1]


def test_arcs_off_by_default():
    assert load_process().arcs is False


@pytest.mark.parametrize("name", ARC_CASES)
def test_krl_output_with_arcs_unchanged(name):
    text = krl_text(name, arcs=True)
    assert "$CIRC_TYPE = #PATH" in text
    assert hashlib.sha256(text.encode()).hexdigest() == ARC_CASES[name]
