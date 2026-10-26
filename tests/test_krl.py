import re

import numpy as np
import pytest

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.post.krl import KRLError, to_krl
from coldspray.toolpath.model import Move, Toolpath
from coldspray.toolpath.raster import plate_raster
from coldspray.transforms import frame


@pytest.fixture(scope="module")
def setup():
    return Robot(load_robot(), load_tool()), load_cell(), load_process()


def test_program_structure(setup):
    robot, cell, proc = setup
    src = to_krl(plate_raster(proc, 100, 100), robot, cell, proc)
    lines = src.splitlines()

    assert lines[2] == "DEF plate_raster( )"
    assert lines[-1] == "END"
    assert f"BAS(#BASE, {cell.base_number})" in lines
    assert f"BAS(#TOOL, {cell.tool_number})" in lines
    on = lines.index(f"$OUT[{cell.gun_output}] = TRUE")
    off = lines.index(f"$OUT[{cell.gun_output}] = FALSE", on)
    assert lines[on - 1] == "WAIT SEC 0" and lines[off - 1] == "WAIT SEC 0"
    assert lines[on + 1].startswith("WAIT SEC 2.0")
    # every spray pass is a LIN between on and off; all but the last are blended
    passes = [l for l in lines[on:off] if l.startswith("LIN")]
    assert len(passes) == 2 * 51 - 1
    assert all(l.endswith("C_DIS") for l in passes[:-1]) and not passes[-1].endswith("C_DIS")


def test_lin_values_are_base_relative(setup):
    robot, cell, proc = setup
    src = to_krl(plate_raster(proc, 100, 100), robot, cell, proc)
    first_lin = next(l for l in src.splitlines() if l.startswith("LIN"))
    nums = [float(v) for v in re.findall(r"[XYZABC] (-?\d+\.\d+)", first_lin)]
    assert np.allclose(nums[:3], [-proc.overrun_mm, 0, proc.standoff_mm])


def test_unreachable_ptp_raises(setup):
    robot, cell, proc = setup
    path = Toolpath("bad", [Move(frame(9000, 0, 0), 100, kind="PTP")])
    with pytest.raises(KRLError):
        to_krl(path, robot, cell, proc)
