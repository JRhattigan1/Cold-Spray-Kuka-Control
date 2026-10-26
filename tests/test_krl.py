import re

import numpy as np
import pytest

from coldspray.cell import base_in_robot
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


def _rotate_in_place(robot, cell):
    """Approach, then spin the table 90 deg with the nozzle over one spot."""
    p = robot.fk([0, -90, 90, 0, 60, 0])  # some reachable pose, robot frame
    pose = np.linalg.inv(base_in_robot(cell)) @ p
    return Toolpath("spin", [Move(pose, 100, kind="PTP", axes={"E2": 0.0, "E5": 0.0}),
                             Move(pose, 100, gun_on=True),
                             Move(pose, 50, gun_on=True, axes={"E5": 90.0})])


def test_turntable_axes_written_and_timed(setup):
    robot, cell, proc = setup
    src = to_krl(_rotate_in_place(robot, cell), robot, cell, proc).splitlines()
    ptp = [l for l in src if l.startswith("PTP {A1") and "E1" in l][0]
    assert "E2 0.000" in ptp and "E5 0.000" in ptp
    last_lin = [l for l in src if l.startswith("LIN")][-1]
    assert "E5 90.000" in last_lin
    # 0 mm over the part: the table rotation sets the time, at full speed
    assert "$VEL_EXTAX[5] = 100.0" in src


def test_coupled_mode_writes_part_relative_points(setup):
    robot, cell, proc = setup
    coupled = cell.model_copy(deep=True)
    coupled.turntable.coupled = True
    path = _rotate_in_place(robot, coupled)
    lins = [l for l in to_krl(path, robot, coupled, proc).splitlines() if l.startswith("LIN")]
    xyz = lambda l: [float(v) for v in re.findall(r"[XYZ] (-?\d+\.\d+)", l)]
    assert np.allclose(xyz(lins[0]), xyz(lins[-1]))  # same spot on the part despite the rotation


def test_unreachable_ptp_raises(setup):
    robot, cell, proc = setup
    path = Toolpath("bad", [Move(frame(9000, 0, 0), 100, kind="PTP")])
    with pytest.raises(KRLError):
        to_krl(path, robot, cell, proc)
