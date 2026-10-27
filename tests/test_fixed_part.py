"""A part fixed in the cell, off the turntable: anywhere, table held at 0."""

import numpy as np
import pytest

from coldspray.models import load_cell, load_process
from coldspray.pipeline import load_part, plan_part
from coldspray.transforms import frame


def _off_table(fixed):
    part = load_part("test_block")
    part.placement = frame(1500.0, -800.0, -300.0, 30.0) @ part.placement    # beyond the faceplate, below it
    part.fixed_in_cell = fixed
    return part


def test_table_part_off_the_faceplate_is_refused():
    with pytest.raises(ValueError):
        plan_part(_off_table(False), load_process(), "layers", cell=load_cell())


def test_fixed_part_plans_anywhere_with_the_table_at_zero():
    cell = load_cell()
    summary, path = plan_part(_off_table(True), load_process(), "layers", tilt_deg=45.0, rotation_deg=90.0,
                              cell=cell, substrate=None)
    assert path.moves
    tt = cell.turntable
    for axes in path.axis_values():
        assert axes.get(tt.tilt.axis, 0.0) == 0.0 and axes.get(tt.rotation.axis, 0.0) == 0.0
    sprayed = np.array([m.pose[:3, 3] for m in path.moves if m.tag == "infill"])
    assert np.linalg.norm(sprayed[:, :2], axis=1).min() > 1000           # where the part is, not on the table


def test_fixed_part_cannot_revolve():
    with pytest.raises(ValueError, match="turntable"):
        plan_part(_off_table(True), load_process(), "revolve", cell=load_cell(), substrate=None)
