"""Surface mode wiring in plan_part (the planner itself is faked here)."""

import sys
import types

import numpy as np
import pytest

from coldspray.cad.part import Substrate, placed_vertices
from coldspray.models import load_cell, load_process
from coldspray.pipeline import MODES, load_part, plan_part
from coldspray.toolpath.model import Move, Toolpath, spray_frame


@pytest.fixture
def fake_surface(monkeypatch):
    """Replace coldspray.planner.surface with a recorder returning a 2-move path."""
    calls = {}

    def plan_surface(vertices, faces, coat_mask, proc, name, table_axes, coats=1, max_tilt_deg=60.0):
        calls.update(vertices=vertices, faces=faces, mask=coat_mask, name=name, table=table_axes,
                     coats=coats, max_tilt=max_tilt_deg)
        pose = spray_frame([0, 0, 100], [0, 0, -1], [1, 0, 0])
        return Toolpath(name, [Move(pose, 10.0, True, axes=dict(table_axes)),
                               Move(pose.copy(), 10.0, True, axes=dict(table_axes))])

    monkeypatch.setitem(sys.modules, "coldspray.planner.surface",
                        types.SimpleNamespace(plan_surface=plan_surface))
    return calls


def test_surface_mode_listed():
    assert "surface" in MODES


def test_surface_empty_selection_sends_user_to_part_step(fake_surface):
    part = load_part("test_repair")
    with pytest.raises(ValueError, match="Part step"):
        plan_part(part, load_process(), mode="surface")
    assert not fake_surface


def test_surface_routes_with_mask_on_any_body(fake_surface):
    part = load_part("test_repair")
    other = 1 - part.build_body                       # a face on the big component, not the patch
    face = int(part.face_ids[part.body_ids == other][0])
    patch = int(part.face_ids[part.body_ids == part.build_body][0])
    part.coat_faces = {face, patch}
    cell = load_cell()
    tt = cell.turntable
    summary, path = plan_part(part, load_process(), mode="surface", tilt_deg=15.0, rotation_deg=30.0,
                              cell=cell, substrate=Substrate())
    assert len(path.moves) == 2 and "2 face" in summary
    np.testing.assert_allclose(fake_surface["vertices"], placed_vertices(part))
    assert fake_surface["faces"].shape == part.faces.shape
    np.testing.assert_array_equal(fake_surface["mask"], np.isin(part.face_ids, [face, patch]))
    assert fake_surface["mask"][part.body_ids == other].any()
    assert fake_surface["table"] == {tt.tilt.axis: 15.0, tt.rotation.axis: 30.0}
    assert fake_surface["coats"] == getattr(load_process(), "surface_coats", 1)
    assert fake_surface["name"] == part.name
