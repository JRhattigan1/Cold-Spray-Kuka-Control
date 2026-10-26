"""STEP part -> placed on the substrate plate -> layers -> toolpath.

The part is built up on a substrate plate lying on the faceplate; planning is
done relative to the plate top and the toolpath moved up into BASE.
"""

from pathlib import Path

import numpy as np

from coldspray.cad.part import Part, Substrate, drop_on_plate, placed_vertices
from coldspray.cad.slicer import slice_mesh
from coldspray.models import CellConfig, ProcessConfig
from coldspray.planner.layers import plan_layers
from coldspray.planner.revolve import plan_revolve
from coldspray.transforms import frame

FACEPLATE_RADIUS = 500.0  # mm


def test_block():
    """40 x 30 x 5 mm block with a 10 mm through hole."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    box = BRepPrimAPI_MakeBox(gp_Pnt(10, 10, 0), 40, 30, 5).Shape()
    hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(30, 25, -1), gp_Dir(0, 0, 1)), 5, 10).Shape()
    return BRepAlgoAPI_Cut(box, hole).Shape()


def program_name(text: str) -> str:
    """A valid KRL program name (letters, digits, _; max 24 chars)."""
    name = "".join(c if c.isalnum() else "_" for c in text)[:24]
    return name if name[:1].isalpha() else f"p_{name}"[:24]


def test_tube():
    """Tube, 120 mm OD x 150 mm long, 80 mm bore - for trying the revolve mode."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    outer = BRepPrimAPI_MakeCylinder(axis, 60, 150).Shape()
    bore = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), 40, 152).Shape()
    return BRepAlgoAPI_Cut(outer, bore).Shape()


BUILT_IN = {"test_block": test_block, "test_tube": test_tube}


def load_part(source=None, deflection=0.05) -> Part:
    """A Part dropped onto the default substrate plate, centred on the table.
    `source` is a STEP path or the name of a built-in test part
    (default: the test block)."""
    from coldspray.cad.step_io import load_step, tessellate_with_ids

    source = source or "test_block"
    builtin = BUILT_IN.get(source)
    shape = builtin() if builtin else load_step(source)
    v, f, ids, planar = tessellate_with_ids(shape, deflection=deflection)
    part = Part(program_name(source if builtin else Path(source).stem), v, f, ids, planar)
    part.placement = drop_on_plate(part, Substrate().top)
    return part


MODES = {
    "layers": "Layers (build up on the substrate plate)",
    "revolve": "Revolve (coat the outside, table spinning)",
}


def check_placement(vertices, substrate: Substrate, mode="layers"):
    """Raises ValueError (with a message for the user) if the placed part can't be sprayed."""
    zmin = vertices[:, 2].min()
    if zmin < substrate.top - 0.1:
        raise ValueError(f"Part goes into the substrate plate by {substrate.top - zmin:.1f} mm"
                         " — snap a face to the plate or raise it")
    if mode == "layers" and zmin > substrate.top + 0.1:
        raise ValueError(f"Part floats {zmin - substrate.top:.1f} mm above the substrate plate"
                         " — snap a face to the plate to start the build")
    reach = np.linalg.norm(vertices[:, :2], axis=1).max()
    if reach > FACEPLATE_RADIUS:
        raise ValueError(f"Part reaches {reach:.0f} mm from the table axis, beyond the"
                         f" {FACEPLATE_RADIUS:.0f} mm faceplate — move it towards the centre")


def plan_part(part: Part, proc: ProcessConfig, mode="layers", tilt_deg=0.0, rotation_deg=0.0,
              coats=1, cell: CellConfig | None = None, substrate=Substrate()):
    """Returns (summary, toolpath in BASE). `summary` is a short description for the user."""
    v = placed_vertices(part)
    check_placement(v, substrate, mode)
    v = v - [0.0, 0.0, substrate.top]  # plan relative to the plate top
    up = frame(z=substrate.top)
    tt = cell.turntable if cell else None
    tilt_axis, rot_axis = (tt.tilt.axis, tt.rotation.axis) if tt else ("E2", "E5")
    if mode == "revolve":
        path = plan_revolve(v, part.faces, proc, part.name, coats=coats, tilt_deg=tilt_deg,
                            tilt_axis=tilt_axis, rotation_axis=rot_axis).transformed(up)
        passes = sum(1 for a, b in zip(path.moves, path.moves[1:]) if b.gun_on and a.gun_on
                     and abs(b.pose[2, 3] - a.pose[2, 3]) > 1e-9)
        return f"{coats} coat(s), {passes + coats} passes", path
    layers = slice_mesh(v, part.faces, proc.layer_thickness_mm)
    table = {tilt_axis: tilt_deg, rot_axis: rotation_deg} if tt else {}
    path = plan_layers(layers, proc, part.name, table_axes=table).transformed(up)
    return f"{len(layers)} layers", path
