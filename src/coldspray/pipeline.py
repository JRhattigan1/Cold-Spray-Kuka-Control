"""STEP part -> placed mesh -> layers -> toolpath."""

from pathlib import Path

from coldspray.cad.slicer import place_on_plate, slice_mesh
from coldspray.models import CellConfig, ProcessConfig
from coldspray.planner.layers import plan_layers
from coldspray.planner.revolve import plan_revolve


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


def load_part(step_file=None, deflection=0.05):
    """(name, vertices, faces) with the part's lowest point at z = 0.
    `step_file` is a STEP path or the name of a built-in test part
    (default: the test block)."""
    from coldspray.cad.step_io import load_step, tessellate

    step_file = step_file or "test_block"
    builtin = BUILT_IN.get(step_file)
    shape = builtin() if builtin else load_step(step_file)
    v, f = tessellate(shape, deflection=deflection)
    name = program_name(step_file if builtin else Path(step_file).stem)
    return name, place_on_plate(v), f


MODES = {
    "layers": "Layers (build up on the faceplate)",
    "revolve": "Revolve (coat the outside, table spinning)",
}


def plan_part(vertices, faces, proc: ProcessConfig, name="part", mode="layers",
              tilt_deg=0.0, rotation_deg=0.0, coats=1, cell: CellConfig | None = None):
    """Returns (summary, toolpath). `summary` is a short description for the user."""
    tt = cell.turntable if cell else None
    tilt_axis, rot_axis = (tt.tilt.axis, tt.rotation.axis) if tt else ("E2", "E5")
    if mode == "revolve":
        path = plan_revolve(vertices, faces, proc, name, coats=coats, tilt_deg=tilt_deg,
                            tilt_axis=tilt_axis, rotation_axis=rot_axis)
        passes = sum(1 for a, b in zip(path.moves, path.moves[1:]) if b.gun_on and a.gun_on
                     and abs(b.pose[2, 3] - a.pose[2, 3]) > 1e-9)
        return f"{coats} coat(s), {passes + coats} passes", path
    layers = slice_mesh(vertices, faces, proc.layer_thickness_mm)
    table = {tilt_axis: tilt_deg, rot_axis: rotation_deg} if tt else {}
    return f"{len(layers)} layers", plan_layers(layers, proc, name, table_axes=table)
