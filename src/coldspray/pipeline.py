"""STEP part -> placed mesh -> layers -> toolpath."""

from pathlib import Path

from coldspray.cad.slicer import place_on_plate, slice_mesh
from coldspray.models import ProcessConfig
from coldspray.planner.layers import plan_layers


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


def load_part(step_file=None, deflection=0.05):
    """(name, vertices, faces) with the part's lowest point at z = 0.
    Without a file, the built-in test block is used."""
    from coldspray.cad.step_io import load_step, tessellate

    shape = load_step(step_file) if step_file else test_block()
    v, f = tessellate(shape, deflection=deflection)
    name = program_name(Path(step_file).stem if step_file else "test_block")
    return name, place_on_plate(v), f


def plan_part(vertices, faces, proc: ProcessConfig, name="part"):
    layers = slice_mesh(vertices, faces, proc.layer_thickness_mm)
    return layers, plan_layers(layers, proc, name)
