"""Phase 2 demo: STEP part -> layers -> spray passes -> KRL + viewer.

Run with:  python -m coldspray.demo_part path\\to\\part.step   [--no-view]
Without a STEP path, a 40 x 30 x 5 mm test block with a hole is used.
The part is placed with its lowest point on the substrate (BASE z = 0).
"""

import sys
from pathlib import Path

from coldspray.cad.slicer import place_on_plate, slice_mesh
from coldspray.cad.step_io import load_step, tessellate
from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.planner.layers import plan_layers
from coldspray.post.krl import to_krl


def _test_block():
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    box = BRepPrimAPI_MakeBox(gp_Pnt(10, 10, 0), 40, 30, 5).Shape()
    hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(30, 25, -1), gp_Dir(0, 0, 1)), 5, 10).Shape()
    return BRepAlgoAPI_Cut(box, hole).Shape()


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    files = [a for a in argv if not a.startswith("--")]
    shape = load_step(files[0]) if files else _test_block()
    name = Path(files[0]).stem if files else "test_block"
    name = "".join(c if c.isalnum() else "_" for c in name)[:24]

    proc, cell = load_process(), load_cell()
    v, f = tessellate(shape, deflection=0.05)
    v = place_on_plate(v)
    layers = slice_mesh(v, f, proc.layer_thickness_mm)
    path = plan_layers(layers, proc, name)
    print(f"{name}: {len(layers)} layers, {len(path.moves)} moves")

    robot = Robot(load_robot(), load_tool())
    out = Path("output")
    out.mkdir(exist_ok=True)
    src = out / f"{name}.src"
    src.write_text(to_krl(path, robot, cell, proc), newline="\r\n")
    print(f"Wrote {src}")

    if "--no-view" not in argv:
        from coldspray.sim.scene import show_toolpath
        show_toolpath(path, part=(v, f), plate_size=tuple(v[:, :2].max(axis=0) + 10), step=2.0)


if __name__ == "__main__":
    main()
