"""STEP part -> placed on the faceplate -> layers -> toolpath.

The part (one body, or an assembly with one build body) stands on an optional
substrate plate on the faceplate. The build body is sliced from its own base
upwards; planning is done relative to that base and the toolpath moved up
into BASE.
"""

from pathlib import Path

import numpy as np

from coldspray.cad.part import Part, Substrate, build_frame, build_mesh, drop_on_plate, placed_vertices
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


def test_repair():
    """Two bodies: a 120 x 80 x 30 mm component with a 40 x 20 x 6 mm patch to
    build on its top face (the patch is the smaller, second body)."""
    from OCP.BRep import BRep_Builder
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    from OCP.TopoDS import TopoDS_Compound

    both, builder = TopoDS_Compound(), BRep_Builder()
    builder.MakeCompound(both)
    builder.Add(both, BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 120, 80, 30).Shape())
    builder.Add(both, BRepPrimAPI_MakeBox(gp_Pnt(30, 40, 30), 40, 20, 6).Shape())
    return both


BUILT_IN = {"test_block": test_block, "test_tube": test_tube, "test_repair": test_repair}


def load_part(source=None, deflection=0.05) -> Part:
    """A Part dropped onto the default substrate plate, centred on the table.
    `source` is a STEP path or the name of a built-in test part
    (default: the test block). With several bodies the smallest is taken as
    the build body (change `build_body` to pick another)."""
    from coldspray.cad.step_io import load_step, tessellate_bodies

    source = source or "test_block"
    builtin = BUILT_IN.get(source)
    shape = builtin() if builtin else load_step(source)
    v, f, ids, planar, body_ids, cylinders = tessellate_bodies(shape, deflection=deflection)
    part = Part(program_name(source if builtin else Path(source).stem), v, f, ids, planar,
                body_ids=body_ids if body_ids.max() > 0 else None, cylinders=cylinders)
    if part.n_bodies > 1:
        part.build_body = int(np.argmin([body_size(part, b) for b in range(part.n_bodies)]))
    part.placement = drop_on_plate(part, Substrate().top)
    return part


def body_size(part: Part, body: int) -> float:
    """Bounding-box volume of one body (mm^3)."""
    faces = part.faces if part.body_ids is None else part.faces[part.body_ids == body]
    return float(np.prod(np.ptp(part.vertices[np.unique(faces)], axis=0)))


MODES = {
    "layers": "Layers (build up on the substrate plate)",
    "revolve": "Revolve (coat the outside, table spinning)",
}


def check_placement(vertices, substrate: Substrate | None, mode="layers"):
    """Raises ValueError (with a message for the user) if the placed part
    (all bodies) can't be sprayed. No substrate: the floor is the faceplate.
    A part held above the floor (fixture, riser) is fine."""
    floor, what = (substrate.top, "the substrate plate") if substrate else (0.0, "the faceplate")
    zmin = vertices[:, 2].min()
    if zmin < floor - 0.1:
        raise ValueError(f"Part goes into {what} by {floor - zmin:.1f} mm — snap a face down or raise it")
    reach = np.linalg.norm(vertices[:, :2], axis=1).max()
    if reach > FACEPLATE_RADIUS:
        raise ValueError(f"Part reaches {reach:.0f} mm from the table axis, beyond the"
                         f" {FACEPLATE_RADIUS:.0f} mm faceplate — move it towards the centre")


def plan_part(part: Part, proc: ProcessConfig, mode="layers", tilt_deg=0.0, rotation_deg=0.0,
              coats=1, cell: CellConfig | None = None, substrate: Substrate | None = Substrate(),
              roll_deg=0.0):
    """Returns (summary, toolpath in BASE) for the build body. `summary` is a
    short description for the user. `substrate` None: no plate, the part
    stands on the faceplate. `roll_deg` turns the gun about its spray axis
    (for reach; the spray is unchanged)."""
    check_placement(placed_vertices(part), substrate, mode)
    v, faces = build_mesh(part)
    tt = cell.turntable if cell else None
    tilt_axis, rot_axis = (tt.tilt.axis, tt.rotation.axis) if tt else ("E2", "E5")
    if mode == "revolve":                       # works from the floor
        floor = substrate.top if substrate else 0.0
        v = v - [0.0, 0.0, floor]
        up = frame(z=floor)
        path = plan_revolve(v, faces, proc, part.name, coats=coats, tilt_deg=tilt_deg,
                            tilt_axis=tilt_axis, rotation_axis=rot_axis).transformed(up).rolled(roll_deg)
        passes = sum(1 for a, b in zip(path.moves, path.moves[1:]) if b.gun_on and a.gun_on
                     and abs(b.pose[2, 3] - a.pose[2, 3]) > 1e-9)
        return f"{coats} coat(s), {passes + coats} passes", path
    # Layers grow away from the start face, or up from the bottom of the build
    # body (on the plate, or on the component a repair patch sits on)
    to_base = build_frame(part)
    layers = slice_mesh((v - to_base[:3, 3]) @ to_base[:3, :3], faces, proc.layer_thickness_mm)
    table = {tilt_axis: tilt_deg, rot_axis: rotation_deg} if tt else {}
    path = plan_layers(layers, proc, part.name, table_axes=table).transformed(to_base).rolled(roll_deg)
    start = f" from start face {part.start_face}" if part.start_face is not None else ""
    return f"{len(layers)} layers{start}", path
