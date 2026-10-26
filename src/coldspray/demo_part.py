"""Command-line version of the GUI: STEP part -> toolpath -> KRL + viewer.

Run with:  python -m coldspray.demo_part [part.step | test_block | test_tube]
                   [--revolve] [--tilt=DEG] [--coats=N] [--no-view]
The part is placed with its lowest point on the faceplate (BASE z = 0).
For adjusting parameters interactively use the GUI:  python -m coldspray.gui
"""

import sys
from pathlib import Path

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.pipeline import load_part, plan_part
from coldspray.post.krl import to_krl


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    opts = dict(a[2:].split("=", 1) if "=" in a else (a[2:], "1") for a in argv if a.startswith("--"))
    files = [a for a in argv if not a.startswith("--")]
    proc, cell = load_process(), load_cell()
    name, v, f = load_part(files[0] if files else None)
    summary, path = plan_part(v, f, proc, name, mode="revolve" if "revolve" in opts else "layers",
                              tilt_deg=float(opts.get("tilt", 0)), coats=int(opts.get("coats", 1)), cell=cell)
    print(f"{name}: {summary}, {len(path.moves)} moves")

    robot = Robot(load_robot(), load_tool())
    out = Path("output")
    out.mkdir(exist_ok=True)
    src = out / f"{name}.src"
    src.write_text(to_krl(path, robot, cell, proc), newline="\r\n")
    print(f"Wrote {src}")

    if "no-view" not in opts:
        from coldspray.sim.scene import show_toolpath
        plate = None if "revolve" in opts else tuple(v[:, :2].max(axis=0) + 10)
        show_toolpath(path, part=(v, f), plate_size=plate, step=2.0)


if __name__ == "__main__":
    main()
