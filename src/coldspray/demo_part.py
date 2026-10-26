"""Phase 2 demo: STEP part -> layers -> spray passes -> KRL + viewer.

Run with:  python -m coldspray.demo_part path\\to\\part.step   [--no-view]
Without a STEP path, a 40 x 30 x 5 mm test block with a hole is used.
The part is placed with its lowest point on the substrate (BASE z = 0).
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
    files = [a for a in argv if not a.startswith("--")]
    proc, cell = load_process(), load_cell()
    name, v, f = load_part(files[0] if files else None)
    layers, path = plan_part(v, f, proc, name)
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
