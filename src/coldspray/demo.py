"""Phase 1 demo: raster over a 100 x 100 mm plate.

Writes output/plate_raster.src and opens the 3D viewer.
Run with:  python -m coldspray.demo   (add --no-view to skip the viewer)
"""

import sys
from pathlib import Path

from coldspray.kinematics.robot import Robot
from coldspray.models import load_cell, load_process, load_robot, load_tool
from coldspray.post.krl import to_krl
from coldspray.toolpath.raster import plate_raster


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    robot = Robot(load_robot(), load_tool())
    cell, proc = load_cell(), load_process()
    path = plate_raster(proc, 100.0, 100.0)

    out = Path("output")
    out.mkdir(exist_ok=True)
    src = out / f"{path.name}.src"
    src.write_text(to_krl(path, robot, cell, proc), newline="\r\n")
    print(f"Wrote {src} ({len(path.moves)} moves)")

    if "--no-view" not in argv:
        from coldspray.sim.scene import show_toolpath
        show_toolpath(path, plate_size=(100.0, 100.0))


if __name__ == "__main__":
    main()
