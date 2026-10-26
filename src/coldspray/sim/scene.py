"""Open the viewer for a toolpath with everything taken from the config files."""

import numpy as np

from coldspray.cell import base_in_robot, gantry_position, robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import PROJECT_ROOT, load_cell, load_robot, load_tool
from coldspray.toolpath.model import Toolpath


def show_toolpath(path: Toolpath, part=None, plate_size=None, step=5.0, screenshot=None):
    from coldspray.cad.mesh_cache import cached_mesh
    from coldspray.sim.cell_model import load_cell_meshes
    from coldspray.sim.viewer import Gun, show
    from coldspray.tool import gun_to_flange

    cell, tool = load_cell(), load_tool()
    robot = Robot(load_robot(), tool)

    meshes = None
    if cell.cad_cache and (PROJECT_ROOT / cell.cad_cache).with_suffix(".json").exists():
        meshes = load_cell_meshes(cell.cad_cache)
    else:
        print("No cell CAD cache found - drawing a simple robot. Make one with coldspray.cad.step_cache.")

    gun = None
    if tool.gun is not None and (PROJECT_ROOT / tool.gun.step_file).exists():
        v, f = cached_mesh(tool.gun.step_file, deflection=0.5)
        gun = Gun(v, f, gun_to_flange(tool.gun), tool.gun.nozzle_length)

    offset = np.asarray(cell.gantry.direction) * gantry_position(cell) if cell.gantry else np.zeros(3)
    return show(robot, path, base_in_robot(cell), world_from_robot=robroot(cell), cell=meshes,
                gantry_offset=offset, cad_pose=cell.robot_cad_pose_deg, gun=gun, part=part,
                plate_size=plate_size, step=step, home=cell.home_deg, screenshot=screenshot)
