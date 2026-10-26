"""Load everything the 3D view needs from the config files."""

from dataclasses import dataclass

import numpy as np

from coldspray.cell import base_in_robot, gantry_position, robroot
from coldspray.kinematics.robot import Robot
from coldspray.models import PROJECT_ROOT, CellConfig, ToolConfig, load_cell, load_robot, load_tool
from coldspray.toolpath.model import Toolpath


@dataclass
class Assets:
    cell: CellConfig
    tool: ToolConfig
    robot: Robot
    meshes: object = None          # CellMeshes, or None without the cell CAD
    gun_mesh: tuple | None = None  # (vertices, faces) of the gun CAD


def load_assets(log=print) -> Assets:
    from coldspray.cad.mesh_cache import cached_mesh
    from coldspray.sim.cell_model import load_cell_meshes

    cell, tool = load_cell(), load_tool()
    assets = Assets(cell, tool, Robot(load_robot(), tool))
    if cell.cad_cache and (PROJECT_ROOT / cell.cad_cache).with_suffix(".json").exists():
        log("Loading cell CAD...")
        assets.meshes = load_cell_meshes(cell.cad_cache)
    else:
        log("No cell CAD cache found - drawing a simple robot. Make one with coldspray.cad.step_cache.")
    if tool.gun is not None and (PROJECT_ROOT / tool.gun.step_file).exists():
        log("Loading spray gun CAD...")
        assets.gun_mesh = cached_mesh(tool.gun.step_file, deflection=0.5)
    return assets


def make_gun(tool: ToolConfig, gun_mesh):
    """Gun (and selected nozzle) meshes for the viewer, in gun-CAD coordinates."""
    from coldspray.cad.mesh_cache import cached_mesh
    from coldspray.sim.viewer import Gun
    from coldspray.tool import gun_to_flange, nozzle_to_gun

    if tool.gun is None or gun_mesh is None:
        return None
    nozzle = None
    n = tool.gun.current_nozzle
    if n.step_file and (PROJECT_ROOT / n.step_file).exists():
        v, f = cached_mesh(n.step_file, deflection=0.2)
        m = nozzle_to_gun(n)
        nozzle = (v @ m[:3, :3].T + m[:3, 3], f)
    return Gun(*gun_mesh, gun_to_flange(tool.gun), tool.gun.nozzle_length, nozzle)


def gantry_offset(cell: CellConfig) -> np.ndarray:
    return np.asarray(cell.gantry.direction) * gantry_position(cell) if cell.gantry else np.zeros(3)


def show_toolpath(path: Toolpath, part=None, plate_size=None, step=5.0, screenshot=None):
    from coldspray.sim.viewer import show

    a = load_assets()
    return show(a.robot, path, base_in_robot(a.cell), world_from_robot=robroot(a.cell), cell=a.meshes,
                gantry_offset=gantry_offset(a.cell), cad_pose=a.cell.robot_cad_pose_deg,
                gun=make_gun(a.tool, a.gun_mesh), part=part, plate_size=plate_size, step=step,
                home=a.cell.home_deg, screenshot=screenshot)
