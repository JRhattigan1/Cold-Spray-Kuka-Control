"""3D viewer: cell, robot with spray gun, part and toolpath.

Drag the slider to move along the path, press "p" to play, "e" to show or
hide the booth walls. Everything is drawn in WORLD (cell CAD) coordinates.
"""

from dataclasses import dataclass

import numpy as np
import pyvista as pv

from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.sim.cell_model import CellMeshes
from coldspray.toolpath.model import Toolpath

SPRAY = "#f28c28"
TRAVEL = "#9aa0a6"
BAD = "#d93025"
KUKA_ORANGE = "#ff6a13"


@dataclass
class Gun:
    vertices: np.ndarray
    faces: np.ndarray
    to_flange: np.ndarray      # gun CAD -> flange frame
    nozzle_length: float


def _polydata(v, f) -> pv.PolyData:
    f = np.asarray(f)
    return pv.PolyData(np.asarray(v, float), np.hstack([np.full((len(f), 1), 3), f]).ravel())


def _polyline(points) -> pv.PolyData:
    line = pv.PolyData(np.asarray(points, float))
    line.lines = np.hstack([[len(points)], np.arange(len(points))])
    return line


def show(robot: Robot, path: Toolpath, base_in_robot: np.ndarray, world_from_robot=None,
         cell: CellMeshes | None = None, gantry_offset=(0.0, 0.0, 0.0), cad_pose=HOME_DEG,
         gun: Gun | None = None, part=None, plate_size=None, step=5.0, home=None,
         screenshot=None):
    """`part` is an optional (vertices, faces) mesh in BASE coordinates.
    `gantry_offset` is how far the gantry carriage has moved from the CAD position."""
    world_from_robot = np.eye(4) if world_from_robot is None else world_from_robot
    world_from_base = world_from_robot @ base_in_robot

    poses, gun_on, _ = path.sample(step)
    joints = robot.ik_path([base_in_robot @ p for p in poses], seed_deg=HOME_DEG if home is None else home)
    reachable = ~np.isnan(joints[:, 0])
    world = np.array([world_from_base @ p for p in poses])
    pts = world[:, :3, 3]

    pl = pv.Plotter(off_screen=screenshot is not None, window_size=(1400, 900))
    pl.set_background("white")
    pl.enable_anti_aliasing("ssaa")

    # --- cell -----------------------------------------------------------
    link_actors, stick = [], None
    if cell is not None:
        pl.add_mesh(cell.static, color="#c9cfd4", smooth_shading=True)
        walls = pl.add_mesh(cell.enclosure, color="#dfe6ea", opacity=0.12)
        walls.SetVisibility(False)
        carriage = pl.add_mesh(cell.carriage, color="#9fb3c1", smooth_shading=True)
        shift = np.eye(4)
        shift[:3, 3] = gantry_offset
        carriage.user_matrix = shift
        pl.add_key_event("e", lambda: walls.SetVisibility(not walls.GetVisibility()))
        cad_frames = robot.link_frames(cad_pose)
        for k, mesh in enumerate(cell.robot):
            color = "#3c3c3c" if k == 0 else KUKA_ORANGE
            link_actors.append(pl.add_mesh(mesh, color=color, smooth_shading=True))
        link_actors[0].user_matrix = world_from_robot
    else:
        pl.add_mesh(pv.Plane(center=world_from_robot[:3, 3] * [1, 1, 0], i_size=6000, j_size=6000), color="#eeeeee")
        stick = _polyline(np.zeros((7, 3)))
        pl.add_mesh(stick, color=KUKA_ORANGE, line_width=14, render_lines_as_tubes=True)

    gun_actor = nozzle_actor = None
    if gun is not None:
        gun_actor = pl.add_mesh(_polydata(gun.vertices, gun.faces), color="#8d99a6", smooth_shading=True)
        nozzle = pv.Cylinder(center=(0, gun.nozzle_length / 2, 0), direction=(0, 1, 0),
                             radius=14, height=gun.nozzle_length)
        nozzle_actor = pl.add_mesh(nozzle, color="#5f6b77")

    # --- workpiece and path ------------------------------------------------
    if plate_size is not None:
        w, l = plate_size
        pl.add_mesh(pv.Box(bounds=(0, w, 0, l, -10, 0)).transform(world_from_base, inplace=False), color="#b0bec5")
    if part is not None:
        pl.add_mesh(_polydata(*part).transform(world_from_base, inplace=False), color="#4f6d7a", opacity=0.4)
    for mask, color, width in ((gun_on, SPRAY, 3), (~gun_on, TRAVEL, 1)):
        segs = [i for i in range(1, len(pts)) if mask[i]]
        if segs:
            lines = np.hstack([[2, i - 1, i] for i in segs])
            pl.add_mesh(pv.PolyData(pts, lines=lines), color=color, line_width=width)
    if (~reachable).any():
        pl.add_points(pts[~reachable], color=BAD, point_size=10, render_points_as_spheres=True)
    spray = _polyline([pts[0], pts[0]])
    pl.add_mesh(spray, color="#1a73e8", line_width=4)
    status = pl.add_text("", position="upper_left", font_size=10, color="black")

    def pose_robot(q):
        flange = world_from_robot @ robot.flange(q)
        if link_actors:
            frames = robot.link_frames(q)
            for k in range(1, 7):
                link_actors[k].user_matrix = world_from_robot @ frames[k - 1] @ np.linalg.inv(cad_frames[k - 1])
        else:
            pts7 = robot.link_points(q)
            stick.points = (world_from_robot @ np.c_[pts7, np.ones(7)].T).T[:, :3]
        if gun_actor is not None:
            gun_actor.user_matrix = nozzle_actor.user_matrix = flange @ gun.to_flange

    def update(value):
        i = int(round(value))
        if reachable[i]:
            pose_robot(joints[i])
            tcp = world[i]
            spray.points = np.array([tcp[:3, 3], tcp[:3, 3] + tcp[:3, 0] * 40])
            j = ", ".join(f"{v:.1f}" for v in joints[i])
            status.SetText(2, f"{path.name}  point {i}/{len(pts) - 1}  spray {'ON' if gun_on[i] else 'off'}\nA1..A6: {j}")
        else:
            status.SetText(2, f"point {i}: UNREACHABLE")

    first = int(np.argmax(reachable)) if reachable.any() else 0
    slider = pl.add_slider_widget(update, [0, len(pts) - 1], value=first, title="Path position",
                                  pointa=(0.25, 0.08), pointb=(0.75, 0.08), fmt="%.0f")

    def play():
        state = {"i": int(slider.GetRepresentation().GetValue())}

        def tick(_step):
            state["i"] = (state["i"] + 1) % len(pts)
            slider.GetRepresentation().SetValue(state["i"])
            update(state["i"])

        pl.add_timer_event(max_steps=len(pts), duration=30, callback=tick)

    pl.add_key_event("p", play)
    unreachable = int((~reachable).sum())
    pl.add_text(f"{unreachable} unreachable points" if unreachable else "all points reachable",
                position="upper_right", font_size=10, color=BAD if unreachable else "green")
    pl.add_text("p: play   e: walls   drag: rotate   scroll: zoom", position="lower_left",
                font_size=8, color="#555555")

    target = world_from_base[:3, 3]
    pl.camera_position = [target + [-2600, -4200, 2600], target + [600, 300, 700], (0, 0, 1)]
    update(first)
    if screenshot:
        pl.show(screenshot=screenshot)
    else:
        pl.show()
    return joints
