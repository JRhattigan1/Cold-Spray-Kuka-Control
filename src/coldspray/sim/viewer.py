"""3D view of the cell, robot with spray gun, part and toolpath.

`Scene` draws into any PyVista plotter (a plain window or the Qt GUI). The
cell is added once; path, gun and gantry position can be swapped afterwards.
Everything is drawn in WORLD (cell CAD) coordinates.
"""

from dataclasses import dataclass

import numpy as np
import pyvista as pv

from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.sim.cell_model import OPTIONAL, CellMeshes
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
    nozzle: tuple | None = None  # (vertices, faces) of the nozzle in gun-CAD coordinates


def _polydata(v, f) -> pv.PolyData:
    f = np.asarray(f)
    return pv.PolyData(np.asarray(v, float), np.hstack([np.full((len(f), 1), 3), f]).ravel())


def _polyline(points) -> pv.PolyData:
    line = pv.PolyData(np.asarray(points, float))
    line.lines = np.hstack([[len(points)], np.arange(len(points))])
    return line


def _shift(offset) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = offset
    return m


class Scene:
    def __init__(self, pl: pv.Plotter, robot: Robot, world_from_robot=None,
                 cell: CellMeshes | None = None, gantry_offset=(0.0, 0.0, 0.0),
                 cad_pose=HOME_DEG, gun: Gun | None = None):
        self.pl, self.robot = pl, robot
        self.world_from_robot = np.eye(4) if world_from_robot is None else world_from_robot
        self.path_actors: list = []
        self.n_points = 0
        self.joints = np.zeros((0, 6))
        self.reachable = np.zeros(0, bool)

        pl.set_background("white")
        pl.enable_anti_aliasing("ssaa")
        self.link_actors, self.stick, self.carriage = [], None, None
        self.group_actors = {}
        if cell is not None:
            pl.add_mesh(cell.static, color="#c9cfd4", smooth_shading=True)
            for label, mesh in cell.groups.items():
                walls = label == "Booth walls"
                actor = pl.add_mesh(mesh, color="#dfe6ea" if walls else "#c9cfd4",
                                    opacity=0.12 if walls else 1.0, smooth_shading=not walls)
                actor.SetVisibility(OPTIONAL[label][1])
                self.group_actors[label] = actor
            self.carriage = pl.add_mesh(cell.carriage, color="#9fb3c1", smooth_shading=True)
            self.cad_frames = robot.link_frames(cad_pose)
            for k, mesh in enumerate(cell.robot):
                color = "#3c3c3c" if k == 0 else KUKA_ORANGE
                self.link_actors.append(pl.add_mesh(mesh, color=color, smooth_shading=True))
        else:
            pl.add_mesh(pv.Plane(center=self.world_from_robot[:3, 3] * [1, 1, 0], i_size=6000, j_size=6000),
                        color="#eeeeee")
            self.stick = _polyline(np.zeros((7, 3)))
            pl.add_mesh(self.stick, color=KUKA_ORANGE, line_width=14, render_lines_as_tubes=True)

        self.gun_actors: list = []
        self.set_gun(robot, gun)
        self.set_placement(self.world_from_robot, gantry_offset)
        self.spray = _polyline([[0, 0, 0], [0, 0, 1]])
        pl.add_mesh(self.spray, color="#1a73e8", line_width=4)
        self.status = pl.add_text("", position="upper_left", font_size=10, color="black")
        self.summary = pl.add_text("", position="upper_right", font_size=10, color="green")

    # --- configuration --------------------------------------------------------
    def set_placement(self, world_from_robot, gantry_offset=(0.0, 0.0, 0.0)):
        """Robot root in WORLD, and how far the gantry carriage moved from the CAD."""
        self.world_from_robot = world_from_robot
        if self.link_actors:
            self.link_actors[0].user_matrix = world_from_robot
            self.carriage.user_matrix = _shift(gantry_offset)

    def set_gun(self, robot: Robot, gun: Gun | None):
        self.robot = robot
        for a in self.gun_actors:
            self.pl.remove_actor(a)
        self.gun, self.gun_actors = gun, []
        if gun is not None:
            body = self.pl.add_mesh(_polydata(gun.vertices, gun.faces), color="#8d99a6", smooth_shading=True)
            if gun.nozzle is not None:
                nozzle = _polydata(*gun.nozzle)
            else:  # no nozzle CAD: a plain tube of the right length
                nozzle = pv.Cylinder(center=(0, gun.nozzle_length / 2, 0), direction=(0, 1, 0),
                                     radius=14, height=gun.nozzle_length)
            self.gun_actors = [body, self.pl.add_mesh(nozzle, color="#5f6b77", smooth_shading=True)]

    def set_group_visible(self, label: str, visible: bool):
        if label in self.group_actors:
            self.group_actors[label].SetVisibility(visible)

    def show_walls(self, visible: bool):
        self.set_group_visible("Booth walls", visible)

    # --- toolpath -------------------------------------------------------------
    def set_path(self, path: Toolpath, base_in_robot, part=None, plate_size=None, step=5.0, home=None):
        """Draw a toolpath (and optional part mesh in BASE coordinates).
        Returns the joint angles for every sampled point (NaN = unreachable)."""
        for a in self.path_actors:
            self.pl.remove_actor(a)
        self.path_actors = []
        world_from_base = self.world_from_robot @ base_in_robot
        self.world_from_base = world_from_base

        poses, gun_on, _ = path.sample(step)
        self.joints = self.robot.ik_path([base_in_robot @ p for p in poses], seed_deg=HOME_DEG if home is None else home)
        self.reachable = ~np.isnan(self.joints[:, 0])
        self.world = np.array([world_from_base @ p for p in poses])
        self.gun_on, self.path_name, self.n_points = gun_on, path.name, len(poses)
        pts = self.world[:, :3, 3]

        add = self.path_actors.append
        if plate_size is not None:
            w, l = plate_size
            box = pv.Box(bounds=(0, w, 0, l, -10, 0)).transform(world_from_base, inplace=False)
            add(self.pl.add_mesh(box, color="#b0bec5"))
        if part is not None:
            add(self.pl.add_mesh(_polydata(*part).transform(world_from_base, inplace=False),
                                 color="#4f6d7a", opacity=0.4))
        for mask, color, width in ((gun_on, SPRAY, 3), (~gun_on, TRAVEL, 1)):
            segs = [i for i in range(1, len(pts)) if mask[i]]
            if segs:
                lines = np.hstack([[2, i - 1, i] for i in segs])
                add(self.pl.add_mesh(pv.PolyData(pts, lines=lines), color=color, line_width=width))
        bad = int((~self.reachable).sum())
        if bad:
            add(self.pl.add_points(pts[~self.reachable], color=BAD, point_size=10, render_points_as_spheres=True))
        self.summary.SetText(3, f"{bad} unreachable points" if bad else "all points reachable")
        self.summary.GetTextProperty().SetColor(pv.Color(BAD if bad else "green").float_rgb)
        self.show_point(self.first_reachable())
        return self.joints

    def first_reachable(self) -> int:
        return int(np.argmax(self.reachable)) if self.reachable.any() else 0

    def show_point(self, i: int):
        i = int(np.clip(i, 0, max(self.n_points - 1, 0)))
        if not self.n_points:
            return
        if not self.reachable[i]:
            self.status.SetText(2, f"point {i}: UNREACHABLE")
            return
        q = self.joints[i]
        if self.link_actors:
            frames = self.robot.link_frames(q)
            for k in range(1, 7):
                self.link_actors[k].user_matrix = (self.world_from_robot @ frames[k - 1]
                                                   @ np.linalg.inv(self.cad_frames[k - 1]))
        else:
            p = self.robot.link_points(q)
            self.stick.points = (self.world_from_robot @ np.c_[p, np.ones(7)].T).T[:, :3]
        if self.gun_actors:
            m = self.world_from_robot @ self.robot.flange(q) @ self.gun.to_flange
            for a in self.gun_actors:
                a.user_matrix = m
        tcp = self.world[i]
        self.spray.points = np.array([tcp[:3, 3], tcp[:3, 3] + tcp[:3, 0] * 40])
        j = ", ".join(f"{v:.1f}" for v in q)
        self.status.SetText(2, f"{self.path_name}  point {i}/{self.n_points - 1}  "
                               f"spray {'ON' if self.gun_on[i] else 'off'}\nA1..A6: {j}")

    def focus_on_work(self):
        target = self.world_from_base[:3, 3]
        self.pl.camera_position = [target + [-2600, -4200, 2600], target + [600, 300, 700], (0, 0, 1)]


def show(robot: Robot, path: Toolpath, base_in_robot: np.ndarray, world_from_robot=None,
         cell: CellMeshes | None = None, gantry_offset=(0.0, 0.0, 0.0), cad_pose=HOME_DEG,
         gun: Gun | None = None, part=None, plate_size=None, step=5.0, home=None,
         screenshot=None):
    """Standalone window: slider to move along the path, "p" to play,
    checkbox (bottom right) to show the booth walls."""
    pl = pv.Plotter(off_screen=screenshot is not None, window_size=(1400, 900))
    scene = Scene(pl, robot, world_from_robot, cell, gantry_offset, cad_pose, gun)
    joints = scene.set_path(path, base_in_robot, part, plate_size, step, home)

    slider = pl.add_slider_widget(scene.show_point, [0, max(scene.n_points - 1, 1)],
                                  value=scene.first_reachable(), title="Path position",
                                  pointa=(0.25, 0.08), pointb=(0.75, 0.08), fmt="%.0f")

    def play():
        state = {"i": int(slider.GetRepresentation().GetValue())}

        def tick(_step):
            state["i"] = (state["i"] + 1) % scene.n_points
            slider.GetRepresentation().SetValue(state["i"])
            scene.show_point(state["i"])

        pl.add_timer_event(max_steps=scene.n_points, duration=30, callback=tick)

    pl.add_key_event("p", play)
    if cell is not None:
        pl.add_checkbox_button_widget(scene.show_walls, value=False, position=(1330, 20), size=30,
                                      color_on="#9fb3c1")
        pl.add_text("walls", position=(1270, 26), font_size=9, color="#555555")
    pl.add_text("p: play   drag: rotate   scroll: zoom   q: quit", position="lower_left",
                font_size=8, color="#555555")
    scene.focus_on_work()
    if screenshot:
        pl.show(screenshot=screenshot)
    else:
        pl.show()
    return joints
