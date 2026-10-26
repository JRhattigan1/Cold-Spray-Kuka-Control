"""3D view of the cell, robot with spray gun, turntable, part and toolpath.

`Scene` draws into any PyVista plotter (a plain window or the Qt GUI). The
cell is added once; path, gun and gantry position can be swapped afterwards.
Everything is drawn in WORLD (cell CAD) coordinates; the part, substrate and
path are drawn in BASE coordinates and ride on the turntable.

Two modes:
  setup   - table at a chosen tilt / rotation, part and substrate with drag
            handles (enter_setup); no path.
  preview - a planned path (set_path); part and substrate ride on the table
            from sample to sample.
"""

from dataclasses import dataclass

import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation
from vtkmodules.vtkRenderingCore import vtkCellPicker

from coldspray.cell import base_world, turntable_motion
from coldspray.kinematics.robot import HOME_DEG, Robot
from coldspray.models import CellConfig
from coldspray.planner.position import gun_down_deg
from coldspray.sim.cell_model import OPTIONAL, CellMeshes
from coldspray.toolpath.model import Toolpath

SPRAY = "#f28c28"
TRAVEL = "#9aa0a6"
BAD = "#d93025"
CRASH = "#c2185b"
KUKA_ORANGE = "#ff6a13"
TABLE = "#a7b4bd"
PART = "#4f6d7a"
SUBSTRATE = "#b0bec5"
BUILD_FACE = "#2bb673"
OUT_OF_DATE = "#e37400"
STEEP = "#f2b600"          # gun pointing further down than allowed


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


def plate_box(plate_size) -> tuple[np.ndarray, np.ndarray]:
    """Old-style plate (width, length) under BASE x, y > 0, as (vertices, faces)."""
    w, l = plate_size
    box = pv.Box(bounds=(0, w, 0, l, -10, 0)).triangulate()
    return np.asarray(box.points), box.faces.reshape(-1, 4)[:, 1:]


class Scene:
    def __init__(self, pl: pv.Plotter, robot: Robot, world_from_robot=None,
                 cell: CellMeshes | None = None, gantry_offset=(0.0, 0.0, 0.0),
                 cad_pose=HOME_DEG, gun: Gun | None = None):
        self.pl, self.robot = pl, robot
        self.world_from_robot = np.eye(4) if world_from_robot is None else world_from_robot
        self.path_actors: list = []      # move with the faceplate
        self.crash_actor = None
        self.n_points = 0
        self.joints = np.zeros((0, 6))
        self.reachable = np.zeros(0, bool)
        self.collisions: dict[int, set] = {}
        self.gun_down = np.zeros(0)       # spray direction below horizontal (deg) per sample
        self.steep = np.zeros(0, bool)    # ... beyond the allowed limit
        self.cell_cfg: CellConfig | None = None
        self.mode = "preview"

        # setup mode: part (drawn in part-CAD coordinates), build face, substrate, drag handles
        self.part = None
        self.part_actor = self.face_actor = self.substrate_actor = None
        self.setup_frame = np.eye(4)     # BASE in WORLD with the table at the setup axes
        self.widget = None
        self._widget_on = False
        self._constrained = False
        self.on_part_moved = None        # callback(placement, final) while / after dragging
        self._pick_cb = self._pick_obs = None

        pl.set_background("white")
        pl.enable_anti_aliasing("msaa")   # not ssaa: its render pass breaks the hardware picker the drag handles use
        self.link_actors, self.stick, self.carriage = [], None, None
        self.group_actors = {}
        self.tilting_actors, self.rotating_actors = [], []
        if cell is not None:
            pl.add_mesh(cell.static, color="#c9cfd4", smooth_shading=True)
            for label, mesh in cell.groups.items():
                walls = label == "Booth walls"
                actor = pl.add_mesh(mesh, color="#dfe6ea" if walls else "#c9cfd4",
                                    opacity=0.12 if walls else 1.0, smooth_shading=not walls)
                actor.SetVisibility(OPTIONAL[label][1])
                self.group_actors[label] = actor
                if OPTIONAL[label][2] == "rotating":
                    self.rotating_actors.append(actor)
            if cell.tilting.n_cells:
                self.tilting_actors.append(pl.add_mesh(cell.tilting, color=TABLE, smooth_shading=True))
            if cell.rotating.n_cells:
                self.rotating_actors.append(pl.add_mesh(cell.rotating, color="#8fa3b0", smooth_shading=True))
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
        self.spray_actor = pl.add_mesh(self.spray, color="#1a73e8", line_width=4)
        # only the part and the drag handles are pickable (keeps hover picking fast)
        for a in pl.renderer.actors.values():
            a.SetPickable(False)
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
            for a in self.gun_actors:
                a.SetPickable(False)

    def set_group_visible(self, label: str, visible: bool):
        if label in self.group_actors:
            self.group_actors[label].SetVisibility(visible)

    def show_walls(self, visible: bool):
        self.set_group_visible("Booth walls", visible)

    def set_robot_pose(self, q):
        """Draw the robot and gun at joint angles `q` (degrees)."""
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

    def _set_table(self, axes):
        tilt, face = turntable_motion(self.cell_cfg, axes)
        for a in self.tilting_actors:
            a.user_matrix = tilt
        for a in self.rotating_actors:
            a.user_matrix = face

    # --- part setup -----------------------------------------------------------
    def set_part(self, part):
        """Part to set up (a coldspray.cad.part.Part, or None): drawn in part-CAD
        coordinates at part.placement, with drag handles in setup mode."""
        self.cancel_face_pick()
        self._remove_widget()
        for a in (self.part_actor, self.face_actor):
            if a is not None:
                self.pl.remove_actor(a)
        self.part, self.part_actor, self.face_actor = part, None, None
        if part is not None:
            self.part_actor = self.pl.add_mesh(_polydata(part.vertices, part.faces), color=PART)
            self.part_actor.SetVisibility(self.mode == "setup")
            self.set_part_placement()
            if self.mode == "setup":
                self._make_widget()

    def set_substrate(self, mesh):
        """Substrate plate (vertices, faces) in BASE, or None."""
        if self.substrate_actor is not None:
            self.pl.remove_actor(self.substrate_actor)
            self.substrate_actor = None
        if mesh is not None:
            self.substrate_actor = self.pl.add_mesh(_polydata(*mesh), color=SUBSTRATE)
            self.substrate_actor.SetPickable(False)
            self.substrate_actor.user_matrix = self.setup_frame
            self.substrate_actor.SetVisibility(self.mode == "setup")

    def set_part_placement(self):
        """Redraw the part (and handles) after part.placement changed."""
        if self.part_actor is None:
            return
        m = self.setup_frame @ self.part.placement
        for a in (self.part_actor, self.face_actor):
            if a is not None:
                a.user_matrix = m
        if self.widget is not None:
            self.widget._cached_matrix = m            # the widget starts its next drag from here
            self.widget.origin = self._part_centre()

    def _part_centre(self) -> np.ndarray:
        v = self.part.vertices
        c = (v.min(axis=0) + v.max(axis=0)) / 2
        m = self.setup_frame @ self.part.placement
        return m[:3, :3] @ c + m[:3, 3]

    def highlight_face(self, face_id: int | None):
        """Colour the triangles of CAD face `face_id` (None: no highlight)."""
        if self.face_actor is not None:
            self.pl.remove_actor(self.face_actor)
            self.face_actor = None
        if face_id is None or self.part is None:
            return
        faces = self.part.faces[self.part.face_ids == face_id]
        self.face_actor = self.pl.add_mesh(_polydata(self.part.vertices, faces), color=BUILD_FACE)
        self.face_actor.mapper.SetResolveCoincidentTopologyToPolygonOffset()
        self.face_actor.mapper.SetRelativeCoincidentTopologyPolygonOffsetParameters(-2, -20)
        self.face_actor.SetPickable(False)
        self.face_actor.SetVisibility(self.mode == "setup")
        self.face_actor.user_matrix = self.setup_frame @ self.part.placement

    def set_constrained(self, on: bool):
        """Build face on the plate: only offer the XY arrows and the yaw ring."""
        self._constrained = on
        self._show_widget(self.mode == "setup")

    def enter_setup(self, cell: CellConfig, axes=None, home=None):
        """Setup mode: no path, table at `axes`, part and substrate on the faceplate
        with drag handles, robot at `home`."""
        self.cancel_face_pick()
        self._clear_path()
        self.cell_cfg = cell
        self.mode = "setup"
        frame = base_world(cell, axes)
        moved = not np.allclose(frame, self.setup_frame)
        self.setup_frame = frame
        self._set_table(axes)
        if home is not None:
            self.set_robot_pose(home)
        if self.substrate_actor is not None:
            self.substrate_actor.user_matrix = frame
        for a in (self.part_actor, self.face_actor, self.substrate_actor):
            if a is not None:
                a.SetVisibility(True)
        self.spray_actor.SetVisibility(False)
        self.set_part_placement()
        if self.part_actor is not None and (self.widget is None or moved):
            self._make_widget()
        self._show_widget(True)
        table = "  ".join(f"{k} {v:.1f}" for k, v in (axes or {}).items())
        self.status.SetText(2, f"SETUP   {table}\nDrag the handles or type a placement, then Generate")
        self.summary.SetText(3, "toolpath out of date")
        self.summary.GetTextProperty().SetColor(pv.Color(OUT_OF_DATE).float_rgb)

    # --- drag handles ---------------------------------------------------------
    def _make_widget(self):
        """Affine widget on the part, aligned with the BASE axes, pivoting about the part centre."""
        self._remove_widget()
        if self.part_actor is None or getattr(self.pl, "iren", None) is None:
            return
        size = self.part_actor.GetLength()
        self.widget = self.pl.add_affine_transform_widget(
            self.part_actor, origin=tuple(self._part_centre()), start=False,
            scale=max(0.45, 70.0 / size), axes=self.setup_frame[:3, :3].T,
            interact_callback=lambda m: self._dragged(m, False),
            release_callback=lambda m: self._dragged(m, True))
        self._widget_on = False
        self._show_widget(self.mode == "setup")

    def _remove_widget(self):
        if self.widget is not None:
            if not self._widget_on:
                self.widget.enable()    # remove() disables, which needs it enabled
            self.widget.remove()
        self.widget, self._widget_on = None, False

    def _show_widget(self, on: bool):
        w = self.widget
        if w is None:
            return
        if on != self._widget_on:
            (w.enable if on else w.disable)()
            self._widget_on = on
        locked = [w._circles[0], w._circles[1], w._arrows[2]] if self._constrained else []
        for a in w._arrows + w._circles:
            visible = on and not any(a is b for b in locked)
            a.SetVisibility(visible)
            a.SetPickable(visible)

    def _dragged(self, world_matrix, final: bool):
        if self.part is None or self.mode != "setup" or self.on_part_moved is None:
            return
        placement = np.linalg.inv(self.setup_frame) @ np.asarray(world_matrix)
        if final and np.allclose(placement, self.part.placement):
            return                       # a click (e.g. turning the view), not a drag
        self.on_part_moved(placement, final)

    # --- face picking ---------------------------------------------------------
    def start_face_pick(self, callback) -> bool:
        """The next left click on the part calls `callback(face_id)` once; clicks
        beside the part still turn the view. False if there is nothing to pick."""
        self.cancel_face_pick()
        if self.part_actor is None or getattr(self.pl, "iren", None) is None:
            return False
        self._pick_cb = callback
        self._show_widget(False)
        self._pick_obs = self.pl.iren.add_observer("LeftButtonPressEvent", self._pick_click)
        return True

    def cancel_face_pick(self):
        if self._pick_obs is not None:
            self.pl.iren.remove_observer(self._pick_obs)
            self._show_widget(self.mode == "setup")
        self._pick_cb = self._pick_obs = None

    def _pick_click(self, iren, _event):
        x, y = iren.GetEventPosition()
        picker = vtkCellPicker()
        picker.SetTolerance(0.0005)
        picker.PickFromListOn()
        picker.AddPickList(self.part_actor)
        if picker.Pick(x, y, 0, self.pl.renderer) and picker.GetCellId() >= 0:
            self.pick_triangle(picker.GetCellId())

    def pick_triangle(self, tri: int):
        """Report the CAD face of part triangle `tri` to the pick callback and stop picking."""
        callback = self._pick_cb
        self.cancel_face_pick()
        if callback is not None and self.part is not None:
            callback(int(self.part.face_ids[tri]))

    # --- toolpath -------------------------------------------------------------
    def _clear_path(self):
        for a in self.path_actors:
            self.pl.remove_actor(a)
        self.path_actors, self.crash_actor = [], None
        self.n_points = 0
        self.collisions = {}

    def set_path(self, path: Toolpath, cell: CellConfig, part=None, plate_size=None, step=5.0, home=None,
                 angle_step=2.0, substrate=None, gun_limit_deg=None):
        """Preview mode: draw a toolpath, with optional part and substrate meshes
        (vertices, faces) in BASE coordinates (`plate_size` (w, l): an old-style
        plate when no substrate is given). Returns the joint angles for every
        sampled point (NaN = unreachable). Points where the gun points more than
        `gun_limit_deg` below horizontal are marked."""
        self.cancel_face_pick()
        self._clear_path()
        self.mode = "preview"
        self._show_widget(False)
        for a in (self.part_actor, self.face_actor, self.substrate_actor):
            if a is not None:
                a.SetVisibility(False)
        self.spray_actor.SetVisibility(True)
        self.cell_cfg = cell

        s = path.sample(step, angle_step)
        self.samples = s
        robot_from_world = np.linalg.inv(self.world_from_robot)
        self.base_frames = np.array([base_world(cell, s.axes_at(i)) for i in range(len(s))])
        targets = [robot_from_world @ b @ p for b, p in zip(self.base_frames, s.poses)]
        self.joints = self.robot.ik_path(targets, seed_deg=HOME_DEG if home is None else home)
        self.reachable = ~np.isnan(self.joints[:, 0])
        self.gun_down = gun_down_deg(self.base_frames @ s.poses)
        self.steep = (self.gun_down > gun_limit_deg + 1e-6) if gun_limit_deg is not None             else np.zeros(len(s), bool)
        self.gun_on, self.path_name, self.n_points = s.gun_on, path.name, len(s)
        pts = s.poses[:, :3, 3]           # BASE coordinates

        add = self.path_actors.append
        if substrate is None and plate_size is not None:
            substrate = plate_box(plate_size)
        if substrate is not None:
            add(self.pl.add_mesh(_polydata(*substrate), color=SUBSTRATE))
        if part is not None:
            add(self.pl.add_mesh(_polydata(*part), color=PART, opacity=0.4))
        for mask, color, width in ((s.gun_on, SPRAY, 3), (~s.gun_on, TRAVEL, 1)):
            segs = [i for i in range(1, len(pts)) if mask[i]]
            if segs:
                lines = np.hstack([[2, i - 1, i] for i in segs])
                add(self.pl.add_mesh(pv.PolyData(pts, lines=lines), color=color, line_width=width))
        if self.steep.any():
            add(self.pl.add_points(pts[self.steep], color=STEEP, point_size=7, render_points_as_spheres=True))
        if (~self.reachable).any():
            add(self.pl.add_points(pts[~self.reachable], color=BAD, point_size=10, render_points_as_spheres=True))
        for a in self.path_actors:
            a.SetPickable(False)
        self._update_summary()
        self.show_point(self.first_reachable())
        return self.joints

    def set_collisions(self, hits):
        """Mark samples where the robot or gun collides (list of Collision)."""
        if self.mode != "preview":
            return
        self.collisions = {h.sample: h.pairs for h in hits}
        if self.crash_actor is not None:
            self.pl.remove_actor(self.crash_actor)
            self.path_actors.remove(self.crash_actor)
            self.crash_actor = None
        if hits:
            pts = self.samples.poses[[h.sample for h in hits], :3, 3]
            self.crash_actor = self.pl.add_points(pts, color=CRASH, point_size=12, render_points_as_spheres=True)
            self.crash_actor.SetPickable(False)
            self.path_actors.append(self.crash_actor)
        self._update_summary()
        self.show_point(self.current if hasattr(self, "current") else 0)

    def _update_summary(self):
        bad = int((~self.reachable).sum())
        crashes = len(self.collisions)
        parts = []
        if bad:
            parts.append(f"{bad} unreachable points")
        if crashes:
            parts.append(f"{crashes} collision points")
        if self.steep.any():
            parts.append(f"{int(self.steep.sum())} points gun too steep (down {self.gun_down.max():.0f}°)")
        self.summary.SetText(3, "\n".join(parts) if parts else "all points reachable")
        self.summary.GetTextProperty().SetColor(pv.Color(BAD if parts else "green").float_rgb)

    def first_reachable(self) -> int:
        return int(np.argmax(self.reachable)) if self.reachable.any() else 0

    def show_point(self, i: int):
        if not self.n_points:
            return
        i = int(np.clip(i, 0, self.n_points - 1))
        self.current = i
        axes = self.samples.axes_at(i)
        self._set_table(axes)
        for a in self.path_actors:
            a.user_matrix = self.base_frames[i]
        table = "  ".join(f"{k} {v:.1f}" for k, v in axes.items())
        if not self.reachable[i]:
            self.status.SetText(2, f"point {i}: UNREACHABLE   {table}")
            return
        q = self.joints[i]
        self.set_robot_pose(q)
        tcp = self.base_frames[i] @ self.samples.poses[i]
        self.spray.points = np.array([tcp[:3, 3], tcp[:3, 3] + tcp[:3, 0] * 40])
        j = ", ".join(f"{v:.1f}" for v in q)
        crash = self.collisions.get(i)
        crash_text = ("\nCOLLISION: " + ", ".join(f"{a} / {b}" for a, b in sorted(crash))) if crash else ""
        self.status.SetText(2, f"{self.path_name}  point {i}/{self.n_points - 1}  "
                               f"spray {'ON' if self.gun_on[i] else 'off'}   gun {self.gun_down[i]:.0f}° down"
                               f"{'  TOO STEEP' if self.steep[i] else ''}\nA1..A6: {j}   {table}{crash_text}")

    def focus_on_work(self):
        target = base_world(self.cell_cfg)[:3, 3] if self.cell_cfg else self.world_from_robot[:3, 3]
        self.pl.camera_position = [target + [-2600, -4200, 2600], target + [600, 300, 700], (0, 0, 1)]

    def camera_axes(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(right, forward, up) unit vectors of the view, in WORLD."""
        cam = self.pl.camera
        fwd = np.subtract(cam.focal_point, cam.position)
        fwd /= np.linalg.norm(fwd)
        right = np.cross(fwd, cam.up)
        right /= np.linalg.norm(right)
        return right, fwd, np.cross(right, fwd)

    def fly(self, t, r, dt: float):
        """SpaceMouse camera move (object mode: the scene goes the way the cap is
        pushed). t = (right, away, up), r = (tilt away, tilt right, twist
        anticlockwise), each -1..1; the horizon stays level."""
        cam = self.pl.camera
        pos, foc = np.array(cam.position), np.array(cam.focal_point)
        dist = np.linalg.norm(foc - pos)
        right, fwd, up = self.camera_axes()
        shift = -(t[0] * right + t[2] * up) * dist * 0.8 * dt
        pos += shift - fwd * t[1] * dist * 0.7 * dt         # push away: scene moves away
        foc += shift
        rate = np.radians(90.0) * dt
        turn = Rotation.from_rotvec(-r[2] * rate * np.array([0.0, 0.0, 1.0]))   # about the vertical
        tip = Rotation.from_rotvec(r[0] * rate * right)
        tipped = foc + (turn * tip).apply(pos - foc)
        if abs((foc - tipped)[2]) < 0.995 * dist:          # don't go over the top
            pos = tipped
        else:
            pos = foc + turn.apply(pos - foc)
        cam.position, cam.focal_point, cam.up = pos, foc, (0.0, 0.0, 1.0)
        self.pl.reset_camera_clipping_range()

    def focus_on_part(self, distance=900.0):
        """Close-up of the faceplate for setting up the part: down the faceplate
        normal, from the side away from the robot (so the arm is behind the part)."""
        normal = self.setup_frame[:3, 2]
        target = self.setup_frame[:3, 3] + normal * 60
        away = target - self.world_from_robot[:3, 3]
        away -= np.dot(away, normal) * normal
        away = away / np.linalg.norm(away) if np.linalg.norm(away) > 1e-6 else np.cross(normal, [0, 1, 0])
        view = normal * 1.2 + away
        self.pl.camera_position = [target + view / np.linalg.norm(view) * distance, target, normal]


def show(robot: Robot, path: Toolpath, cell_cfg: CellConfig, world_from_robot=None,
         cell: CellMeshes | None = None, gantry_offset=(0.0, 0.0, 0.0), cad_pose=HOME_DEG,
         gun: Gun | None = None, part=None, plate_size=None, step=5.0, home=None,
         screenshot=None, collisions=False, substrate=None):
    """Standalone window: slider to move along the path, "p" to play,
    checkbox (bottom right) to show the booth walls. `part` and `substrate`
    are (vertices, faces) in BASE."""
    pl = pv.Plotter(off_screen=screenshot is not None, window_size=(1400, 900))
    scene = Scene(pl, robot, world_from_robot, cell, gantry_offset, cad_pose, gun)
    joints = scene.set_path(path, cell_cfg, part, plate_size, step, home, substrate=substrate)
    if collisions and cell is not None:
        from coldspray.sim.collision import CollisionChecker
        checker = CollisionChecker(robot, cell_cfg, cell, gun, part, cad_pose, substrate=substrate)
        scene.set_collisions(checker.check_path(joints, scene.samples.axes_at, every=3))

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
