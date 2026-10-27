"""Markers drawn at a fixed WORLD pose in the cell view: the ghost gun at the
warm-up / powder purge point.

The marker is built once in its own frame (tool X = spray direction, nozzle exit
at the origin) and moved by its actors' user matrix, so `set_pose` is cheap.
"""

import numpy as np
import pyvista as pv

from coldspray.toolpath.model import spray_frame

GHOST = "#7e57c2"
ACCENT = "#ff8a1f"   # orange accent (never blue)

# spray direction presets for the purge point, WORLD; "down-angled" is 30° below horizontal into -Y
DIRECTION_PRESETS = {
    "+X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
    "Down-angled (-Y, 30° down)": (0.0, -np.cos(np.radians(30)), -np.sin(np.radians(30))),
}


def preset_for(spray_dir) -> str:
    """Name of the preset matching `spray_dir`, or "Custom"."""
    d = np.asarray(spray_dir, float)
    d = d / np.linalg.norm(d)
    for name, v in DIRECTION_PRESETS.items():
        if np.allclose(d, v, atol=1e-6):
            return name
    return "Custom"


def marker_pose(point, spray_dir) -> np.ndarray:
    """4x4 WORLD pose of a gun whose nozzle exit is at `point`, spraying along
    `spray_dir` (tool X). Tool Y is horizontal where possible."""
    d = np.asarray(spray_dir, float)
    side = np.cross((0.0, 0.0, 1.0), d)
    if np.linalg.norm(side) < 1e-6:            # spraying straight up / down
        side = np.array([0.0, 1.0, 0.0])
    return spray_frame(point, d, side)


class GunMarker:
    """Semi-transparent gun (body behind a nozzle) plus a spray arrow, at a WORLD pose."""

    def __init__(self, pl: pv.Plotter, length=250.0, color=GHOST):
        nozzle = pv.Cylinder(center=(-length * 0.25, 0, 0), direction=(1, 0, 0), radius=12, height=length * 0.5)
        body = pv.Cylinder(center=(-length * 0.75, 0, 0), direction=(1, 0, 0), radius=45, height=length * 0.5)
        arrow = pv.Arrow(start=(0, 0, 0), direction=(1, 0, 0), scale=length * 0.6)
        glass = dict(opacity=0.35, smooth_shading=True, specular=0.9, specular_power=40, ambient=0.25)
        self.actors = [pl.add_mesh(m, color=color, **glass) for m in (body, nozzle)]
        # orange silhouette: slightly inflated back-face shell gives a rim on the glass
        self.actors += [pl.add_mesh(m, color=ACCENT, opacity=0.55, style="wireframe", line_width=1.0)
                        for m in (body.extract_feature_edges(), nozzle.extract_feature_edges())]
        self.actors.append(pl.add_mesh(arrow, color=ACCENT, opacity=0.8, smooth_shading=True,
                                       specular=0.9, specular_power=40))
        self.pose = np.eye(4)

    def set_pose(self, pose):
        self.pose = np.asarray(pose, float)
        for a in self.actors:
            a.user_matrix = self.pose

    def set_visible(self, on: bool):
        for a in self.actors:
            a.SetVisibility(bool(on))

    @property
    def visible(self) -> bool:
        return bool(self.actors[0].GetVisibility())

    def set_color(self, color):
        """Tint the gun (e.g. red when the robot can't reach the pose)."""
        for a in self.actors[:2]:   # glass bodies only; the orange rim stays
            a.prop.color = color
