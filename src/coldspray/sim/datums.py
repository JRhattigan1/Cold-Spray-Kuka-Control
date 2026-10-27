"""Datum planes XY / XZ / YZ through the faceplate centre (the BASE origin).

They ride on the turntable like BASE, each with an offset along its normal,
and are named "datum:XY" etc. so a pick can hand them to the constraint
solver as a reference plane. `datum_plane` is pure (no Qt / VTK).
"""

import numpy as np
import pyvista as pv

NORMALS = {"XY": (0.0, 0.0, 1.0), "XZ": (0.0, 1.0, 0.0), "YZ": (1.0, 0.0, 0.0)}
COLOURS = {"XY": "#4f8fd6", "XZ": "#5cb85c", "YZ": "#d9534f"}
PREFIX = "datum:"
# in-plane axes (u = text right, v = text up) for each plane's label
LABEL_AXES = {"XY": ((1, 0, 0), (0, 1, 0)), "XZ": ((1, 0, 0), (0, 0, 1)), "YZ": ((0, 1, 0), (0, 0, 1))}
LABEL_FRACTION = 0.06                     # label height as a fraction of plane size


def _key(name: str) -> str:
    key = name[len(PREFIX):] if name.startswith(PREFIX) else name
    if key not in NORMALS:
        raise KeyError(f"unknown datum plane {name!r}")
    return key


def datum_plane(name: str, offset: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """(point, unit normal) of datum `name` ("XZ" or "datum:XZ") in BASE."""
    n = np.array(NORMALS[_key(name)])
    return n * offset, n


def datum_matrix(name: str, offset: float = 0.0) -> np.ndarray:
    """4x4 placing a plane drawn through the BASE origin at its offset."""
    m = np.eye(4)
    m[:3, 3] = datum_plane(name, offset)[0]
    return m


def label_matrix(name: str, size: float) -> np.ndarray:
    """4x4 placing a unit-height text (drawn in XY, centred) in the plane's
    corner (+u, +v), lying in the plane, in the plane's local frame."""
    u, v = (np.array(a, float) for a in LABEL_AXES[_key(name)])
    h = LABEL_FRACTION * size
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2] = u * h, v * h, np.cross(u, v) * h
    m[:3, 3] = (u + v) * (0.5 * size - 1.5 * h)
    return m


class Datums:
    """The three translucent planes in a plotter, hidden until toggled on."""

    def __init__(self, pl: pv.Plotter, size: float = 600.0):
        self.offsets = {k: 0.0 for k in NORMALS}
        self.frame = np.eye(4)            # BASE in WORLD
        self.size = float(size)
        self.actors = {}
        self.labels = {}                  # text actors, toggled with their plane, never pickable
        for k, n in NORMALS.items():
            a = pl.add_mesh(self._plane_mesh(k), color=COLOURS[k], opacity=0.18,
                            show_edges=True, edge_color=COLOURS[k])
            a.SetVisibility(False)
            a.SetPickable(False)
            self.actors[k] = a
            t = pl.add_mesh(pv.Text3D(k, depth=0.0), color=COLOURS[k])
            t.SetVisibility(False)
            t.SetPickable(False)
            self.labels[k] = t
        self._place()

    def _plane_mesh(self, k: str) -> pv.PolyData:
        return pv.Plane(center=(0, 0, 0), direction=NORMALS[k], i_size=self.size, j_size=self.size)

    def _place(self):
        for k, a in self.actors.items():
            m = self.frame @ datum_matrix(k, self.offsets[k])
            a.user_matrix = m
            self.labels[k].user_matrix = m @ label_matrix(k, self.size)

    def set_size(self, size: float):
        """Edge length (mm) of each square plane, e.g. sized to the faceplate."""
        self.size = float(size)
        for k, a in self.actors.items():
            a.mapper.dataset.copy_from(self._plane_mesh(k))
        self._place()

    def set_frame(self, base_world: np.ndarray):
        """Follow the table: `base_world` is BASE in WORLD (cell.base_world)."""
        self.frame = np.asarray(base_world, float)
        self._place()

    def set_offset(self, name: str, offset: float):
        self.offsets[_key(name)] = float(offset)
        self._place()

    def set_visible(self, name: str, on: bool):
        """Show / hide a plane; visible planes are pickable."""
        a = self.actors[_key(name)]
        a.SetVisibility(on)
        a.SetPickable(on)
        self.labels[_key(name)].SetVisibility(on)

    def visible(self, name: str) -> bool:
        return bool(self.actors[_key(name)].GetVisibility())

    def plane(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        """(point, normal) in BASE, with its offset."""
        return datum_plane(name, self.offsets[_key(name)])

    def name_of(self, actor) -> str | None:
        """"datum:XZ" for a datum actor, else None."""
        for k, a in self.actors.items():
            if a is actor:
                return PREFIX + k
        return None
