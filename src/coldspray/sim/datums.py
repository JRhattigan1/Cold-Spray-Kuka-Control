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


class Datums:
    """The three translucent planes in a plotter, hidden until toggled on."""

    def __init__(self, pl: pv.Plotter, size: float = 600.0):
        self.offsets = {k: 0.0 for k in NORMALS}
        self.frame = np.eye(4)            # BASE in WORLD
        self.actors = {}
        for k, n in NORMALS.items():
            a = pl.add_mesh(pv.Plane(center=(0, 0, 0), direction=n, i_size=size, j_size=size),
                            color=COLOURS[k], opacity=0.18, show_edges=True, edge_color=COLOURS[k])
            a.SetVisibility(False)
            a.SetPickable(False)
            self.actors[k] = a
        self._place()

    def _place(self):
        for k, a in self.actors.items():
            a.user_matrix = self.frame @ datum_matrix(k, self.offsets[k])

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
