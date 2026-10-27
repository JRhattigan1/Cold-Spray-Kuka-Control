"""3D view of a part or assembly on its own, for choosing what to build.

Each body is its own actor so it can be hidden (to reach faces behind it,
e.g. the underside of a repair patch against the component) and coloured by
its role: build, fixture, or ignore. Flat faces are picked by clicking them;
only visible bodies can be picked. Drawn in part-CAD coordinates.
"""

import numpy as np
import pyvista as pv
from vtkmodules.vtkRenderingCore import vtkCellPicker

from coldspray.sim.viewer import BUILD_FACE, FIXTURE, PART, VIEW_DIRS, _aim, _bounds_centre, _polydata, _section_plane

BASE_FACE = "#1a73e8"
COAT_FACE = "#f28c28"   # faces chosen for surface coating
IGNORED = "#c9ccd1"
ROLES = ("build", "fixture", "ignore")


class PartView:
    def __init__(self, pl):
        self.pl = pl
        self.part = None
        self.actors: list = []           # one per body
        self.triangles: list = []        # per body: part triangle index of each actor cell
        self.roles: list[str] = []
        self.visible: list[bool] = []
        self.highlights: dict = {}       # "start" / "base" -> actor
        self.highlighted: dict = {}      # "start" / "base" -> face id
        self._pick_cb = self._pick_obs = None
        self._keep = False               # coat-face picking: keep picking after each click
        self.coat_actor, self.coat_faces = None, set()   # the coat faces, highlighted together
        self.frame_actors: list = []     # the KRL base's X / Y / Z arrows
        self.pick_point = None           # part-CAD point of the last pick click
        self.section_plane = None        # vtkPlane while the section is on
        self.section_axis = "x"
        pl.set_background("white")
        pl.enable_anti_aliasing("msaa")
        pl.add_axes()
        self.hint = pl.add_text("", position="upper_left", font_size=10, color="black")

    def set_part(self, part, roles: list[str]):
        self.cancel_pick()
        for a in self.actors + list(self.highlights.values()):
            self.pl.remove_actor(a)
        self.highlight_coat(set())
        self.show_frame(None)
        self.part, self.actors, self.triangles, self.highlights, self.highlighted = part, [], [], {}, {}
        self.roles, self.visible = list(roles), [True] * len(roles)
        for b in range(part.n_bodies):
            tri = np.arange(len(part.faces)) if part.body_ids is None else np.flatnonzero(part.body_ids == b)
            self.triangles.append(tri)
            self.actors.append(self.pl.add_mesh(_polydata(part.vertices, part.faces[tri]), smooth_shading=False))
        self.set_roles(roles)
        if self.section_plane is not None:         # a new part: a new plane through it
            self.set_section(True)
        self.fit()

    def set_roles(self, roles: list[str]):
        self.roles = list(roles)
        for actor, role in zip(self.actors, self.roles):
            actor.prop.color = {"build": PART, "fixture": FIXTURE, "ignore": IGNORED}[role]
            actor.prop.opacity = 0.35 if role == "ignore" else 1.0
        self.pl.render()

    def set_visible(self, body: int, on: bool):
        self.visible[body] = on
        self.actors[body].SetVisibility(on)
        for kind, actor in self.highlights.items():       # a face is hidden with its body
            actor.SetVisibility(self.visible[self.part.body_of_face(self.highlighted[kind])])
        if self.coat_faces:
            self.highlight_coat(self.coat_faces)
        self.pl.render()

    def fit(self):
        """Zoom to the visible bodies."""
        shown = [a for a, v in zip(self.actors, self.visible) if v] or self.actors
        if shown:
            bounds = np.array([a.GetBounds() for a in shown])
            self.pl.reset_camera(bounds=(bounds[:, 0].min(), bounds[:, 1].max(), bounds[:, 2].min(),
                                         bounds[:, 3].max(), bounds[:, 4].min(), bounds[:, 5].max()))
        self.pl.render()

    def _shown_bounds(self):
        shown = [a for a, v in zip(self.actors, self.visible) if v] or self.actors
        return [a.GetBounds() for a in shown]

    def view_preset(self, name: str):
        """Camera preset in part-CAD coordinates: "fit" (keep the direction, zoom
        to the visible bodies), "iso", "top", "front", "side" or "bottom"."""
        if name == "fit":
            self.fit()
            return
        if name not in VIEW_DIRS:
            raise ValueError(f"unknown view {name!r}")
        bounds = self._shown_bounds()
        _aim(self.pl, name, _bounds_centre(bounds) if bounds else (0, 0, 0))
        self.fit()

    def set_section(self, on: bool, axis: str | None = None):
        """Clip the bodies (and face highlights) with a plane through the middle of
        the visible bodies across `axis` ("x", "y" or "z"; default: the last one),
        keeping the half away from the camera."""
        if axis is not None:
            if axis not in "xyz" or len(axis) != 1:
                raise ValueError(f"axis must be x, y or z, not {axis!r}")
            self.section_axis = axis
        for a in self._clipped():
            a.mapper.RemoveAllClippingPlanes()
        self.section_plane = None
        bounds = self._shown_bounds()
        if on and bounds:
            self.section_plane = _section_plane(_bounds_centre(bounds), self.section_axis,
                                                self.pl.camera.position)
            self._apply_section()
        self.pl.render()

    def _clipped(self) -> list:
        return self.actors + list(self.highlights.values()) + ([self.coat_actor] if self.coat_actor is not None else [])

    def _apply_section(self):
        if self.section_plane is None:
            return
        for a in self._clipped():
            planes = a.mapper.GetClippingPlanes()
            if planes is None or not any(planes.GetItemAsObject(k) is self.section_plane
                                         for k in range(planes.GetNumberOfItems())):
                a.mapper.AddClippingPlane(self.section_plane)

    def highlight(self, kind: str, face_id: int | None):
        """Show face `face_id` as the start face (green) or base face (blue); None clears it."""
        old = self.highlights.pop(kind, None)
        if old is not None:
            self.pl.remove_actor(old)
        if face_id is not None and self.part is not None:
            faces = self.part.faces[self.part.face_ids == face_id]
            actor = self.pl.add_mesh(_polydata(self.part.vertices, faces),
                                     color=BUILD_FACE if kind == "start" else BASE_FACE)
            actor.mapper.SetResolveCoincidentTopologyToPolygonOffset()
            actor.mapper.SetRelativeCoincidentTopologyPolygonOffsetParameters(-2, -20)
            actor.SetPickable(False)
            self.highlighted[kind] = face_id
            actor.SetVisibility(self.visible[self.part.body_of_face(face_id)])
            self.highlights[kind] = actor
            self._apply_section()
        self.pl.render()

    def highlight_coat(self, faces: set[int]):
        """Show the coat faces (orange) on the visible bodies; an empty set clears them."""
        if self.coat_actor is not None:
            self.pl.remove_actor(self.coat_actor)
        self.coat_actor, self.coat_faces = None, set(faces)
        if faces and self.part is not None:
            tri = np.isin(self.part.face_ids, list(faces))
            if self.part.body_ids is not None:
                tri &= np.isin(self.part.body_ids, [b for b, v in enumerate(self.visible) if v])
            if tri.any():
                actor = self.coat_actor = self.pl.add_mesh(_polydata(self.part.vertices, self.part.faces[tri]),
                                                           color=COAT_FACE)
                actor.mapper.SetResolveCoincidentTopologyToPolygonOffset()
                actor.mapper.SetRelativeCoincidentTopologyPolygonOffsetParameters(-1, -10)
                actor.SetPickable(False)
                self._apply_section()
        self.pl.render()

    def show_frame(self, t: np.ndarray | None):
        """Draw the KRL base `t` (part-CAD <- base) as red X, green Y and blue Z arrows; None clears it."""
        for a in self.frame_actors:
            self.pl.remove_actor(a)
        self.frame_actors = []
        if t is not None and self.part is not None:
            size = 0.15 * float(np.ptp(self.part.vertices, axis=0).max())
            for k, color in enumerate(("red", "green", "blue")):
                arrow = pv.Arrow(t[:3, 3], t[:3, k], scale=size, tip_radius=0.08, shaft_radius=0.03)
                actor = self.pl.add_mesh(arrow, color=color)
                actor.SetPickable(False)
                self.frame_actors.append(actor)
        self.pl.render()

    # --- picking ----------------------------------------------------------------
    def start_pick(self, callback, hint="", keep=False) -> bool:
        """The next left click on a visible body calls `callback(face_id, body)`
        once (every click until `cancel_pick` with `keep`); clicks beside the
        part still turn the view."""
        self.cancel_pick()
        self._keep = keep
        if not self.actors or getattr(self.pl, "iren", None) is None:
            return False
        self._pick_cb = callback
        self._pick_obs = self.pl.iren.add_observer("LeftButtonPressEvent", self._click)
        self.hint.SetText(2, hint)
        self.pl.render()
        return True

    def cancel_pick(self):
        if self._pick_obs is not None:
            self.pl.iren.remove_observer(self._pick_obs)
        self._pick_cb = self._pick_obs = None
        self._keep = False
        self.hint.SetText(2, "")

    def _click(self, iren, _event):
        x, y = iren.GetEventPosition()
        picker = vtkCellPicker()
        picker.SetTolerance(0.0005)
        picker.PickFromListOn()
        for actor, shown, role in zip(self.actors, self.visible, self.roles):
            if shown and role != "ignore":
                picker.AddPickList(actor)
        if picker.Pick(x, y, 0, self.pl.renderer) and picker.GetCellId() >= 0:
            body = next(b for b, a in enumerate(self.actors) if a is picker.GetActor())
            self.pick_triangle(int(self.triangles[body][picker.GetCellId()]), picker.GetPickPosition())

    def pick_triangle(self, tri: int, point=None):
        """Report the CAD face and body of part triangle `tri`; stop picking
        unless it was started with `keep`. `point` (part-CAD, where the click hit)
        is kept in `pick_point`; by default the triangle's centre."""
        self.pick_point = np.asarray(point if point is not None else
                                     self.part.vertices[self.part.faces[tri]].mean(0), float)
        callback = self._pick_cb
        if not self._keep:
            self.cancel_pick()
        if callback is not None:
            body = 0 if self.part.body_ids is None else int(self.part.body_ids[tri])
            callback(int(self.part.face_ids[tri]), body)
