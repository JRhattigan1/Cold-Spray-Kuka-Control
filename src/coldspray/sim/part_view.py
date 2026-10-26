"""3D view of a part or assembly on its own, for choosing what to build.

Each body is its own actor so it can be hidden (to reach faces behind it,
e.g. the underside of a repair patch against the component) and coloured by
its role: build, fixture, or ignore. Flat faces are picked by clicking them;
only visible bodies can be picked. Drawn in part-CAD coordinates.
"""

import numpy as np
from vtkmodules.vtkRenderingCore import vtkCellPicker

from coldspray.sim.viewer import BUILD_FACE, FIXTURE, PART, _polydata

BASE_FACE = "#1a73e8"
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
        pl.set_background("white")
        pl.enable_anti_aliasing("msaa")
        pl.add_axes()
        self.hint = pl.add_text("", position="upper_left", font_size=10, color="black")

    def set_part(self, part, roles: list[str]):
        self.cancel_pick()
        for a in self.actors + list(self.highlights.values()):
            self.pl.remove_actor(a)
        self.part, self.actors, self.triangles, self.highlights, self.highlighted = part, [], [], {}, {}
        self.roles, self.visible = list(roles), [True] * len(roles)
        for b in range(part.n_bodies):
            tri = np.arange(len(part.faces)) if part.body_ids is None else np.flatnonzero(part.body_ids == b)
            self.triangles.append(tri)
            self.actors.append(self.pl.add_mesh(_polydata(part.vertices, part.faces[tri]), smooth_shading=False))
        self.set_roles(roles)
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
        self.pl.render()

    def fit(self):
        """Zoom to the visible bodies."""
        shown = [a for a, v in zip(self.actors, self.visible) if v] or self.actors
        if shown:
            bounds = np.array([a.GetBounds() for a in shown])
            self.pl.reset_camera(bounds=(bounds[:, 0].min(), bounds[:, 1].max(), bounds[:, 2].min(),
                                         bounds[:, 3].max(), bounds[:, 4].min(), bounds[:, 5].max()))
        self.pl.render()

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
        self.pl.render()

    # --- picking ----------------------------------------------------------------
    def start_pick(self, callback, hint="") -> bool:
        """The next left click on a visible body calls `callback(face_id, body)`
        once; clicks beside the part still turn the view."""
        self.cancel_pick()
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
            self.pick_triangle(int(self.triangles[body][picker.GetCellId()]))

    def pick_triangle(self, tri: int):
        """Report the CAD face and body of part triangle `tri` and stop picking."""
        callback = self._pick_cb
        self.cancel_pick()
        if callback is not None:
            body = 0 if self.part.body_ids is None else int(self.part.body_ids[tri])
            callback(int(self.part.face_ids[tri]), body)
