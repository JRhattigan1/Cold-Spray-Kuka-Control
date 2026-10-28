"""Compact toolbar above a 3D view: camera presets, cell groups, section plane,
follow tool and layer filter.

`ViewBar(target, render, kind)` drives a `Scene` (kind="cell") or a
`PartView` (kind="part"); `render` is a zero-argument callable that redraws
the view (plotter.render). Call `refresh()` whenever the target's content
changes (new part, new path, setup entered) so the Cell, Layers and Section
controls match it. Buttons carry object names (view_fit, view_iso, ...) for
styling and icons; there are no colours here.
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QAction, QActionGroup, QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (QFrame, QGraphicsDropShadowEffect, QHBoxLayout, QLabel, QMenu, QSizePolicy, QSpinBox, QToolButton, QWidget,
                               QWidgetAction)

PRESETS = {
    # name: (label, tooltip, kinds)
    "fit": ("Fit", "Fit the whole view", ("cell", "part")),
    "part": ("Part", "Close-up of the part on the faceplate", ("cell",)),
    "iso": ("Iso", "Isometric view", ("cell", "part")),
    "top": ("Top", "View from above", ("cell", "part")),
    "front": ("Front", "View from the front", ("cell", "part")),
    "side": ("Side", "View from the side", ("cell", "part")),
    "bottom": ("Bottom", "View from below", ("part",)),
}


def _rgba(hex_colour: str, alpha: float) -> str:
    c = QColor(hex_colour)
    return f"rgba({c.red()},{c.green()},{c.blue()},{int(alpha * 255)})"


def glass_button_qss(t: dict | None = None) -> str:
    """Round glass buttons: faint translucent fill on hover, orange tint when checked."""
    from coldspray.gui.style import current_mode, tokens
    t = t or tokens(current_mode())
    return f"""
QToolButton {{ background: transparent; border: 1px solid transparent; border-radius: 14px;
    padding: 3px 10px; min-height: 22px; color: {t['ink']}; }}
QToolButton:hover {{ background: {_rgba(t['ink'], 0.08)}; border-color: {_rgba('#ffffff', 0.35)}; }}
QToolButton:pressed {{ background: {_rgba(t['ink'], 0.16)}; }}
QToolButton:checked {{ background: {_rgba(t['accent'], 0.22)}; border: 1px solid {_rgba(t['accent'], 0.7)};
    color: {t['accent']}; }}
QToolButton:disabled {{ color: {t['muted']}; }}
QToolButton::menu-indicator {{ image: none; width: 0px; }}
QToolButton::menu-button {{ border: none; width: 12px; }}
"""


LAYER_MODES = (("all", "All layers"), ("upto", "Up to layer"), ("only", "Only layer"))


class ViewBar(QWidget):
    groupToggled = Signal(str, bool)      # a cell group was shown / hidden from the Cell menu

    def __init__(self, target, render, kind: str = "cell", parent=None):
        super().__init__(parent)
        if kind not in ("cell", "part"):
            raise ValueError(f"kind must be 'cell' or 'part', not {kind!r}")
        self.target, self.render, self.kind = target, render, kind
        self.setObjectName(f"view_bar_{kind}")
        row = QHBoxLayout(self)
        row.setContentsMargins(10, 4, 10, 4)
        row.setSpacing(3)
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(18)
        shadow.setOffset(0, 3)
        shadow.setColor(QColor(0, 0, 0, 70))
        self.setGraphicsEffect(shadow)
        self.apply_glass()
        self.buttons: dict[str, QToolButton] = {}

        for name, (label, tip, kinds) in PRESETS.items():
            if kind in kinds:
                b = self._button(name, label, tip)
                b.clicked.connect(lambda _=False, n=name: self.preset(n))
                row.addWidget(b)

        line = QFrame()
        line.setObjectName("view_sep")
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        row.addWidget(line)

        # Cell groups
        self.cell_menu = None
        if kind == "cell":
            b = self._button("cell", "Cell ▾", "Show or hide cell equipment")
            self.cell_menu = QMenu(b)
            b.setMenu(self.cell_menu)
            b.setPopupMode(QToolButton.InstantPopup)
            row.addWidget(b)

        # Section plane: click toggles, the arrow (or right-click) picks the axis
        b = self._button("section", "Section", "Cut the view with a plane through the part\n"
                                               "(arrow or right-click: X / Y / Z)")
        b.setCheckable(True)
        b.toggled.connect(self._section_toggled)
        self.axis_menu = QMenu(b)
        self.axis_group = QActionGroup(self.axis_menu)
        self.axis_actions = {}
        for axis in "xyz":
            act = QAction(f"{axis.upper()} plane", self.axis_menu, checkable=True)
            act.setObjectName(f"view_section_{axis}")
            act.setChecked(axis == getattr(target, "section_axis", "x"))
            act.triggered.connect(lambda _=False, a=axis: self.set_section_axis(a))
            self.axis_group.addAction(act)
            self.axis_menu.addAction(act)
            self.axis_actions[axis] = act
        b.setMenu(self.axis_menu)
        b.setPopupMode(QToolButton.MenuButtonPopup)
        b.setContextMenuPolicy(Qt.CustomContextMenu)
        b.customContextMenuRequested.connect(lambda pos, btn=b: self.axis_menu.exec(btn.mapToGlobal(pos)))
        row.addWidget(b)

        if kind == "cell":
            b = self._button("follow", "Follow tool", "Keep the camera on the nozzle during playback")
            b.setCheckable(True)
            b.toggled.connect(self._follow_toggled)
            row.addWidget(b)

            b = self._button("layers", "Layers ▾", "Show all layers, up to a layer, or one layer")
            self.layer_menu = QMenu(b)
            self.layer_group = QActionGroup(self.layer_menu)
            self.layer_actions = {}
            for mode, label in LAYER_MODES:
                act = QAction(label, self.layer_menu, checkable=True)
                act.setObjectName(f"view_layers_{mode}")
                act.setChecked(mode == "all")
                act.triggered.connect(lambda _=False, m=mode: self.set_layer_mode(m))
                self.layer_group.addAction(act)
                self.layer_menu.addAction(act)
                self.layer_actions[mode] = act
            box = QWidget()
            h = QHBoxLayout(box)
            h.setContentsMargins(8, 2, 8, 2)
            h.addWidget(QLabel("Layer"))
            self.layer_spin = QSpinBox()
            self.layer_spin.setObjectName("view_layer_n")
            self.layer_spin.setRange(1, 1)
            self.layer_spin.valueChanged.connect(self._layer_n_changed)
            h.addWidget(self.layer_spin)
            self.layer_count = QLabel("of 0")
            h.addWidget(self.layer_count)
            wa = QWidgetAction(self.layer_menu)
            wa.setDefaultWidget(box)
            self.layer_menu.addSeparator()
            self.layer_menu.addAction(wa)
            b.setMenu(self.layer_menu)
            b.setPopupMode(QToolButton.InstantPopup)
            row.addWidget(b)
            self.layer_mode = "all"
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)   # a capsule, not a full-width bar
        self.refresh()

    def apply_glass(self):
        """(Re)read the theme tokens: restyle the buttons; the capsule is painted in paintEvent."""
        from coldspray.gui.style import current_mode, tokens
        self._tokens = t = tokens(current_mode())
        self.setStyleSheet(glass_button_qss(t) + f"QFrame#view_sep {{ color: {_rgba(t['ink'], 0.15)}; }}")
        self.update()

    def paintEvent(self, _event):
        t = self._tokens
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = self.rect().adjusted(2, 2, -2, -4)
        path = QPainterPath()
        path.addRoundedRect(r, r.height() / 2, r.height() / 2)
        fill = QColor(t["panel"])
        fill.setAlpha(158)
        p.fillPath(path, fill)
        hi = QColor(255, 255, 255, 140)
        p.setPen(QPen(hi, 1))
        p.drawPath(path)

    def _button(self, name: str, label: str, tip: str) -> QToolButton:
        b = QToolButton(self)
        b.setObjectName(f"view_{name}")
        b.setText(label)
        b.setToolTip(tip)
        b.setAutoRaise(True)
        b.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self.buttons[name] = b
        return b

    # --- actions -----------------------------------------------------------------
    def preset(self, name: str):
        self.target.view_preset(name)
        self.render()

    def _section_toggled(self, on: bool):
        self.target.set_section(on)
        self.render()

    def set_section_axis(self, axis: str):
        """Pick the section axis; turns the section on."""
        self.axis_actions[axis].setChecked(True)
        b = self.buttons["section"]
        if b.isChecked():
            self.target.set_section(True, axis)
            self.render()
        else:
            self.target.section_axis = axis
            b.setChecked(True)          # -> _section_toggled

    def _follow_toggled(self, on: bool):
        self.target.set_follow(on)
        self.render()

    def _group_toggled(self, label: str, on: bool):
        self.target.set_group_visible(label, on)
        self.render()
        self.groupToggled.emit(label, on)

    def set_layer_mode(self, mode: str):
        self.layer_mode = mode
        self.layer_actions[mode].setChecked(True)
        self._apply_layers()

    def set_layer(self, n: int):
        """Layer number (0-based) for "up to" / "only", e.g. the layer being played."""
        self.layer_spin.setValue(int(n) + 1)       # -> _layer_n_changed

    def _layer_n_changed(self, _value):
        if self.layer_mode != "all":
            self._apply_layers()

    def _apply_layers(self):
        if self.target.set_layer_filter(self.layer_mode, self.layer_spin.value() - 1):
            self.render()

    # --- sync with the target ----------------------------------------------------
    def refresh(self):
        """Re-read the cell groups, layer count and section / follow state."""
        t = self.target
        sec = self.buttons["section"]
        sec.blockSignals(True)
        sec.setChecked(getattr(t, "section_plane", None) is not None)
        sec.blockSignals(False)
        self.axis_actions[getattr(t, "section_axis", "x")].setChecked(True)
        if self.kind != "cell":
            return
        self.cell_menu.clear()
        groups = t.cell_groups() if hasattr(t, "cell_groups") else []
        for label, visible in groups:
            act = QAction(label, self.cell_menu, checkable=True)
            act.setChecked(visible)
            act.toggled.connect(lambda on, lab=label: self._group_toggled(lab, on))
            self.cell_menu.addAction(act)
        self.buttons["cell"].setEnabled(bool(groups))

        fol = self.buttons["follow"]
        fol.blockSignals(True)
        fol.setChecked(bool(getattr(t, "follow", False)))
        fol.blockSignals(False)
        fol.setEnabled(getattr(t, "mode", "preview") == "preview" and bool(getattr(t, "n_points", 0)))

        n = int(getattr(t, "n_layers", 0) or 0)
        self.buttons["layers"].setEnabled(n > 0)
        mode, cur = getattr(t, "layer_filter", ("all", None))
        self.layer_spin.blockSignals(True)
        self.layer_spin.setRange(1, max(n, 1))
        if cur is not None:
            self.layer_spin.setValue(cur + 1)
        self.layer_spin.blockSignals(False)
        self.layer_count.setText(f"of {n}")
        self.layer_mode = mode if n else "all"
        self.layer_actions[self.layer_mode].setChecked(True)
