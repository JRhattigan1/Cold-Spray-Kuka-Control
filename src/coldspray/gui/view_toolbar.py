"""Toolbars above the 3D views (camera presets, cell groups, section, ...). Not built yet.

`install(win)` is called once at the end of startup, when `win.scene` and
`win.part_view` exist; it fills `win.part_view_bar` and `win.cell_view_bar`.
"""

from PySide6.QtWidgets import QDoubleSpinBox, QHBoxLayout, QLabel, QMenu, QToolButton, QWidget, QWidgetAction

from coldspray.gui.view_bar import glass_button_qss
from coldspray.sim.datums import NORMALS


MARKER_LABELS = {"collision": "Collision points", "unreachable": "Unreachable points",
                 "steep": "Gun too steep"}


def install(win):
    win.datum_button = datum_button(win.scene, win.plotter)
    win.cell_view_bar.addWidget(win.datum_button)
    win.marker_button = marker_button(win.scene, win.plotter)
    win.cell_view_bar.addWidget(win.marker_button)


def marker_button(scene, plotter) -> QToolButton:
    """"Markers" button: a checkbox per kind of point marker on the toolpath (all shown at first)."""
    button = QToolButton(text="Markers", popupMode=QToolButton.InstantPopup,
                         toolTip="Show or hide the collision, unreachable and steep-gun points on the toolpath")
    menu = QMenu(button)
    button.actions_by_kind = {}
    for kind, text in MARKER_LABELS.items():
        act = menu.addAction(text)
        act.setCheckable(True)
        act.setChecked(scene.marker_shown[kind])
        act.toggled.connect(lambda on, kind=kind: (scene.set_marker_visible(kind, on), plotter.render()))
        button.actions_by_kind[kind] = act
    button.setMenu(menu)
    button.setStyleSheet(glass_button_qss())
    return button


def datum_button(scene, plotter) -> QToolButton:
    """"Datums" button: a menu with a checkbox and an offset (mm) per datum plane."""
    button = QToolButton(text="Datums", popupMode=QToolButton.InstantPopup,
                         toolTip="Show the XY / XZ / YZ datum planes through the faceplate centre")
    menu = QMenu(button)
    button.actions_by_plane = {}
    for k in NORMALS:
        act = menu.addAction(f"{k} plane")
        act.setCheckable(True)
        act.toggled.connect(lambda on, k=k: (scene.datums.set_visible(k, on), plotter.render()))
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(24, 0, 8, 2)
        h.addWidget(QLabel("offset"))
        off = QDoubleSpinBox(minimum=-2000, maximum=2000, decimals=1, suffix=" mm", singleStep=5)
        off.valueChanged.connect(lambda v, k=k: (scene.datums.set_offset(k, v), plotter.render()))
        h.addWidget(off)
        wa = QWidgetAction(menu)
        wa.setDefaultWidget(row)
        menu.addAction(wa)
        button.actions_by_plane[k] = (act, off)
    button.setMenu(menu)
    button.setStyleSheet(glass_button_qss())
    return button


def refresh_icons(win, mode: str):
    """Re-colour the toolbar icons after a theme change."""
