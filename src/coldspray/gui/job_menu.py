"""Job save / open in the main window: a "Job" menu in the step bar (Save job..., Open job...,
recent jobs) and Ctrl+S / Ctrl+O. The file format is coldspray.job_io."""

from pathlib import Path

import numpy as np
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import QFileDialog, QLineEdit, QMenu, QMessageBox, QToolButton

from coldspray import job_io
from coldspray.gui import widgets
from coldspray.gui.stepper import POSITION, TOOLPATH
from coldspray.gui.steps.toolpath import box_value, split_plane
from coldspray.models import PROJECT_ROOT, ProcessConfig, PurgeConfig

RECENT_KEY = "jobs/recent"
RECENT_MAX = 6
FILTER = f"Cold spray job (*{job_io.SUFFIX});;All files (*)"


# --- window state <-> job dict ------------------------------------------------------
def collect(win) -> dict:
    """The job as a dict from the window's current choices (every step)."""
    src = win.source
    part = {}
    if src is not None:
        source = getattr(win, "part_source", None)
        part = dict(source=str(Path(source).resolve()) if source and Path(str(source)).suffix else source,
                    roles=list(win.roles), build_body=src.build_body, start_face=src.start_face,
                    base_face=src.base_face, coat_faces=sorted(src.coat_faces), krl_base=src.krl_base,
                    krl_base_face=src.krl_base_face)
    rows = [dict(kind=r.kind.currentText(), face=r.face.currentText(), face2=r.face2.currentText(),
                 ref=r.ref.currentText(), value=r.value.value(), suppressed=r.suppress.isChecked(),
                 pose=r.pose) for r in win.constraints.rows]
    sub = win.substrate
    position = dict(
        fixed_in_cell=win._fixed(), keep_on_plate=win.keep_on_plate.isChecked(),
        placement=None if win.part is None else win.part.placement,
        substrate=dict(enabled=win.sub_check.isChecked(),
                       **{k: b.value() for k, b in win.sub_fields.items()}),
        nozzle=win.nozzle.currentData(), tilt_deg=win.tilt.value(), rotation_deg=win.rotation.value(),
        e1_mm=win.e1.value() if win.e1.isEnabled() else None, roll_deg=win.roll.value(), constraints=rows)
    if sub is None:
        position["substrate"]["enabled"] = False
    process = {name: box_value(box) for name, box in win.fields.items()}
    return job_io.plain(dict(
        version=job_io.JOB_VERSION, part=part, position=position,
        toolpath=dict(mode=win.mode.currentData(), coats=win.coats.value(), process=process),
        purge=dict(win.purge_cfg), export=dict(preflight=win.preflight_check.isChecked(),
                    spray_output=win.spray_output_check.isChecked(), split=win.split_check.isChecked())))


def _set(box, value):
    if value is not None:
        box.setValue(float(value))


def _set_process_box(box, value):
    from PySide6.QtWidgets import QCheckBox, QComboBox
    if isinstance(box, QComboBox):
        i = box.findData(value)
        if i >= 0:
            box.setCurrentIndex(i)
    elif isinstance(box, QCheckBox):
        box.setChecked(bool(value))
    elif isinstance(box, QLineEdit):
        box.setText(str(value))
    else:
        box.setValue(value)


def apply(win, job: dict):
    """Restore every step's widgets from `job` (job_io.normalize'd). The toolpath is not
    generated: press Generate (the same settings give the same KRL)."""
    job = job_io.normalize(job)
    p, pos, tp = job["part"], job["position"], job["toolpath"]
    win._set_part(p["source"])
    src = win.source
    if src is None or getattr(win, "part_source", object()) != p["source"]:
        raise ValueError(f"Could not load the part {p['source']}")

    # Part step: bodies, faces, coat faces, KRL base
    if len(p["roles"]) == src.n_bodies:
        win.roles = list(p["roles"])
    if 0 <= int(p["build_body"]) < src.n_bodies:
        src.build_body = int(p["build_body"])
        win.roles = ["build" if b == src.build_body else ("fixture" if r == "build" else r)
                     for b, r in enumerate(win.roles)]
    src.start_face, src.base_face = p["start_face"], p["base_face"]
    src.coat_faces = set(int(f) for f in p["coat_faces"])
    src.krl_base = None if p["krl_base"] is None else np.array(p["krl_base"], float)
    src.krl_base_face = p["krl_base_face"]
    win._fill_bodies()
    win.part_view.set_roles(win.roles)
    win.part_view.highlight("start", src.start_face)
    win.part_view.highlight("base", src.base_face)
    win.part_view.highlight_coat(src.coat_faces)
    win.part_view.show_frame(src.krl_base)
    win._show_faces()

    # Toolpath step: mode, coats, every process value
    i = win.mode.findData(tp["mode"])
    if i >= 0:
        win.mode.setCurrentIndex(i)
    win.coats.setValue(int(tp["coats"]))
    process = tp["process"] if isinstance(tp["process"], dict) else {}
    for name, value in process.items():
        box = win.fields.get(name)
        if box is not None and name in ProcessConfig.model_fields:
            _set_process_box(box, value)
    row = getattr(win, "plane_row", None)
    if row is not None and "turn_plane" in process:     # the datum combo + offset that edit turn_plane
        name, offset = split_plane(str(process["turn_plane"]))
        for w in (row.combo, row.spin):
            w.blockSignals(True)
        row.combo.setCurrentIndex(max(0, row.combo.findData(name)))
        row.spin.setValue(offset)
        row.spin.setEnabled(bool(name))
        for w in (row.combo, row.spin):
            w.blockSignals(False)
    win._process_changed()

    # Position step: plate, mount, table / gantry / gun, then the part in the cell
    win.mount.setCurrentIndex(max(0, win.mount.findData(bool(pos["fixed_in_cell"]))))
    s = pos["substrate"]
    for k, b in win.sub_fields.items():
        _set(b, s.get(k))
    win.sub_check.setChecked(bool(s.get("enabled", True)) and not win._fixed())
    if pos["nozzle"] is not None and win.nozzle.findData(pos["nozzle"]) >= 0:
        win.nozzle.setCurrentIndex(win.nozzle.findData(pos["nozzle"]))
    for box, key in ((win.tilt, "tilt_deg"), (win.rotation, "rotation_deg"), (win.e1, "e1_mm"),
                     (win.roll, "roll_deg")):
        _set(box, pos[key])
    win.go_to(POSITION)                         # the part goes into the cell
    if win.part is not None:
        if win.keep_on_plate.isEnabled():
            win.keep_on_plate.setChecked(bool(pos["keep_on_plate"]))
        for c in pos["constraints"]:
            try:
                r = win.constraints.add(c.get("kind", "fix"), c.get("face") or None, c.get("ref", "faceplate"),
                                        float(c.get("value", 0.0)), c.get("face2") or None)
            except (TypeError, ValueError):
                continue
            if c.get("pose") is not None:
                r.pose = np.array(c["pose"], float)
            r.suppress.setChecked(bool(c.get("suppressed", False)))
        if pos["placement"] is not None:
            placement = np.array(pos["placement"], float)
            win._set_placement(placement)       # updates the constraint status
            win.part.placement = placement      # exactly as saved: the same KRL
            win.scene.set_part_placement()
            win._show_placement()
            win._enter_setup()

    # Purge point (the cell's copy, as the Position step's edits do) and export options
    if isinstance(job["purge"], dict):
        cfg = PurgeConfig(**{**win.purge_cfg, **{k: v for k, v in job["purge"].items()
                                                 if k in PurgeConfig.model_fields}})
        win.purge_cfg = cfg.model_dump()
        cell = win._purge_cell()
        if cell is not None:
            cell.purge = PurgeConfig(**win.purge_cfg)
        win._purge_load()
    win.preflight_check.setChecked(bool(job["export"].get("preflight", False)))
    win.spray_output_check.setChecked(bool(job["export"].get("spray_output", True)))
    win.split_check.setChecked(bool(job["export"].get("split", False)))
    widgets.refresh()
    if getattr(win, "undo", None) is not None:
        win.undo.reset()                 # undo starts from the opened job, not its restore steps
    win.go_to(TOOLPATH)


# --- menu, dialogs, recent jobs -------------------------------------------------------
def recent(win) -> list[str]:
    value = win.ui_settings.value(RECENT_KEY, []) or []
    return [value] if isinstance(value, str) else list(value)


def _remember(win, path):
    path = str(Path(path).resolve())
    items = [path] + [p for p in recent(win) if p != path]
    win.ui_settings.setValue(RECENT_KEY, items[:RECENT_MAX])
    _fill_menu(win)


def save_job(win, path=None) -> Path | None:
    """Save the job (asks for a file if `path` is None); returns the path written."""
    if win.source is None:
        win._status("Nothing to save: load a part first")
        return None
    if path is None:
        start = getattr(win, "job_path", None) or str(PROJECT_ROOT / "output" / f"{win.source.name}{job_io.SUFFIX}")
        path, _ = QFileDialog.getSaveFileName(win, "Save job", str(start), FILTER)
        if not path:
            return None
    try:
        path = job_io.save_job(collect(win), path)
    except OSError as e:
        QMessageBox.warning(win, "Save job", f"Could not save: {e}")
        return None
    win.job_path = str(path)
    _remember(win, path)
    win._status(f"Job saved to {path}")
    return path


def open_job(win, path=None) -> bool:
    """Open a job (asks for a file if `path` is None) and restore every step."""
    if path is None:
        start = getattr(win, "job_path", None) or str(PROJECT_ROOT / "output")
        path, _ = QFileDialog.getOpenFileName(win, "Open job", str(start), FILTER)
        if not path:
            return False
    try:
        job = job_io.load_job(path)
        apply(win, job)
    except Exception as e:      # unreadable file, missing STEP, bad values
        QMessageBox.warning(win, "Open job", f"Could not open {path}:\n{e}")
        return False
    win.job_path = str(path)
    _remember(win, path)
    win._status(f"Opened {Path(path).name}: press Generate for the toolpath")
    return True


def _fill_menu(win):
    menu = win.job_menu
    menu.clear()
    menu.addAction("Save job...\tCtrl+S", lambda: save_job(win))
    menu.addAction("Open job...\tCtrl+O", lambda: open_job(win))
    items = recent(win)
    if items:
        menu.addSeparator()
        for p in items:
            menu.addAction(Path(p).name, lambda p=p: open_job(win, p)).setToolTip(p)


def install(win):
    """Add the Job menu button to the step bar and the Ctrl+S / Ctrl+O shortcuts."""
    btn = win.job_btn = QToolButton()
    btn.setText("Job")
    btn.setToolTip("Save or open a job: the part and every step's settings")
    btn.setPopupMode(QToolButton.InstantPopup)
    win.job_menu = QMenu(btn)
    btn.setMenu(win.job_menu)
    _fill_menu(win)
    win.stepbar.chip_area.insertWidget(0, btn)
    win.job_shortcuts = [QShortcut(QKeySequence("Ctrl+S"), win, lambda: save_job(win)),
                         QShortcut(QKeySequence("Ctrl+O"), win, lambda: open_job(win))]
