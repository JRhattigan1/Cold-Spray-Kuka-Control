"""Undo / redo for the Position step: part placement, mount, keep on plate, substrate plate, nozzle,
table tilt / rotation, gantry E1, gun angle and the constraint rows.

A snapshot of those settings is committed once the edits pause (COALESCE_MS), so a drag, a held spin
arrow or a burst of typed values is one step. Undo restores the widgets, the part placement and the
scene; the toolpath is not regenerated (it is out of date, as after any Position edit).
Ctrl+Z undoes, Ctrl+Shift+Z / Ctrl+Y redo (Position step only)."""

import numpy as np
from PySide6.QtCore import QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import QToolButton

from coldspray.gui import widgets
from coldspray.gui.stepper import POSITION

LIMIT = 50
COALESCE_MS = 400


def snapshot(win) -> dict:
    """The Position step's undoable state."""
    return dict(
        part=id(win.part), placement=np.array(win.part.placement, float),
        fixed=win._fixed(), keep=win.keep_on_plate.isChecked(), sub=win.sub_check.isChecked(),
        sub_fields={k: b.value() for k, b in win.sub_fields.items()}, nozzle=win.nozzle.currentIndex(),
        tilt=win.tilt.value(), rotation=win.rotation.value(), e1=win.e1.value(), roll=win.roll.value(),
        rows=win.constraints.rows_state())


def same(a: dict, b: dict) -> bool:
    if a.keys() != b.keys():
        return False
    for k in a:
        if k == "placement":
            if not np.allclose(a[k], b[k], atol=1e-9):
                return False
        elif k == "rows":
            if len(a[k]) != len(b[k]) or any(
                    {**x, "pose": None} != {**y, "pose": None}
                    or (x["pose"] is None) != (y["pose"] is None)
                    or (x["pose"] is not None and not np.allclose(x["pose"], y["pose"]))
                    for x, y in zip(a[k], b[k])):
                return False
        elif a[k] != b[k]:
            return False
    return True


class Undo:
    def __init__(self, win):
        self.win = win
        self.undo_stack, self.redo_stack, self.current = [], [], None
        self.restoring = False
        self.timer = QTimer(win)
        self.timer.setSingleShot(True)
        self.timer.setInterval(COALESCE_MS)
        self.timer.timeout.connect(self.commit)

    def touch(self, *_):
        """An edit happened: commit it once the edits pause."""
        if not self.restoring and self.win.part is not None:
            self.timer.start()

    def reset(self):
        """A new part in the cell: forget the history, start from its state."""
        self.timer.stop()
        self.undo_stack.clear()
        self.redo_stack.clear()
        self.current = None if self.win.part is None else snapshot(self.win)
        self._buttons()

    def commit(self):
        """Push the state before the pending edits (if anything changed)."""
        self.timer.stop()
        if self.restoring or self.win.part is None:
            return
        s = snapshot(self.win)
        if self.current is None or self.current["part"] != s["part"]:
            self.reset()
            return
        if same(s, self.current):
            return
        self.undo_stack.append(self.current)
        del self.undo_stack[:-LIMIT]
        self.redo_stack.clear()
        self.current = s
        self._buttons()

    def undo(self) -> bool:
        return self._step(self.undo_stack, self.redo_stack, "Undo")

    def redo(self) -> bool:
        return self._step(self.redo_stack, self.undo_stack, "Redo")

    def _step(self, src, dst, name) -> bool:
        win = self.win
        if getattr(win, "step", POSITION) != POSITION:
            win._status(f"{name}: works in the Position step")
            return False
        self.commit()
        if not src or self.current is None:
            win._status(f"Nothing to {name.lower()}")
            return False
        dst.append(self.current)
        self.current = src.pop()
        self.restore(self.current)
        self._buttons()
        win._status(f"{name}: Position settings restored (press Generate for the toolpath)")
        return True

    def restore(self, s: dict):
        """Put the widgets, part placement and scene back to snapshot `s`."""
        win = self.win
        self.restoring = True
        try:
            if win._fixed() != s["fixed"]:
                win.mount.setCurrentIndex(max(0, win.mount.findData(s["fixed"])))
            boxes = [*win.sub_fields.values(), win.sub_check, win.keep_on_plate, win.nozzle,
                     win.tilt, win.rotation, win.e1, win.roll]
            for b in boxes:
                b.blockSignals(True)
            for k, b in win.sub_fields.items():
                b.setValue(s["sub_fields"][k])
            win.sub_check.setChecked(s["sub"])
            win.keep_on_plate.setChecked(s["keep"])
            if 0 <= s["nozzle"] < win.nozzle.count():
                win.nozzle.setCurrentIndex(s["nozzle"])
            for name in ("tilt", "rotation", "e1", "roll"):
                getattr(win, name).setValue(s[name])
            for b in boxes:
                b.blockSignals(False)
            win._substrate_changed()                  # substrate object, plate mesh (placement set below)
            if win.scene is not None:
                win.scene.set_constrained(win._kept())
            win.constraints.set_rows(s["rows"])
            placement = np.array(s["placement"], float)
            win.constraints.apply(placement)          # row status icons, DOF chip
            win.part.placement = placement            # exactly as it was
            if win.scene is not None:
                win.scene.set_part_placement()
            win._show_placement()
            win._enter_setup()
            widgets.refresh()
        finally:
            self.timer.stop()
            self.restoring = False

    def _buttons(self):
        for btn, stack in ((getattr(self, "undo_btn", None), self.undo_stack),
                           (getattr(self, "redo_btn", None), self.redo_stack)):
            if btn is not None:
                btn.setEnabled(bool(stack))


def install(win):
    """Undo / redo buttons in the step bar, Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y, and the edit hooks."""
    u = win.undo = Undo(win)
    for key, slot in (("Ctrl+Z", u.undo), ("Ctrl+Shift+Z", u.redo), ("Ctrl+Y", u.redo)):
        win.shortcuts.append(QShortcut(QKeySequence(key), win, slot))
    for attr, text, tip, slot in (("undo_btn", "↶", "Undo the last Position edit (Ctrl+Z)", u.undo),
                                  ("redo_btn", "↷", "Redo (Ctrl+Shift+Z or Ctrl+Y)", u.redo)):
        btn = QToolButton()
        btn.setText(text)
        btn.setToolTip(tip)
        btn.setEnabled(False)
        btn.clicked.connect(slot)
        setattr(u, attr, btn)
        win.stepbar.chip_area.insertWidget(1 if attr == "undo_btn" else 2, btn)
    for b in (*win.place.values(), *win.sub_fields.values(), win.tilt, win.rotation, win.e1, win.roll):
        b.valueChanged.connect(u.touch)
    for sig in (win.mount.currentIndexChanged, win.nozzle.currentIndexChanged, win.keep_on_plate.toggled,
                win.sub_check.toggled, win.constraints.changed):
        sig.connect(u.touch)
    return u
