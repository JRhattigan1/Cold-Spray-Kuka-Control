"""Step 5 Export: checklist before the robot, save the KRL program."""

from pathlib import Path

from PySide6.QtWidgets import QFileDialog, QFormLayout, QMessageBox, QVBoxLayout, QWidget

from coldspray.gui.checks import CheckList, confirm_text, export_checks, open_items
from coldspray.gui.stepper import EXPORT
from coldspray.gui import widgets
from coldspray.gui.widgets import Section, primary_button, wrap_label
from coldspray.models import PROJECT_ROOT
from coldspray.post.krl import KRLError, to_krl


class ExportStep:
    def _build_export_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.addWidget(self._toolpath_notice())

        program = self.program_section = Section("Program")
        form = QFormLayout()
        self.program_fields = {}
        for key, text in (("name", "Name"), ("frames", "BASE / TOOL"), ("output", "Spray output"),
                          ("table", "Turntable")):
            self.program_fields[key] = wrap_label("–")
            form.addRow(text, self.program_fields[key])
        program.add(form)
        self.export_note = wrap_label()      # kept for scripts; the checklist explains the rest
        layout.addWidget(widgets.advanced(program))     # details only; the checklist covers what matters

        robot = self.robot_section = Section("Before the robot")
        self.export_list = CheckList()
        self.export_list.goTo.connect(lambda i: (self._goto(i), self.go_to_simulate()))
        robot.add(self.export_list)
        layout.addWidget(robot)

        self.save_krl_btn = primary_button("Save KRL program...", self._save_krl)
        self.save_krl_btn.setEnabled(False)
        layout.addWidget(self.save_krl_btn)
        self.export_status = wrap_label()
        layout.addWidget(self.export_status)
        note = self.export_save_note = wrap_label("Saving with open items asks once, listing them all.")
        note.setProperty("note", True)
        layout.addWidget(widgets.advanced(note))
        layout.addStretch(1)
        return inner

    def go_to_simulate(self):
        from coldspray.gui.stepper import SIMULATE
        self.go_to(SIMULATE)

    def _show_program(self):
        """Program details for the current toolpath."""
        if self.result is None:
            return
        path, _, cell, _ = self.result
        f = self.program_fields
        f["name"].setText(path.name)
        on_part = self._part_base() is not None
        f["frames"].setText(f"{cell.part_base_number if on_part else cell.base_number} · {cell.tool_number}"
                            + ("  (KRL base on the part)" if on_part else ""))
        f["output"].setText(f"$OUT[{cell.gun_output}]")
        tt = cell.turntable
        f["table"].setText("none" if tt is None else
                           "coupled to BASE" if tt.coupled else "fixed base, axes on every point")

    def _part_base(self):
        """Faceplate BASE <- the KRL base picked on the part, or None."""
        part = getattr(self, "part", None)
        if part is None or getattr(part, "krl_base", None) is None:
            return None
        return part.placement @ part.krl_base

    def _save_krl(self):
        path, proc, cell, robot = self.result
        part_base = self._part_base()
        items = open_items(export_checks(self.scene, cell, self.assets.tool, self._collision_state(),
                                         part_base=part_base is not None))
        if items and QMessageBox.question(self, "Open items", confirm_text(items)) \
                != QMessageBox.StandardButton.Yes:
            return
        out = PROJECT_ROOT / "output"
        out.mkdir(exist_ok=True)
        fn, _ = QFileDialog.getSaveFileName(self, "Save KRL program", str(out / f"{path.name}.src"),
                                            "KRL source (*.src)")
        if not fn:
            return
        try:
            Path(fn).write_text(to_krl(path, robot, cell, proc, part_base), newline="\r\n")
        except (KRLError, ValueError) as e:
            QMessageBox.warning(self, "Could not write program", str(e))
            return
        self.export_status.setText(f"Saved {fn}")
        self._mark_done(EXPORT)
        self._status(f"Saved {fn}")
