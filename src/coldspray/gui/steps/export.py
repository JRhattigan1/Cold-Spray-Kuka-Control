"""Step 5 Export: save the KRL program."""

from pathlib import Path

from PySide6.QtWidgets import QFileDialog, QMessageBox, QPushButton, QVBoxLayout, QWidget

from coldspray.gui.stepper import EXPORT
from coldspray.gui.widgets import Section, wrap_label
from coldspray.models import PROJECT_ROOT
from coldspray.post.krl import KRLError, to_krl


class ExportStep:
    def _build_export_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.addWidget(self._toolpath_notice())

        program = self.program_section = Section("Program")
        self.export_note = wrap_label("Save the toolpath as a KRL program (.src) for the robot controller. "
                                      "Check the list under \"Before running on the real robot\" in the "
                                      "README before the first run.")
        program.add(self.export_note)
        self.save_krl_btn = QPushButton("Save KRL program...")
        self.save_krl_btn.clicked.connect(self._save_krl)
        self.save_krl_btn.setEnabled(False)
        program.add(self.save_krl_btn)
        self.export_status = wrap_label()
        program.add(self.export_status)
        layout.addWidget(program)
        layout.addStretch(1)
        return inner

    def _save_krl(self):
        path, proc, cell, robot = self.result
        hits = self.collisions
        if hits and QMessageBox.question(
                self, "Collisions found",
                f"The simulation found {len(hits)} collision points. Save the program anyway?") \
                != QMessageBox.StandardButton.Yes:
            return
        steep, limit = int(self.scene.steep.sum()), cell.gun_max_down_deg
        if steep and QMessageBox.question(
                self, "Gun pointing down",
                f"At {steep} points the gun points more than {limit:.0f}° below horizontal "
                f"(down to {self.scene.gun_down.max():.0f}°), which is bad for the equipment. "
                "Save the program anyway?") != QMessageBox.StandardButton.Yes:
            return
        out = PROJECT_ROOT / "output"
        out.mkdir(exist_ok=True)
        fn, _ = QFileDialog.getSaveFileName(self, "Save KRL program", str(out / f"{path.name}.src"),
                                            "KRL source (*.src)")
        if not fn:
            return
        try:
            Path(fn).write_text(to_krl(path, robot, cell, proc), newline="\r\n")
        except (KRLError, ValueError) as e:
            QMessageBox.warning(self, "Could not write program", str(e))
            return
        self.export_status.setText(f"Saved {fn}")
        self._mark_done(EXPORT)
        self._status(f"Saved {fn}")
