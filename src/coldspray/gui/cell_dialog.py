"""Cell dialog: TOOL_DATA / BASE_DATA as on the KUKA pendant, so taught frames copy in exactly."""
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QHBoxLayout, QLabel, QLineEdit,
                               QPushButton, QTableWidget, QTableWidgetItem, QTabWidget,
                               QVBoxLayout, QWidget)

from coldspray import config_io
from coldspray.models import load_cell, load_tool
from coldspray.tool import cad_tool_frame, parse_pendant
from coldspray.transforms import xyzabc

COLS = ["No.", "Name", "X", "Y", "Z", "A", "B", "C"]
KEYS = "xyzabc"


def _fmt(v) -> str:
    return "" if v is None else f"{v:.3f}".rstrip("0").rstrip(".")


class FrameTable(QWidget):
    """Rows of numbered frames (X Y Z mm, A B C deg) + a paste box for a pendant line.
    Empty X..C cells = not taught."""

    def __init__(self, rows, cad=None):
        super().__init__()
        self.keys = [r[0] for r in rows]
        self.cad = cad or {}
        extra = ["CAD", "Diff"] if cad else []
        self.table = QTableWidget(len(rows), len(COLS) + len(extra))
        self.table.setHorizontalHeaderLabels(COLS + extra)
        for i, (_, num, name, frame) in enumerate(rows):
            vals = [str(num), name] + [_fmt(getattr(frame, k)) if frame else "" for k in KEYS]
            for j, v in enumerate(vals):
                self.table.setItem(i, j, QTableWidgetItem(v))
        self.table.itemChanged.connect(lambda *_: self._update_cad())
        self.paste = QLineEdit(placeholderText="Paste {X 517.79,Y 214.24,Z 130.91,A 0,B 0,C 0} into the selected row")
        self.paste.returnPressed.connect(self._paste)
        self.status = QLabel()
        btn = QPushButton("Paste", clicked=self._paste)
        row = QHBoxLayout()
        row.addWidget(self.paste)
        row.addWidget(btn)
        lay = QVBoxLayout(self)
        lay.addWidget(self.table)
        lay.addLayout(row)
        lay.addWidget(self.status)
        self._update_cad()

    def _paste(self):
        try:
            f = parse_pendant(self.paste.text())
        except ValueError as e:
            self.status.setText(str(e))
            return
        i = max(self.table.currentRow(), 0)
        for j, k in enumerate(KEYS):
            self.table.item(i, 2 + j).setText(_fmt(getattr(f, k)))
        self.status.setText(f"Row {i + 1} set")

    def frame(self, i):
        """The taught frame of row i, or None if all its X..C cells are empty."""
        txt = [self.table.item(i, 2 + j).text().strip() for j in range(6)]
        if not any(txt):
            return None
        return dict(zip(KEYS, (float(t or 0) for t in txt)))

    def number(self, i):
        return int(self.table.item(i, 0).text())

    def _update_cad(self):
        if not self.cad:
            return
        self.table.blockSignals(True)
        for i, key in enumerate(self.keys):
            c = self.cad.get(key)
            if c is None:
                continue
            self.table.setItem(i, 8, QTableWidgetItem(" ".join(f"{v:.2f}" for v in c)))
            try:
                f = self.frame(i)
            except ValueError:
                f = None
            diff = "" if f is None else " ".join(f"{f[k] - v:+.2f}" for k, v in zip(KEYS, c))
            self.table.setItem(i, 9, QTableWidgetItem(diff or "(using CAD)"))
        self.table.blockSignals(False)


class CellDialog(QDialog):
    """Non-modal: Tool and Base tabs; `add_tab` for more (e.g. HOME)."""
    saved = Signal(bool)    # True: the active tool's TCP changed

    def __init__(self, parent=None, tool_path=None, cell_path=None):
        super().__init__(parent)
        self.setWindowTitle("Cell")
        self.setModal(False)
        self.tool_path, self.cell_path = tool_path, cell_path
        self.tool = load_tool(tool_path)
        self.cell = load_cell(cell_path)
        gun = self.tool.gun
        self.tabs = QTabWidget()
        rows, cad = [], {}
        for name, nz in gun.nozzles.items():
            num = nz.tool_number or (self.cell.tool_number if name == gun.nozzle else 0)
            label = nz.label + (" (selected)" if name == gun.nozzle else "")
            rows.append((name, num, label, nz.tcp))
            cad[name] = xyzabc(cad_tool_frame(gun, name))
        self.tool_tab = FrameTable(rows, cad)
        self.add_tab(self.tool_tab, "Tool")
        taught = self.cell.bases_taught
        brows = [(n, n, lbl, taught.get(n)) for n, lbl in
                 ((self.cell.base_number, "Faceplate"), (self.cell.part_base_number, "Part"))]
        self.base_tab = FrameTable(brows)
        self.add_tab(self.base_tab, "Base")
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Close)
        buttons.accepted.connect(self.save)
        buttons.rejected.connect(self.close)
        lay = QVBoxLayout(self)
        lay.addWidget(self.tabs)
        lay.addWidget(buttons)
        self.resize(900, 360)

    def add_tab(self, widget, name):
        self.tabs.addTab(widget, name)

    def save(self):
        """Write taught tool/base frames to the YAML (comments kept); returns True if the
        active tool's TCP changed."""
        from coldspray.models import XYZABC
        gun = self.tool.gun
        before = gun.current_nozzle.tcp
        for i, name in enumerate(self.tool_tab.keys):
            f = self.tool_tab.frame(i)
            gun.nozzles[name].tcp = XYZABC(**f) if f else None
            gun.nozzles[name].tool_number = self.tool_tab.number(i)
        config_io.save_tools(self.tool, self.tool_path)
        bases = {}
        for i in range(len(self.base_tab.keys)):
            f = self.base_tab.frame(i)
            if f:
                bases[self.base_tab.number(i)] = XYZABC(**f)
        config_io.save_bases(bases, self.cell_path)
        changed = gun.current_nozzle.tcp != before
        self.saved.emit(changed)
        return changed
