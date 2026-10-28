"""Simulate / Export checks: pure logic building a list of Check rows, and CheckList to show them.

collision_state (what the window knows about the background collision check):
  "off"        the Check collisions box is off
  "running"    the check is still running (or about to start)
  "failed: …"  the check raised (text after "failed:" is shown)
  a result     scene.collisions ({sample: pairs}) or the list of Collision hits
"""

from collections import Counter
from dataclasses import dataclass

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QSizePolicy, QToolButton, QVBoxLayout, QWidget

from coldspray.gui import style
from coldspray.gui.widgets import Chip
from coldspray.planner.solve_arm import JUMP_DEG

STATES = ("ok", "bad", "warn", "crash", "todo")
CHIP_KIND = {"ok": "good", "bad": "bad", "warn": "warn", "crash": "crash", "todo": "neutral"}

CHIP_KIND_TOKEN = {"ok": "good", "bad": "bad", "warn": "warn", "crash": "crash", "todo": "muted"}


@dataclass
class Check:
    key: str
    state: str                      # ok | bad | warn | crash | todo
    title: str
    detail: str = ""
    sample: int | None = None       # first problem sample, for "Go to"
    count: int | None = None        # shown in the chip for bad / warn / crash

    def __post_init__(self):
        if self.state not in STATES:
            raise ValueError(f"check state must be one of {STATES}")

    @property
    def chip_text(self) -> str:
        if self.state == "ok":
            return "✓"
        if self.state == "todo":
            return "?"
        if self.count is not None:
            return f"{self.count}"
        return "!"


def _first(mask) -> int | None:
    idx = np.flatnonzero(mask)
    return int(idx[0]) if len(idx) else None


def _hits(collision_state) -> dict[int, set]:
    if isinstance(collision_state, dict):
        return {int(k): set(v) for k, v in collision_state.items()}
    return {int(h.sample): set(h.pairs) for h in collision_state}      # list of Collision


def pair_summary(hits: dict[int, set], top: int = 3) -> str:
    """Most frequent colliding pairs, e.g. "A2 / Filtration unit, A3 / Filtration unit"."""
    counts = Counter(p for pairs in hits.values() for p in pairs)
    pairs = [p for p, _ in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))]
    text = ", ".join(f"{a} / {b}" for a, b in pairs[:top])
    return text + (f" and {len(pairs) - top} more" if len(pairs) > top else "")


def _flips(scene):
    """Samples (the later of each pair) where a joint steps > JUMP_DEG inside one non-PTP move;
    None when the scene lacks joints / move kinds. Kinds: scene.move_kind (per sample) or
    scene.path.moves[scene.samples.move_index]."""
    joints = getattr(scene, "joints", None)
    samples = getattr(scene, "samples", None)
    if joints is None or samples is None or len(joints) != scene.n_points:
        return None
    mi = np.asarray(samples.move_index)
    kind = getattr(scene, "move_kind", None)
    if kind is None:
        moves = getattr(getattr(scene, "path", None), "moves", None)
        if moves is None:
            return None
        kind = [moves[i].kind for i in mi]
    lin = np.asarray(kind) != "PTP"
    j = np.asarray(joints, float)
    if len(j) < 2:
        return np.zeros(0, int)
    with np.errstate(invalid="ignore"):
        step = np.nanmax(np.abs(np.diff(j, axis=0)), axis=1)
    bad = (mi[1:] == mi[:-1]) & lin[1:] & (step > JUMP_DEG)
    return np.flatnonzero(bad) + 1


def simulate_checks(scene, cell, collision_state, solve_report=None) -> list[Check]:
    """Reach, gun angle and collision rows for the toolpath shown in `scene`."""
    if not getattr(scene, "n_points", 0):
        return [Check("toolpath", "todo", "No toolpath yet", "Generate a toolpath in step 3")]
    out = []
    bad = ~np.asarray(scene.reachable, bool)
    n_bad = int(bad.sum())
    out.append(Check("reach", "ok", "All points reachable") if not n_bad else
               Check("reach", "bad", f"{n_bad} unreachable points",
                     "The robot cannot reach these (red): move the part, gantry or table", _first(bad), n_bad))

    limit = cell.gun_max_down_deg
    steep = np.asarray(scene.steep, bool)
    n_steep = int(steep.sum())
    worst = max(float(np.max(scene.gun_down)), 0.0)
    out.append(Check("gun", "ok", f"Gun ≤ {limit:.0f}° down (max {worst:.0f}°)") if not n_steep else
               Check("gun", "warn", f"Gun more than {limit:.0f}° down at {n_steep} points",
                     f"Down to {worst:.0f}° below horizontal (yellow), bad for the equipment",
                     _first(steep), n_steep))

    flips = _flips(scene)
    if flips is not None:
        out.append(Check("flip", "ok", "No configuration flips in linear moves") if not len(flips) else
                   Check("flip", "bad", f"{len(flips)} configuration flips inside linear moves",
                         f"A joint jumps more than {JUMP_DEG:.0f}° between neighbouring points of one LIN move; "
                         "the real robot cannot follow that", int(flips[0]), len(flips)))

    if isinstance(collision_state, str):
        if collision_state == "off":
            out.append(Check("collisions", "warn", "Collision check off",
                             "Turn on Check collisions and generate again to check the cell"))
        elif collision_state == "running":
            out.append(Check("collisions", "todo", "Collision check running…",
                             "Robot, gun and table against the cell and the part"))
        elif collision_state.startswith("failed"):
            msg = collision_state.partition(":")[2].strip()
            out.append(Check("collisions", "warn", "Collision check failed", msg))
        else:
            raise ValueError(f"unknown collision state {collision_state!r}")
    else:
        hits = _hits(collision_state)
        if hits:
            out.append(Check("collisions", "crash", f"{len(hits)} collision points", pair_summary(hits),
                             min(hits), len(hits)))
        else:
            out.append(Check("collisions", "ok", "No collisions found"))
    if solve_report is not None:
        out.extend(solve_checks(solve_report))
    return out


def solve_checks(report) -> list[Check]:
    """Rows for a SolveReport: what the solver fixed, and each problem point it could not."""
    out = []
    for line in report.fixed:
        out.append(Check("solve_fixed", "ok", "Solver: " + line))
    sug = getattr(report, "suggested", None) or {}
    if "gantry_e1" in sug:
        out.append(Check("solve_gantry", "ok", f"Suggested gantry E1 = {sug['gantry_e1']:.0f} mm",
                         "The solved toolpath already drives the gantry there; Apply sets it in the cell"))
    if "purge_point" in sug:
        x, y, z = sug["purge_point"]
        out.append(Check("solve_purge", "ok", f"Suggested purge point = ({x:.0f}, {y:.0f}, {z:.0f}) mm",
                         "WORLD, spray direction unchanged; the solved toolpath already uses it, Apply sets it"
                         " in the cell"))
    if report.unfixed:
        first = report.unfixed[0]
        shown = "; ".join(f"point {i.sample} ({i.kind}{': ' + i.detail if i.detail else ''})"
                          for i in report.unfixed[:5])
        more = f" and {len(report.unfixed) - 5} more" if len(report.unfixed) > 5 else ""
        out.append(Check("solve_unfixed", "bad", f"{len(report.unfixed)} problem points could not be fixed",
                         "Left as planned; export is blocked until they are dealt with: " + shown + more,
                         first.sample, len(report.unfixed)))
    elif not report.fixed:
        out.append(Check("solve_none", "ok", "Solver: nothing to fix"))
    return out


def robot_checks(cell, tool, part_base: bool = False) -> list[Check]:
    """Items not yet confirmed on the real robot (tool.yaml, cell.yaml, README list).
    `part_base`: the program is written relative to a KRL base picked on the part."""
    out = []
    if tool.verified:
        out.append(Check("tcp", "ok", "Gun TCP verified on the robot"))
    else:
        how = ("Taught TOOL_DATA values from tool.yaml (tcp)" if tool.tcp is not None else
               "Computed from the gun + nozzle CAD: check clock_deg and nozzle length, or paste the taught "
               "TOOL_DATA")
        out.append(Check("tcp", "todo", "Gun TCP not verified on the robot",
                         how + "; set verified: true in tool.yaml once checked"))
    b = cell.base_number
    if cell.turntable is not None:
        mode = ("coupled: positions move with the faceplate" if cell.turntable.coupled else
                "fixed base: axis values added to every point")
        out.append(Check("coupling", "todo", f"Turntable coupling to BASE {b} not confirmed",
                         f"Assumed {mode} (cell.yaml turntable.coupled)"))
    if part_base:
        out.append(Check("base", "todo", f"BASE_DATA[{cell.part_base_number}] taught on the part",
                         "This program is relative to the KRL base picked in the Part step: teach it on the"
                         " real part (3-point: origin corner, along the X edge, into the face's XY plane)"
                         + ("" if cell.turntable is None or cell.turntable.coupled else ", table axes at 0")
                         + " and compare with the program header"))
    else:
        out.append(Check("base", "todo", f"BASE_DATA[{b}] taught at the faceplate centre",
                         "Programs are written relative to the turntable faceplate top centre"))
    if cell.gantry is not None:
        out.append(Check("e1_zero", "todo", f"{cell.gantry.axis} = 0 matches the CAD carriage position",
                         "And the gantry travel limits"))
    if cell.turntable is not None:
        t = cell.turntable
        out.append(Check("table_dirs", "todo", f"Turntable directions: +{t.tilt.axis} and +{t.rotation.axis}",
                         f"+{t.tilt.axis} tilts towards the tailstock, which way +{t.rotation.axis} turns, "
                         "axis limits and max speeds"))
    out.append(Check("kinematics", "todo", "Kinematics checked against the pendant",
                     "Compare pendant X/Y/Z/A/B/C with our FK at several joint positions"))
    out.append(Check("first_run", "todo", "First run in T1, spray off",
                     "Well clear of the part"))
    return out


def export_checks(scene, cell, tool, collision_state, solve_report=None, part_base=False) -> list[Check]:
    """The simulate checks plus the robot-side items for the "Before the robot" list."""
    return simulate_checks(scene, cell, collision_state, solve_report) + robot_checks(cell, tool, part_base)


def open_items(checks) -> list[Check]:
    return [c for c in checks if c.state != "ok"]


def confirm_text(items) -> str:
    """One confirmation listing every open item ("" if there are none)."""
    if not items:
        return ""
    lines = [f"• {c.title}" + (f" - {c.detail}" if c.detail else "") for c in items]
    n = len(items)
    return (f"{n} open item{'s' if n != 1 else ''} before running on the robot:\n\n" + "\n".join(lines)
            + "\n\nSave the program anyway?")


class CheckList(QWidget):
    """Rows of chip, title, muted detail and an optional "Go to" button."""

    goTo = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        self.grid_host = None
        self.checks: list[Check] = []
        self.rows: list[dict] = []

    def set_checks(self, checks):
        self.checks = list(checks)
        if self.grid_host is not None:
            self.layout().removeWidget(self.grid_host)
            self.grid_host.deleteLater()
        self.grid_host = QWidget()
        grid = QGridLayout(self.grid_host)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        grid.setColumnStretch(1, 1)
        self.rows = []
        t = style.tokens(style.current_mode())
        for r, c in enumerate(self.checks):
            chip = Chip(c.chip_text, CHIP_KIND[c.state])
            chip.setToolTip(c.state)
            title = QLabel(c.title)
            title.setWordWrap(True)
            detail = QLabel(c.detail)
            detail.setWordWrap(True)
            detail.setProperty("note", "true")
            detail.setVisible(bool(c.detail))
            text = QWidget()
            tl = QVBoxLayout(text)
            tl.setContentsMargins(0, 4, 0, 4)
            tl.setSpacing(0)
            tl.addWidget(title)
            tl.addWidget(detail)
            text.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            capsule = QFrame()          # glass capsule behind the whole row
            capsule.setObjectName("checkRow")
            capsule.setStyleSheet(
                f"#checkRow {{ background: {t['soft']}; border: 1px solid {t['line']}; border-radius: 14px; }}")
            grid.addWidget(capsule, r, 0, 1, 3)
            capsule.lower()
            chip.setStyleSheet(f"background: {t[CHIP_KIND_TOKEN[c.state]]}; color: #ffffff;"
                               " border-radius: 9px; padding: 1px 7px; margin-left: 8px;")
            grid.addWidget(chip, r, 0, Qt.AlignTop | Qt.AlignHCenter)
            grid.addWidget(text, r, 1)
            btn = None
            if c.sample is not None:
                btn = QToolButton(text="Go to")
                btn.setToolTip(f"Jump to point {c.sample}")
                btn.clicked.connect(lambda _=False, i=c.sample: self.goTo.emit(i))
                grid.addWidget(btn, r, 2, Qt.AlignTop)
            self.rows.append({"check": c, "chip": chip, "title": title, "detail": detail, "goto": btn})
        self.layout().addWidget(self.grid_host)

    def open_count(self) -> int:
        return len(open_items(self.checks))
