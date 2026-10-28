"""Simulate playback track and controls.

`Timeline` paints the whole toolpath as one bar (samples 0..n-1): spray-on spans
tinted with the accent, problem marks (unreachable, collision, gun too steep)
and a cursor at the current sample; a legend and "mm:ss / mm:ss" sit under it.
Click or drag on the bar to seek. `PlaybackControls` holds the buttons; neither
widget has a timer - the window drives playback.
"""

import numpy as np
from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFontMetrics, QPainter, QPainterPath, QRadialGradient
from coldspray.gui import style
from PySide6.QtWidgets import QCheckBox, QComboBox, QHBoxLayout, QSizePolicy, QToolButton, QWidget

KINDS = ("unreachable", "collision", "steep")        # problem kinds
SHORT_LABELS = {"collision": "collision", "unreachable": "unreach.", "steep": "steep"}
LABELS = {"collision": "collision", "unreachable": "unreachable", "steep": "gun too steep"}
KIND_COLOUR = {"unreachable": "bad", "collision": "crash", "steep": "steep"}
DEFAULT_COLOURS = {
    "accent": "#e8590c", "bad": "#c53030", "crash": "#b8326b", "steep": "#d4a017",
    "track": "#eceff3", "cursor": "#1d2329", "text": "#5b6672",
}
SPEEDS = (0.5, 1.0, 2.0, 5.0, 10.0)


def sample_times(path, samples, proc) -> np.ndarray:
    """Cumulative program time (s) at each sample of `samples` (= path.sample(...)).

    Same model as estimate_time_s in gui/steps/toolpath.py, spread over the samples:
    each sample adds the distance from the previous sample divided by the speed of
    the move it belongs to, and the first sample of a move that switches the spray
    on adds proc.spray_settle_s (the gun dwells before moving). Pure turntable
    moves have no TCP travel and take no time. times[-1] equals estimate_time_s.
    """
    n = len(samples)
    t = np.zeros(n)
    if n < 2:
        return t
    pts = samples.poses[:, :3, 3]
    idx = np.asarray(samples.move_index)
    speed = np.array([m.speed for m in path.moves], float)
    on = np.array([m.gun_on for m in path.moves], bool)
    # as in estimate_time_s: the state before move i is moves[i-1].gun_on, "off" before move 1
    was_on = np.concatenate([[False, False], on[1:-1]])[: len(on)]
    settle = np.where(on & ~was_on, proc.spray_settle_s, 0.0)
    settle[0] = 0.0
    dist = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    sp = speed[idx[1:]]
    dt = np.divide(dist, sp, out=np.zeros_like(dist), where=sp > 0)
    first = np.flatnonzero(np.diff(idx) != 0) + 1          # first sample of each later move
    dt[first - 1] += settle[idx[first]]
    t[1:] = np.cumsum(dt)
    return t


def spans(mask, gap: int = 0) -> list[tuple[int, int]]:
    """Runs of True in `mask` as (start, end) with end exclusive. Runs separated
    by at most `gap` False samples are merged (collisions are only checked at
    every few samples)."""
    m = np.asarray(mask, bool)
    if not m.any():
        return []
    d = np.diff(np.concatenate([[0], m.view(np.int8), [0]]))
    starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    if gap > 0 and len(starts) > 1:
        keep = (starts[1:] - ends[:-1]) > gap
        starts = np.concatenate([starts[:1], starts[1:][keep]])
        ends = np.concatenate([ends[:-1][keep], ends[-1:]])
    return list(zip(starts.tolist(), ends.tolist()))


def fmt_time(s: float) -> str:
    s = max(0, int(round(s)))
    return f"{s // 60:02d}:{s % 60:02d}"


def _mask(x, n: int) -> np.ndarray:
    """A bool mask of length n from a mask, or from sample indices (list, set, dict keys)."""
    if x is None:
        return np.zeros(n, bool)
    if isinstance(x, np.ndarray) and x.dtype == bool:
        if len(x) != n:
            raise ValueError(f"mask has {len(x)} samples, expected {n}")
        return x.copy()
    m = np.zeros(n, bool)
    idx = np.fromiter((int(i) for i in x), int)
    m[idx[(idx >= 0) & (idx < n)]] = True
    return m


class Timeline(QWidget):
    """The playback track. set_data(...) after Generate, set_current(i) while playing."""

    seek = Signal(int)

    TRACK_H, PAD = 14, 5

    def __init__(self, parent=None, merge_gap: int = 3):
        super().__init__(parent)
        self.merge_gap = merge_gap
        self.colours = dict(DEFAULT_COLOURS)
        self.n, self.current = 0, 0
        self.spray = np.zeros(0, bool)
        self.masks = {k: np.zeros(0, bool) for k in KINDS}
        self.problem_spans = {k: [] for k in KINDS}
        self.times = None
        self.setMouseTracking(False)
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        fm = QFontMetrics(self.font())
        self.setFixedHeight(self.TRACK_H + fm.height() + 2 * self.PAD + 4)
        self.setMinimumWidth(200)

    # --- data ------------------------------------------------------------------
    def set_data(self, n: int, spray=None, unreachable=None, collision=None, steep=None, times=None):
        """`spray`, `unreachable`, `steep`: bool per sample; `collision`: bool mask or
        sample indices (e.g. scene.collisions); `times`: cumulative seconds per sample."""
        self.n = int(n)
        self.spray = _mask(spray, self.n)
        self.masks = {"unreachable": _mask(unreachable, self.n), "collision": _mask(collision, self.n),
                      "steep": _mask(steep, self.n)}
        self.problem_spans = {k: spans(m, self.merge_gap if k == "collision" else 0)
                              for k, m in self.masks.items()}
        self.times = None if times is None else np.asarray(times, float)
        if self.times is not None and len(self.times) != self.n:
            raise ValueError(f"times has {len(self.times)} samples, expected {self.n}")
        self.current = min(self.current, max(self.n - 1, 0))
        self.update()

    def set_collisions(self, collision):
        """Update only the collision marks (when the background check finishes)."""
        self.masks["collision"] = _mask(collision, self.n)
        self.problem_spans["collision"] = spans(self.masks["collision"], self.merge_gap)
        self.update()

    def set_current(self, i: int):
        i = int(np.clip(i, 0, max(self.n - 1, 0)))
        if i != self.current:
            self.current = i
            self.update()

    def set_colours(self, colours: dict):
        """Keys: accent, bad, crash, steep, track, cursor, text (any subset)."""
        self.colours.update(colours)
        self.update()

    def time_text(self) -> str:
        if not self.n:
            return "--:-- / --:--"
        if self.times is None:
            return f"point {self.current} / {self.n - 1}"
        return f"{fmt_time(self.times[self.current])} / {fmt_time(self.times[-1])}"

    # --- problem navigation -----------------------------------------------------
    def _starts(self, kind) -> list[int]:
        kinds = KINDS if kind in (None, "any") else (kind,)
        return sorted(s for k in kinds for s, _ in self.problem_spans[k])

    def next_problem(self, kind=None, from_i: int | None = None):
        """Start of the next problem span after `from_i` (default: the current sample).
        `kind`: unreachable | collision | steep | None/"any". None if there is none."""
        from_i = self.current if from_i is None else from_i
        return next((s for s in self._starts(kind) if s > from_i), None)

    def prev_problem(self, kind=None, from_i: int | None = None):
        """Start of the previous problem span before `from_i` (inside a span: its start)."""
        from_i = self.current if from_i is None else from_i
        return next((s for s in reversed(self._starts(kind)) if s < from_i), None)

    # --- geometry and painting -----------------------------------------------------
    def _track_rect(self) -> QRectF:
        return QRectF(self.PAD, self.PAD, max(self.width() - 2 * self.PAD, 1), self.TRACK_H)

    def index_at(self, x: float) -> int:
        r = self._track_rect()
        if self.n <= 1:
            return 0
        f = (x - r.left()) / r.width()
        return int(np.clip(round(f * (self.n - 1)), 0, self.n - 1))

    def _x(self, i: float) -> float:
        r = self._track_rect()
        return r.left() + r.width() * (i / max(self.n - 1, 1))

    def _col_spans(self, mask, width: int) -> list[tuple[int, int]]:
        """Mask -> spans of pixel columns, so 10k samples cost one numpy pass."""
        idx = np.flatnonzero(mask)
        if not len(idx):
            return []
        cols = np.zeros(width, bool)
        cols[np.minimum((idx * width) // max(self.n, 1), width - 1)] = True
        return spans(cols)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        c = {k: QColor(v) for k, v in self.colours.items()}
        r = self._track_rect()
        p.setPen(Qt.NoPen)
        rad = r.height() / 2
        # glass capsule track: soft fill, hairline rim
        p.setBrush(c["track"])
        p.drawRoundedRect(r, rad, rad)
        w = int(r.width())
        if self.n:
            capsule = QPainterPath()
            capsule.addRoundedRect(r, rad, rad)
            p.save()
            p.setClipPath(capsule)
            spray = QColor(c["accent"])
            spray.setAlphaF(0.4)
            layers = [(self.spray, spray, 1)]
            # gun too steep under collisions under unreachable: the worst problem is drawn on top;
            # problem marks are at least 4 px wide so a single point stays visible
            layers += [(self.masks[k], c[KIND_COLOUR[k]], 4) for k in ("steep", "collision", "unreachable")]
            for mask, colour, min_w in layers:
                p.setBrush(colour)
                for a, b in self._col_spans(mask, w):
                    width = max(b - a, min_w)
                    left = min(max(r.left() + (a + b) / 2 - width / 2, r.left()), r.right() - width)
                    if min_w > 1:
                        p.drawRoundedRect(QRectF(left, r.top() + 2, width, r.height() - 4), width / 2 if width < 8 else 3, 3)
                    else:
                        p.drawRect(QRectF(left, r.top(), width, r.height()))
            p.restore()
        rim = QColor(c["text"])
        rim.setAlphaF(0.25)
        p.setPen(rim)
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(r.adjusted(0.5, 0.5, -0.5, -0.5), rad, rad)
        if self.n:
            # playhead: orange knob with glow
            x, cy, kr = self._x(self.current), r.center().y(), 6.5
            glow = QColor(c["accent"])
            g = QRadialGradient(x, cy, kr * 2)
            glow.setAlphaF(0.55)
            g.setColorAt(0.35, glow)
            glow.setAlphaF(0.0)
            g.setColorAt(1.0, glow)
            p.setPen(Qt.NoPen)
            p.setBrush(g)
            p.drawEllipse(QRectF(x - kr * 2, cy - kr * 2, kr * 4, kr * 4))
            p.setBrush(c["accent"])
            p.setPen(QColor(255, 255, 255, 230))
            p.drawEllipse(QRectF(x - kr, cy - kr, 2 * kr, 2 * kr))

        # legend and time
        fm = QFontMetrics(self.font())
        y = r.bottom() + 2
        h = fm.height()
        x = r.left()
        labels = LABELS
        need = sum(9 + 4 + fm.horizontalAdvance(t) + 12 for t in LABELS.values())
        if need + fm.horizontalAdvance(self.time_text()) + 8 > r.width():
            labels = SHORT_LABELS      # narrow panel: keep the legend clear of the time
        for kind in ("collision", "unreachable", "steep"):
            p.setPen(Qt.NoPen)
            p.setBrush(c[KIND_COLOUR[kind]])
            sw = 9
            p.drawRoundedRect(QRectF(x, y + (h - sw) / 2, sw, sw), 2, 2)
            x += sw + 4
            p.setPen(c["text"])
            p.drawText(QRectF(x, y, fm.horizontalAdvance(labels[kind]) + 2, h),
                       Qt.AlignVCenter | Qt.AlignLeft, labels[kind])
            x += fm.horizontalAdvance(labels[kind]) + 12
        p.setPen(c["text"])
        p.drawText(QRectF(r.left(), y, r.width(), h), Qt.AlignVCenter | Qt.AlignRight, self.time_text())
        p.end()

    # --- mouse -----------------------------------------------------------------
    def _seek_to(self, x):
        if not self.n:
            return
        i = self.index_at(x)
        self.set_current(i)
        self.seek.emit(i)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._seek_to(e.position().x())

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.LeftButton:
            self._seek_to(e.position().x())


class PlaybackControls(QWidget):
    """⏮ ▶/⏸ ⏭, playback speed and Follow tool. No timer: connect the signals."""

    playToggled = Signal(bool)
    stepProblem = Signal(int)          # -1 previous problem, +1 next problem
    speedChanged = Signal(float)
    followToggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.prev_btn = QToolButton(text="⏮")
        self.prev_btn.setToolTip("Previous problem")
        self.prev_btn.clicked.connect(lambda: self.stepProblem.emit(-1))
        self.play_btn = QToolButton()
        self.play_btn.setCheckable(True)
        self.play_btn.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self.play_btn.toggled.connect(self._play_toggled)
        self.next_btn = QToolButton(text="⏭")
        self.next_btn.setToolTip("Next problem")
        self.next_btn.clicked.connect(lambda: self.stepProblem.emit(1))
        self.speed = QComboBox()
        for s in SPEEDS:
            self.speed.addItem(f"{s:g}×", s)
        self.speed.setCurrentIndex(SPEEDS.index(1.0))
        self.speed.setToolTip("Playback speed (1× = program time)")
        self.speed.currentIndexChanged.connect(lambda _: self.speedChanged.emit(self.speed_factor()))
        self.follow = QCheckBox("Follow tool")
        self.follow.setToolTip("Camera follows the nozzle during playback")
        self.follow.toggled.connect(self.followToggled)
        for w in (self.prev_btn, self.play_btn, self.next_btn, self.speed):
            row.addWidget(w)
        row.addWidget(self.follow)
        row.addStretch(1)
        self.apply_glass()
        self._label(False)

    def apply_glass(self, mode: str | None = None):
        """Round glass buttons in the current theme (call again after a theme switch)."""
        t = style.tokens(mode or style.current_mode())
        dark = t['bg'] < '#808080'
        glass = 'rgba(255,255,255,0.10)' if dark else 'rgba(255,255,255,0.65)'
        hover = 'rgba(255,255,255,0.18)' if dark else 'rgba(255,255,255,0.95)'
        rim = 'rgba(255,255,255,0.22)' if dark else 'rgba(0,0,0,0.12)'
        for b in (self.prev_btn, self.next_btn):
            b.setFixedSize(30, 30)
        self.play_btn.setMinimumSize(84, 30)
        self.setStyleSheet(
            f"QToolButton{{background:{glass};border:1px solid {rim};border-radius:15px;color:{t['ink']};padding:0 10px;}}"
            f"QToolButton:hover{{background:{hover};border-color:{t['accent']};}}"
            f"QToolButton:checked{{background:{t['accent']};border-color:{t['accent']};color:#ffffff;}}")

    def _label(self, playing):
        self.play_btn.setText("⏸ Pause" if playing else "▶ Play")

    def _play_toggled(self, playing):
        self._label(playing)
        self.playToggled.emit(playing)

    def set_playing(self, playing: bool):
        """Show the play state without emitting playToggled (e.g. playback reached the end)."""
        self.play_btn.blockSignals(True)
        self.play_btn.setChecked(playing)
        self.play_btn.blockSignals(False)
        self._label(playing)

    def is_playing(self) -> bool:
        return self.play_btn.isChecked()

    def speed_factor(self) -> float:
        return float(self.speed.currentData())
