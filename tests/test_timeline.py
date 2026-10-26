import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QMouseEvent
from PySide6.QtWidgets import QApplication

from coldspray.gui.steps.toolpath import estimate_time_s
from coldspray.gui.timeline import PlaybackControls, Timeline, fmt_time, sample_times, spans
from coldspray.models import load_process
from coldspray.toolpath.model import Move, Toolpath
from coldspray.transforms import frame


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def test_spans_merge():
    m = np.zeros(20, bool)
    m[[2, 3, 4, 8, 11, 19]] = True
    assert spans(m) == [(2, 5), (8, 9), (11, 12), (19, 20)]
    assert spans(m, gap=2) == [(2, 5), (8, 12), (19, 20)]
    assert spans(m, gap=10) == [(2, 20)]
    assert spans(np.zeros(5, bool)) == []
    assert spans(np.ones(5, bool)) == [(0, 5)]


def test_spans_fast_for_many_samples():
    m = np.zeros(10000, bool)
    m[::3] = True                                 # collisions checked every 3rd sample
    assert spans(m, gap=2) == [(0, 10000)]       # 9999 is a multiple of 3
    assert len(spans(m)) == 3334


def _timeline(app):
    t = Timeline()
    t.resize(600, t.height())
    n = 100
    unreach = np.zeros(n, bool)
    unreach[[40, 41]] = True
    steep = np.zeros(n, bool)
    steep[70:75] = True
    t.set_data(n, spray=np.arange(n) < 50, unreachable=unreach, collision={10: set(), 13: set(), 16: set(), 60: set()},
               steep=steep, times=np.linspace(0, 125, n))
    return t


def test_next_prev_problem(app):
    t = _timeline(app)
    assert t.problem_spans["collision"] == [(10, 17), (60, 61)]   # every-3rd hits merged
    assert t.next_problem(None, 0) == 10
    assert t.next_problem(None, 10) == 40
    assert t.next_problem("collision", 10) == 60
    assert t.next_problem("steep", 0) == 70
    assert t.next_problem(None, 70) is None
    assert t.prev_problem(None, 72) == 70
    assert t.prev_problem(None, 70) == 60
    assert t.prev_problem("unreachable", 99) == 40
    assert t.prev_problem(None, 10) is None
    t.set_current(45)
    assert t.next_problem() == 60 and t.prev_problem() == 40


def test_time_label_and_paint(app):
    t = _timeline(app)
    t.set_current(99)
    assert t.time_text() == "02:05 / 02:05"
    assert fmt_time(61.4) == "01:01"
    assert not t.grab().isNull()
    t.set_data(5)
    assert t.time_text() == "point 4 / 4"


def test_seek_on_click(app):
    t = _timeline(app)
    got = []
    t.seek.connect(got.append)
    r = t._track_rect()
    x = r.left() + r.width() * 0.5
    pos = QPointF(x, r.center().y())
    ev = QMouseEvent(QMouseEvent.MouseButtonPress, pos, t.mapToGlobal(pos), Qt.LeftButton, Qt.LeftButton,
                     Qt.NoModifier)
    QApplication.sendEvent(t, ev)
    assert got == [50] and t.current == 50
    pos = QPointF(r.right() + 50, r.center().y())                 # dragged past the end clamps
    ev = QMouseEvent(QMouseEvent.MouseMove, pos, t.mapToGlobal(pos), Qt.NoButton, Qt.LeftButton, Qt.NoModifier)
    QApplication.sendEvent(t, ev)
    assert got == [50, 99]


def test_wrong_length_rejected(app):
    with pytest.raises(ValueError):
        Timeline().set_data(10, spray=np.zeros(5, bool))


def test_sample_times_match_estimate():
    proc = load_process()
    p = lambda x, y: frame(x, y, 50)                             # noqa: E731
    path = Toolpath("t", [
        Move(p(0, 0), 200),
        Move(p(100, 0), 200),                                    # travel 100 mm
        Move(p(100, 50), 20, gun_on=True),                       # spray on: settle + 50 mm
        Move(p(0, 50), 20, gun_on=True),                         # 100 mm
        Move(p(0, 50), 20, gun_on=True, axes={"E5": 90.0}),      # table turn only: no time
        Move(p(0, 0), 200),                                      # spray off: travel 50 mm
    ])
    s = path.sample(5.0, 2.0)
    t = sample_times(path, s, proc)
    assert len(t) == len(s) and t[0] == 0
    assert np.all(np.diff(t) >= 0)
    assert t[-1] == pytest.approx(estimate_time_s(path, proc))
    expected = 100 / 200 + proc.spray_settle_s + 50 / 20 + 100 / 20 + 50 / 200
    assert t[-1] == pytest.approx(expected)
    first_spray = int(np.argmax(s.move_index == 2))
    assert t[first_spray] == pytest.approx(100 / 200 + proc.spray_settle_s + (50 / 20) / 10)


def test_playback_controls(app):
    c = PlaybackControls()
    played, steps, speeds, follow = [], [], [], []
    c.playToggled.connect(played.append)
    c.stepProblem.connect(steps.append)
    c.speedChanged.connect(speeds.append)
    c.followToggled.connect(follow.append)
    c.play_btn.click()
    c.play_btn.click()
    c.prev_btn.click()
    c.next_btn.click()
    c.speed.setCurrentIndex(3)
    c.follow.setChecked(True)
    assert played == [True, False] and steps == [-1, 1] and speeds == [5.0] and follow == [True]
    c.set_playing(True)
    assert played == [True, False] and c.is_playing() and "Pause" in c.play_btn.text()
