"""Viewer labels hold a real degree sign; Toolpath.sample honours configured axis defaults."""
from pathlib import Path

import numpy as np

import coldspray.sim.viewer as viewer
from coldspray.toolpath.model import Move, Toolpath


def test_viewer_source_has_no_mojibake():
    text = Path(viewer.__file__).read_text(encoding="utf-8")
    assert "Â" not in text
    assert "gun {round(self.gun_down[i])}° down" in text


def _path():
    p0, p1 = np.eye(4), np.eye(4)
    p1[0, 3] = 10
    return Toolpath("t", [Move(p0, 100, False, "travel", {}), Move(p1, 100, True, "spray", {"E2": 90.0})])


def test_sample_axis_default():
    path = _path()
    assert path.sample().axes["E2"][0] == 90.0                       # old behaviour without defaults
    s = path.sample(defaults={"E2": 30.0})
    assert s.axes["E2"][0] == 30.0 and s.axes["E2"][-1] == 90.0
