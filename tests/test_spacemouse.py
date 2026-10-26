import struct

import numpy as np

from coldspray.sim.spacemouse import FULL_SCALE, R_SIGNS, T_SIGNS, SpaceMouse


class FakeDevice:
    def __init__(self, reports):
        self.reports = list(reports)

    def read(self, _size):
        return self.reports.pop(0) if self.reports else []


def report(kind, *values):
    return [kind, *struct.pack("<" + "h" * len(values), *values)]


def test_separate_translation_rotation_reports():
    sm = SpaceMouse(FakeDevice([report(1, 350, -175, 0), report(2, 0, 0, 350)]), "fake")
    assert sm.poll() == []
    t, r = sm.motion()
    assert np.allclose(t, [T_SIGNS[0], -0.5 * T_SIGNS[1], 0.0])
    assert np.allclose(r, [0.0, 0.0, R_SIGNS[2]])


def test_six_axes_in_one_report_and_deadzone():
    sm = SpaceMouse(FakeDevice([report(1, 10, 0, 0, 0, 175, 0)]), "fake")
    sm.poll()
    t, r = sm.motion()
    assert not t.any()                                     # 10 / 350 is inside the deadzone
    assert np.isclose(r[1], 175 / FULL_SCALE * R_SIGNS[1])


def test_button_presses_reported_once():
    sm = SpaceMouse(FakeDevice([[3, 1, 0], [3, 3, 0], [3, 2, 0], [3, 0, 0]]), "fake")
    assert sm.poll() == [0, 1]                             # left, then right pressed
    assert sm.buttons == 0
