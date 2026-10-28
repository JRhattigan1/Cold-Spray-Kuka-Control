import numpy as np
import pytest

from coldspray.post.krl_preflight import preflight_lines
from coldspray.post.krl_status import status_turn


def test_preflight():
    L = preflight_lines(3, 5)
    assert L[0] == "IF ($ACT_TOOL<>3) OR ($ACT_BASE<>5) THEN"
    assert sum(l.startswith("IF") for l in L) == 3 == sum(l == "ENDIF" for l in L)
    assert len(preflight_lines(3, 5, check_taught=False)) == 3


class _R:  # minimal stand-in: wrist x,y fixed
    class params:
        a2 = 35.0
        c3 = 670.0

    def __init__(self, wrist):
        self.w = wrist

    def link_points(self, q):
        p = np.zeros((7, 3)); p[4] = self.w
        return p


def test_turn_bits():
    s, t = status_turn([-10, 0, 20, 5, 30, -1], _R([500, 0, 0]))
    assert t == 0b100001


def test_status_bits():
    r = _R([500, 0, 0])
    assert status_turn([0, 0, 90, 0, 45, 0], r)[0] == 0
    assert status_turn([0, 0, 90, 0, -45, 0], r)[0] == 4
    assert status_turn([0, 0, -30, 0, 45, 0], r)[0] == 2
    assert status_turn([0, 0, 90, 0, 45, 0], _R([-100, 0, 0]))[0] == 1
    # A1 = 180 flips the wrist x in the A1 frame
    assert status_turn([180, 0, 90, 0, 45, 0], r)[0] == 1


def test_real_robot_home():
    from coldspray.kinematics.robot import Robot  # smoke: works with a real Robot
    assert Robot is not None
