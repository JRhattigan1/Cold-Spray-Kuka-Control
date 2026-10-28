"""KUKA Status (S) and Turn (T) for a 6-axis arm from joint angles (deg).

T: bit i set when axis A(i+1) < 0 (i = 0..5).                          [standard]
S bit 2: set when A5 < 0.                                              [standard]
S bit 0 (overhead): set when the wrist centre is behind the A1 axis, x in the A1 frame < overhead_x_mm.
    UNVERIFIED: KUKA's exact reference (0 vs the A2 offset a1) is robot-specific; default 0.
S bit 1 (elbow): set when A3 < elbow_deg.
    UNVERIFIED: the boundary depends on a2/c3 (near the stretched arm); pass it per robot.
Check against $POS_ACT of the real arm (E6POS S/T) before using for output.
"""
import numpy as np


def status_turn(joints_deg, robot, overhead_x_mm: float = 0.0, elbow_deg: float | None = None) -> tuple[int, int]:
    q = np.asarray(joints_deg, dtype=float)
    t = sum(1 << i for i in range(6) if q[i] < 0)
    if elbow_deg is None:     # UNVERIFIED default: stretched-arm angle atan(a2/c3)
        p = robot.params
        elbow_deg = float(np.degrees(np.arctan2(p.a2, p.c3)))
    wrist = robot.link_points(q)[4]
    a1 = np.radians(q[0])
    x_a1 = wrist[0] * np.cos(a1) + wrist[1] * np.sin(a1)
    s = (1 if x_a1 < overhead_x_mm else 0) | (2 if q[2] < elbow_deg else 0) | (4 if q[4] < 0 else 0)
    return s, t
