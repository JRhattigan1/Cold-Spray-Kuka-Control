"""Frame helpers using KUKA conventions: mm, degrees, ABC = Z-Y'-X'' Euler."""

import warnings

import numpy as np
from scipy.spatial.transform import Rotation


def frame(x=0.0, y=0.0, z=0.0, a=0.0, b=0.0, c=0.0) -> np.ndarray:
    """4x4 frame from KUKA X, Y, Z, A, B, C."""
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("ZYX", [a, b, c], degrees=True).as_matrix()
    t[:3, 3] = [x, y, z]
    return t


def xyzabc(t: np.ndarray) -> np.ndarray:
    """KUKA X, Y, Z, A, B, C of a 4x4 frame.

    At B = ±90° (e.g. gun pointing straight down) A and C are not unique;
    C is set to 0, which describes the same frame.
    """
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock")
        a, b, c = Rotation.from_matrix(t[:3, :3]).as_euler("ZYX", degrees=True)
    return np.array([*t[:3, 3], a, b, c])
