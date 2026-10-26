"""Spray gun geometry: where the gun CAD sits on the flange, and the TCP."""

import numpy as np

from coldspray.models import GunConfig, NozzleConfig, ToolConfig
from coldspray.transforms import frame

# Tool axes expressed in gun-CAD coordinates: X = spray (+Y gun), Y = -X gun, Z = +Z gun
_TOOL_IN_GUN = frame(a=90.0)


def nozzle_to_gun(nozzle: NozzleConfig) -> np.ndarray:
    """Maps nozzle-CAD coordinates into gun-CAD coordinates: the nozzle's spray
    axis (-X) becomes gun +Y and its mounting face lands on the gun origin."""
    return frame(y=nozzle.mount_x_mm, a=-90.0)


def gun_to_flange(gun: GunConfig) -> np.ndarray:
    """Maps gun-CAD coordinates into the KUKA flange frame.

    The gun's mounting face (normal -X in gun CAD) sits on the flange face, so
    gun +X lines up with flange +X; `clock_deg` turns the gun about that axis.
    """
    shift = np.eye(4)
    shift[:3, 3] = -np.asarray(gun.mount_point)
    return frame(c=gun.clock_deg) @ shift


def tool_frame(tool: ToolConfig | None) -> np.ndarray:
    """TCP frame relative to the flange (identity if there is no tool)."""
    if tool is None:
        return np.eye(4)
    if tool.tcp is not None:
        return frame(**tool.tcp.model_dump())
    if tool.gun is None:
        raise ValueError("tool needs either `tcp` or `gun`")
    tip = frame(y=tool.gun.nozzle_length) @ _TOOL_IN_GUN
    return gun_to_flange(tool.gun) @ tip
