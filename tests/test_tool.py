import numpy as np
import pytest

from coldspray.models import ToolConfig, XYZABC, load_tool
from coldspray.tool import gun_to_flange, nozzle_to_gun, tool_frame
from coldspray.transforms import xyzabc


def test_gun_mount_face_lands_on_flange():
    gun = load_tool().gun
    g2f = gun_to_flange(gun)
    mount = g2f @ [*gun.mount_point, 1]
    assert np.allclose(mount[:3], 0)
    # mounting face normal (-X gun) points back into the robot (-X flange)
    assert np.allclose(g2f[:3, :3] @ [-1, 0, 0], [-1, 0, 0])


def test_tcp_from_gun_geometry():
    tool = load_tool()
    x, y, z, a, b, c = xyzabc(tool_frame(tool))
    assert np.allclose([x, y, z], [138.0, 274.0 + tool.gun.nozzle_length, 0.0])
    assert np.allclose([a, b, c], [90, 0, 0])  # spray direction = flange +Y


def test_nozzle_choice_moves_tcp():
    tool = load_tool()
    tool.gun.nozzle = "central_injector"
    longer = tool.model_copy(deep=True)
    longer.gun.nozzles["central_injector"].mount_x_mm += 50.0
    d = tool_frame(longer)[:3, 3] - tool_frame(tool)[:3, 3]
    assert np.isclose(np.linalg.norm(d), 50.0)


def test_both_nozzles_seat_on_the_same_housing_face():
    # same barrel and clamp housing, only the injector differs -> same TCP
    tool = load_tool()
    radial = tool.model_copy(deep=True)
    tool.gun.nozzle, radial.gun.nozzle = "central_injector", "radial_injector"
    assert np.allclose(tool_frame(tool), tool_frame(radial))


@pytest.mark.parametrize("key", ["central_injector", "radial_injector"])
def test_nozzle_outlet_lands_on_tcp(key):
    tool = load_tool()
    tool.gun.nozzle = key
    n = tool.gun.current_nozzle
    outlet_in_gun = nozzle_to_gun(n) @ [n.exit_x_mm, 0, 0, 1]
    assert np.allclose(outlet_in_gun[:3], [0, n.length, 0])           # on the spray axis
    assert np.allclose(nozzle_to_gun(n)[:3, :3] @ [-1, 0, 0], [0, 1, 0])  # spraying along gun +Y
    outlet_in_flange = gun_to_flange(tool.gun) @ outlet_in_gun
    assert np.allclose(outlet_in_flange[:3], tool_frame(tool)[:3, 3])


def test_explicit_tcp_overrides_gun():
    tool = load_tool().model_copy(update={"tcp": XYZABC(x=1, y=2, z=3)})
    assert np.allclose(tool_frame(tool)[:3, 3], [1, 2, 3])


def test_no_tool_is_flange():
    assert np.allclose(tool_frame(None), np.eye(4))
    assert np.allclose(tool_frame(ToolConfig(name="x", tcp=XYZABC())), np.eye(4))
