"""Config schemas, loaded from the YAML files in coldspray/config."""

from pathlib import Path

import yaml
from pydantic import BaseModel

CONFIG_DIR = Path(__file__).parent / "config"


class OPWConfig(BaseModel):
    a1: float
    a2: float
    b: float
    c1: float
    c2: float
    c3: float
    c4: float
    offsets_deg: list[float]
    signs: list[int]


class RobotConfig(BaseModel):
    name: str
    opw: OPWConfig
    joint_limits_deg: list[tuple[float, float]]
    max_speed_deg_s: list[float]


class XYZABC(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0


class ToolConfig(BaseModel):
    name: str
    verified: bool = False
    tcp: XYZABC


class CellConfig(BaseModel):
    base_number: int
    base: XYZABC
    tool_number: int
    gun_output: int
    ptp_velocity_pct: int
    home_deg: list[float]


class ProcessConfig(BaseModel):
    standoff_mm: float
    traverse_speed_mm_s: float
    step_over_mm: float
    overrun_mm: float
    approach_height_mm: float
    rapid_speed_mm_s: float
    spray_settle_s: float
    approx_distance_mm: float
    layer_thickness_mm: float
    track_width_mm: float
    raster_angle_step_deg: float
    contour_passes: int


def _load(model, name, path):
    return model.model_validate(yaml.safe_load(Path(path or CONFIG_DIR / name).read_text()))


def load_robot(path=None) -> RobotConfig:
    return _load(RobotConfig, "robot.yaml", path)


def load_tool(path=None) -> ToolConfig:
    return _load(ToolConfig, "tool.yaml", path)


def load_cell(path=None) -> CellConfig:
    return _load(CellConfig, "cell.yaml", path)


def load_process(path=None) -> ProcessConfig:
    return _load(ProcessConfig, "process.yaml", path)
