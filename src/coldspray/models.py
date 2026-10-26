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


def load_robot(path: Path = CONFIG_DIR / "robot.yaml") -> RobotConfig:
    return RobotConfig.model_validate(yaml.safe_load(path.read_text()))


def load_tool(path: Path = CONFIG_DIR / "tool.yaml") -> ToolConfig:
    return ToolConfig.model_validate(yaml.safe_load(path.read_text()))
