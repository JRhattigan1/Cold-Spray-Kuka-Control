"""Config schemas, loaded from the YAML files in coldspray/config."""

from pathlib import Path

import yaml
from pydantic import BaseModel, Field

CONFIG_DIR = Path(__file__).parent / "config"
PROJECT_ROOT = Path(__file__).parents[2]


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


class GunConfig(BaseModel):
    step_file: str
    mount_point: tuple[float, float, float]
    clock_deg: float = 0.0
    nozzle: str
    nozzle_lengths_mm: dict[str, float]

    @property
    def nozzle_length(self) -> float:
        return self.nozzle_lengths_mm[self.nozzle]


class ToolConfig(BaseModel):
    name: str
    verified: bool = False
    tcp: XYZABC | None = None
    gun: GunConfig | None = None


class GantryConfig(BaseModel):
    axis: str = "E1"
    direction: tuple[float, float, float]
    limits_mm: tuple[float, float]
    position_mm: float = 0.0


class CellConfig(BaseModel):
    cad_cache: str | None = None
    robroot: XYZABC = XYZABC()
    robot_cad_pose_deg: list[float] = [0, -90, 90, 0, 0, 0]
    gantry: GantryConfig | None = None
    base_number: int
    base: XYZABC
    tool_number: int
    gun_output: int
    ptp_velocity_pct: int
    home_deg: list[float]


class ProcessConfig(BaseModel):
    """Cold spray process parameters. Descriptions double as GUI tooltips and
    as the comments written by save_process()."""

    standoff_mm: float = Field(gt=0, description="nozzle exit to surface")
    traverse_speed_mm_s: float = Field(gt=0, description="gun speed while spraying")
    step_over_mm: float = Field(gt=0, description="spacing between neighbouring passes")
    overrun_mm: float = Field(ge=0, description="plate raster: pass extends this far past the edge")
    approach_height_mm: float = Field(ge=0, description="clearance above the start / end point")
    rapid_speed_mm_s: float = Field(gt=0, description="LIN speed for non-spray moves")
    spray_settle_s: float = Field(ge=0, description="dwell after switching spray on, before moving")
    approx_distance_mm: float = Field(ge=0, description="$APO.CDIS corner blending")
    layer_thickness_mm: float = Field(gt=0, description="deposit height from one full layer of passes")
    track_width_mm: float = Field(gt=0, description="width of a single spray track (nozzle footprint)")
    raster_angle_step_deg: float = Field(description="raster direction rotates by this each layer")
    contour_passes: int = Field(ge=0, description="perimeter passes around each layer before infill")
    jump_height_mm: float = Field(ge=0, description="long moves between areas lift the gun this much (spray stays on)")


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


def save_process(proc: ProcessConfig, path=None):
    """Write process parameters as YAML, one commented line per field."""
    lines = ["# Cold spray process parameters (saved by coldspray)"]
    for name, info in ProcessConfig.model_fields.items():
        lines.append(f"{name + ':':28s}{getattr(proc, name)!r:<10} # {info.description}")
    Path(path or CONFIG_DIR / "process.yaml").write_text("\n".join(lines) + "\n")
