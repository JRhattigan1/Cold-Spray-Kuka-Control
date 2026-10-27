import pytest
from pydantic import ValidationError

from coldspray.models import CellConfig, ProcessConfig, PurgeConfig, field_level, load_cell, load_process, save_process


def test_save_and_reload_process(tmp_path):
    proc = load_process().model_copy(update={"standoff_mm": 42.5, "contour_passes": 3})
    out = tmp_path / "process.yaml"
    save_process(proc, out)
    assert load_process(out) == proc
    assert "# nozzle exit to surface" in out.read_text()


def test_invalid_values_rejected():
    values = load_process().model_dump() | {"step_over_mm": 0}
    with pytest.raises(ValidationError):
        ProcessConfig(**values)


def test_field_levels():
    assert field_level("standoff_mm") == "basic"
    assert field_level("infill_pattern") == "basic"
    assert field_level("turn_mode") == "advanced"
    assert {field_level(n) for n in ProcessConfig.model_fields} == {"basic", "advanced"}


def test_save_and_reload_literal_and_str_fields(tmp_path):
    proc = load_process().model_copy(update={"infill_pattern": "concentric", "turn_mode": "beyond_plane",
                                             "turn_plane": "Plane 2", "surface_coats": 3})
    out = tmp_path / "process.yaml"
    save_process(proc, out)
    assert load_process(out) == proc
    assert "'concentric'" in out.read_text()


def test_bad_literal_rejected():
    with pytest.raises(ValidationError):
        ProcessConfig(**(load_process().model_dump() | {"infill_pattern": "swirl"}))


def test_purge_config_round_trip():
    purge = PurgeConfig(point=(-800.0, 400.0, 2000.0), dwell_s=3.0, every_n_layers=5)
    assert purge.spray_dir == (0.0, -1.0, 0.0) and purge.at_start and purge.at_end
    assert PurgeConfig(**purge.model_dump()) == purge
    cell = load_cell()
    assert cell.purge is None or isinstance(cell.purge, PurgeConfig)
    cell2 = CellConfig(**(cell.model_dump() | {"purge": purge.model_dump()}))
    assert cell2.purge == purge
    with pytest.raises(ValidationError):
        PurgeConfig(point=(0, 0, 0), dwell_s=-1)
