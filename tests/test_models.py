import pytest
from pydantic import ValidationError

from coldspray.models import ProcessConfig, load_process, save_process


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
