import shutil

import pytest

from coldspray.config_io import save_purge
from coldspray.models import CONFIG_DIR, PurgeConfig, load_cell


@pytest.fixture
def cell_copy(tmp_path):
    p = tmp_path / "cell.yaml"
    shutil.copyfile(CONFIG_DIR / "cell.yaml", p)
    return p


def _lines(p):
    with open(p, newline="") as f:
        return f.read().splitlines(keepends=True)


def _non_purge(lines):
    """Lines outside the purge block (live or commented)."""
    out, skip = [], False
    for line in lines:
        s = line.rstrip("\r\n")
        if s.startswith("purge:") or s.replace(" ", "") == "#purge:":
            skip = True
            continue
        if skip and (s.startswith((" ", "\t")) or s.startswith("#  ")):
            continue
        skip = False
        out.append(line)
    return out


def test_save_purge_round_trip_keeps_other_lines(cell_copy):
    before = _lines(cell_copy)
    purge = PurgeConfig(point=(-900.0, 410.5, 1990.0), spray_dir=(1.0, 0.0, 0.0), dwell_s=7.5,
                        at_end=False, every_n_layers=3, approach_mm=50.0)
    save_purge(purge, cell_copy)
    assert load_cell(cell_copy).purge == purge
    after = _lines(cell_copy)
    assert _non_purge(after) == _non_purge(before)

    # saving again replaces the live block in place (idempotent)
    purge2 = purge.model_copy(update=dict(enabled=False, dwell_s=1.0))
    save_purge(purge2, cell_copy)
    assert load_cell(cell_copy).purge == purge2
    assert len(_lines(cell_copy)) == len(after)
    assert _non_purge(_lines(cell_copy)) == _non_purge(before)


def test_save_purge_appends_when_no_block(tmp_path, cell_copy):
    text = "".join(l for l in _non_purge(_lines(cell_copy)))
    p = tmp_path / "bare.yaml"
    p.write_bytes(text.encode())
    purge = PurgeConfig(point=(1.0, 2.0, 3.0))
    save_purge(purge, p)
    assert load_cell(p).purge == purge
    assert p.read_bytes().startswith(text.encode())
