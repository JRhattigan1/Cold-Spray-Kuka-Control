"""B2: every infill pattern plans a toolpath on the built-in parts."""
from types import SimpleNamespace

import pytest

from coldspray.models import load_process
from coldspray.pipeline import load_part, plan_part
from coldspray.planner.patterns import PATTERNS


def _proc(pattern):
    proc = load_process()
    if "infill_pattern" in type(proc).model_fields:
        return proc.model_copy(update={"infill_pattern": pattern})
    # infill_pattern not on this branch yet: wrap so plan_layers sees it via getattr
    return SimpleNamespace(**{k: getattr(proc, k) for k in type(proc).model_fields}, infill_pattern=pattern)


@pytest.fixture(scope="module", params=["test_block", "test_repair"])
def part(request):
    return load_part(request.param)


@pytest.mark.parametrize("pattern", sorted(PATTERNS))
def test_every_pattern_plans_on_builtin_parts(part, pattern):
    _, path = plan_part(part, _proc(pattern))
    tags = {m.tag for m in path.moves}
    assert any(m.gun_on for m in path.moves)
    assert "infill" in tags and "contour" in tags


def test_patterns_change_the_infill(part):
    counts = {p: sum(m.tag == "infill" for m in plan_part(part, _proc(p))[1].moves) for p in PATTERNS}
    assert len(set(counts.values())) > 1, counts


def test_gun_stays_on_between_layers_without_a_jump():
    part = load_part("test_block")
    cell = None
    for h, stays in ((0.0, True), (20.0, False)):
        proc = load_process().model_copy(update={"jump_height_mm": h})
        moves = plan_part(part, proc, "layers")[1].moves
        on = [m.gun_on for m in moves]
        first, last = on.index(True), len(on) - 1 - on[::-1].index(True)
        gaps = [i for i in range(first, last) if not on[i]]
        assert (not gaps) == stays
        assert not moves[-1].gun_on
