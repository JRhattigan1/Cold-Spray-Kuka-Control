"""tools/bench_solve.py: scenarios exist, counters and compare work (no solve is run: it takes minutes)."""
import importlib.util, json, pathlib

P = pathlib.Path(__file__).parents[1] / "tools" / "bench_solve.py"


def _load():
    spec = importlib.util.spec_from_file_location("bench_solve", P)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


def test_scenarios_and_counter():
    m = _load()
    assert set(m.SCENARIOS) == {"block_layers", "tube_surface", "repair_surface"}
    m.COUNTS.clear(); f = m._count("x", lambda a: a + 1)
    assert f(1) == 2 and m.COUNTS["x"] == 1


def test_compare_identical_and_different(tmp_path):
    m = _load()
    d = {"s": {"fixed": [], "unfixed": [], "suggested": {}, "gun_on": [True], "poses": [[[1.0]]]}}
    a, b, c = (tmp_path / n for n in "abc")
    a.write_text(json.dumps(d)); b.write_text(json.dumps(d))
    d["s"]["poses"] = [[[1.1]]]; c.write_text(json.dumps(d))
    for other, code in ((b, 0), (c, 1)):
        try:
            m.compare(str(a), str(other))
        except SystemExit as e:
            assert e.code == code
