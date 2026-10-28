import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from types import SimpleNamespace

import numpy as np
import pytest
from PySide6.QtWidgets import QApplication

from coldspray.gui.checks import (
    Check, CheckList, confirm_text, export_checks, open_items, pair_summary, simulate_checks,
)
from coldspray.models import load_cell, load_tool
from coldspray.sim.collision import Collision


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def cell():
    return load_cell()


@pytest.fixture
def tool():
    return load_tool()


def fake_scene(n=50, unreachable=(), steep=(), gun_max=30.0):
    reach = np.ones(n, bool)
    reach[list(unreachable)] = False
    st = np.zeros(n, bool)
    st[list(steep)] = True
    down = np.full(n, gun_max / 2)
    down[0] = gun_max
    down[list(steep)] = 60.0
    return SimpleNamespace(n_points=n, reachable=reach, steep=st, gun_down=down, collisions={},
                           gun_on=np.ones(n, bool))


def by_key(checks):
    return {c.key: c for c in checks}


def test_all_ok(cell):
    c = by_key(simulate_checks(fake_scene(), cell, {}))
    assert [c[k].state for k in ("reach", "gun", "collisions")] == ["ok", "ok", "ok"]
    assert c["gun"].title == f"Gun ≤ {cell.gun_max_down_deg:.0f}° down (max 30°)"
    assert all(x.sample is None for x in c.values())


def test_problems_and_first_samples(cell):
    scene = fake_scene(unreachable=(7, 3, 20), steep=(12, 30))
    hits = [Collision(21, {("A2", "Filtration unit")}), Collision(18, {("A2", "Filtration unit"), ("A3", "Wall")}),
            Collision(24, {("A3", "Filtration unit")})]
    c = by_key(simulate_checks(scene, cell, hits))
    assert (c["reach"].state, c["reach"].sample, c["reach"].count) == ("bad", 3, 3)
    assert (c["gun"].state, c["gun"].sample, c["gun"].count) == ("warn", 12, 2)
    assert "60°" in c["gun"].detail
    assert (c["collisions"].state, c["collisions"].sample, c["collisions"].count) == ("crash", 18, 3)
    assert c["collisions"].detail.startswith("A2 / Filtration unit")
    # the same result as scene.collisions dict
    d = by_key(simulate_checks(scene, cell, {h.sample: h.pairs for h in hits}))
    assert d["collisions"] == c["collisions"]


def test_collision_states(cell):
    scene = fake_scene()
    assert by_key(simulate_checks(scene, cell, "running"))["collisions"].state == "todo"
    assert by_key(simulate_checks(scene, cell, "off"))["collisions"].state == "warn"
    failed = by_key(simulate_checks(scene, cell, "failed: out of memory"))["collisions"]
    assert failed.state == "warn" and failed.detail == "out of memory"
    with pytest.raises(ValueError):
        simulate_checks(scene, cell, "maybe")


def test_no_toolpath(cell):
    checks = simulate_checks(SimpleNamespace(n_points=0), cell, "off")
    assert [c.key for c in checks] == ["toolpath"]


def test_pair_summary():
    hits = {1: {("a", "b"), ("c", "d")}, 2: {("a", "b")}, 3: {("e", "f"), ("g", "h")}}
    assert pair_summary(hits, top=2) == "a / b, c / d and 2 more"


def test_export_includes_config_items(cell, tool):
    checks = export_checks(fake_scene(), cell, tool, {})
    c = by_key(checks)
    assert {"reach", "gun", "collisions", "tcp", "coupling", "base", "e1_zero", "table_dirs"} <= set(c)
    assert c["tcp"].state == ("ok" if tool.verified else "todo")
    assert ("coupled" if cell.turntable.coupled else "fixed base") in c["coupling"].detail
    assert f"BASE_DATA[{cell.base_number}]" in c["base"].title
    verified = by_key(export_checks(fake_scene(), cell, tool.model_copy(update={"verified": True}), {}))
    assert verified["tcp"].state == "ok"
    no_axes = by_key(export_checks(fake_scene(), cell.model_copy(update={"gantry": None, "turntable": None}),
                                   tool, {}))
    assert not {"coupling", "e1_zero", "table_dirs"} & set(no_axes)


def test_confirm_text_lists_every_open_item(cell, tool):
    checks = export_checks(fake_scene(unreachable=(1,)), cell, tool, "running")
    items = open_items(checks)
    assert all(c.state != "ok" for c in items) and "gun" not in {c.key for c in items}
    text = confirm_text(items)
    for c in items:
        assert c.title in text
    assert text.startswith(f"{len(items)} open items")
    assert confirm_text([]) == ""


def test_check_state_validated():
    with pytest.raises(ValueError):
        Check("x", "fine", "t")
    assert Check("x", "crash", "t", count=12).chip_text == "12"
    assert Check("x", "warn", "t").chip_text == "!"


def test_checklist_widget(app, cell, tool):
    w = CheckList()
    got = []
    w.goTo.connect(got.append)
    checks = export_checks(fake_scene(unreachable=(5,)), cell, tool, {9: {("A2", "Wall")}})
    w.set_checks(checks)
    assert len(w.rows) == len(checks)
    reach = next(r for r in w.rows if r["check"].key == "reach")
    assert reach["chip"].property("kind") == "bad" and reach["chip"].text() == "1"
    assert reach["detail"].property("note") == "true"
    reach["goto"].click()
    next(r for r in w.rows if r["check"].key == "collisions")["goto"].click()
    assert got == [5, 9]
    assert next(r for r in w.rows if r["check"].key == "gun")["goto"] is None
    w.set_checks(checks[:2])                         # rebuilding replaces the rows
    assert len(w.rows) == 2 and w.open_count() == 1
    assert not w.grab().isNull()


def test_flip_row(cell):
    sc = fake_scene(6)
    j = np.zeros((6, 6))
    j[3:, 0] = 90.0                                   # jump between samples 2 and 3
    sc.joints = j
    sc.samples = SimpleNamespace(move_index=np.array([0, 0, 0, 0, 0, 1]))
    sc.move_kind = ["LIN"] * 5 + ["LIN"]
    c = by_key(simulate_checks(sc, cell, "off"))["flip"]
    assert (c.state, c.sample, c.count) == ("bad", 3, 1)
    sc.move_kind = ["PTP"] * 6
    assert by_key(simulate_checks(sc, cell, "off"))["flip"].state == "ok"
    sc.move_kind = ["LIN"] * 6
    sc.samples = SimpleNamespace(move_index=np.array([0, 0, 0, 1, 1, 1]))
    assert by_key(simulate_checks(sc, cell, "off"))["flip"].state == "ok"


def test_cable_windup_rows(cell):
    """A4 / A6 turned past cell.cable_windup_deg from home: one warning row per axis, at the first sample."""
    sc = fake_scene(6)
    sc.joints = np.zeros((6, 6))
    sc.samples = SimpleNamespace(move_index=np.zeros(6, int))
    sc.move_kind = ["PTP"] * 6
    assert not [k for k in by_key(simulate_checks(sc, cell, "off")) if k.startswith("windup")]
    lim = cell.cable_windup_deg["A6"]
    sc.joints[4:, 5] = -(lim + 10.0)
    sc.joints[5, 3] = np.nan                          # unsolved samples are not counted
    c = by_key(simulate_checks(sc, cell, "off"))
    assert "windup_A4" not in c
    assert (c["windup_A6"].state, c["windup_A6"].sample, c["windup_A6"].count) == ("warn", 4, 2)
