"""Every test file belongs to a test area (tests/conftest.py), so area runs don't miss it."""

from pathlib import Path

from conftest import AREAS, GUARD, SLOW_FILES, area_of

TESTS = Path(__file__).parent


def test_every_test_file_has_an_area():
    files = [p.name for p in TESTS.glob("test_*.py")]
    missing = [f for f in files if f not in GUARD and not area_of(f)]
    assert not missing, f"add these to an area in tests/conftest.py AREAS: {missing}"


def test_lists_name_real_files():
    names = {p.name for p in TESTS.glob("test_*.py")}
    assert set(GUARD) <= names and set(SLOW_FILES) <= names
    assert all(isinstance(p, list) and p for p in AREAS.values())
