"""Every module imports: catches a syntax error in code no other test loads (e.g. a GUI step)."""

import importlib
import pkgutil

import pytest

import coldspray

MODULES = sorted(m.name for m in pkgutil.walk_packages(coldspray.__path__, "coldspray.")
                 if not m.name.endswith("__main__"))     # __main__ starts the app


@pytest.mark.parametrize("name", MODULES)
def test_module_imports(name):
    importlib.import_module(name)
