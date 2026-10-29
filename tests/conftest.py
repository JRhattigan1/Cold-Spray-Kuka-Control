"""Test areas: run the key tests for one part of the app instead of the whole suite.

    pytest --area surface          that area's tests + the guard set, slow ones left out
    pytest --area surface,krl      several areas
    pytest --changed               the areas the branch and working-tree changes touch
    pytest --area surface --slow   include the slow end-to-end tests too
    pytest                         everything (no selection)

Areas go by test-file name (AREAS); `--changed` maps source paths to areas (SOURCE_AREAS).
The guard set (GUARD) always runs with an area: cheap checks that catch breakage across
areas (KRL output, imports, config loading). Every test file must be in an area
(tests/test_test_areas.py checks).
"""

import fnmatch
import subprocess
from pathlib import Path

import pytest

TESTS = Path(__file__).parent

AREAS = {
    "surface": ["test_surface*", "test_coat_faces*", "test_toolpath*", "test_S12*"],
    "layers": ["test_layers_patterns*", "test_patterns*", "test_turns*", "test_turn_plane*", "test_contour_tilt*",
               "test_slicing*", "test_revolve*", "test_simplify*", "test_sample_speed*"],
    "krl": ["test_krl.py", "test_krl_joints*", "test_krl_preflight*", "test_krl_regression*", "test_krl_split*",
            "test_review_k2*", "test_ptp_joint_space*"],
    "solver": ["test_solve*", "test_solver*", "test_bench_solve*"],
    "collision": ["test_collision*", "test_home_legs*", "test_checks*"],
    "kinematics": ["test_opw*", "test_robot*", "test_ik_batch*", "test_gun*", "test_tool.py"],
    "part": ["test_part.py", "test_constraints*", "test_datums*", "test_fixed_part*", "test_position*",
             "test_purge*", "test_krl_base*", "test_test_parts*", "test_spacemouse*"],
    "cell": ["test_cell*", "test_config_io*", "test_models*", "test_home_editor*", "test_job*"],
    "sim": ["test_deposit*", "test_markers*", "test_timeline*", "test_plan_jobs*"],
    "gui": ["test_gui_layout*", "test_sidebar_fit*", "test_part_position_fit*", "test_style*", "test_dark_theme*",
            "test_stepper*", "test_u20*", "test_ui_mode*", "test_view_bar*", "test_undo_gui*", "test_shortcuts*",
            "test_workflow*", "test_f3_widgets*", "test_process_form*", "test_s6a*", "test_imports*"],
}
GUARD = ["test_imports.py", "test_krl_regression.py", "test_models.py", "test_config_io.py", "test_test_areas.py"]
# whole files of long end-to-end runs (minutes): left out of area runs unless --slow
SLOW_FILES = ["test_collision_e2e.py", "test_workflow.py", "test_patterns_e2e.py", "test_surface_e2e.py",
              "test_sidebar_fit_f2.py"]

# source path prefix (under src/coldspray/) -> areas; first match wins
SOURCE_AREAS = [
    ("planner/surface", ["surface"]), ("toolpath/", ["surface", "layers"]),
    ("planner/layers", ["layers"]), ("planner/patterns", ["layers"]), ("planner/turns", ["layers"]),
    ("planner/contour_tilt", ["layers"]), ("planner/revolve", ["layers"]), ("cad/slicer", ["layers"]),
    ("planner/solve", ["solver"]), ("planner/position", ["solver", "part"]), ("planner/purge", ["part", "solver"]),
    ("planner/gun", ["kinematics", "surface", "layers"]),
    ("post/", ["krl"]), ("kinematics/", ["kinematics", "krl"]), ("tool.py", ["kinematics", "cell"]),
    ("sim/collision", ["collision", "solver"]), ("sim/deposit", ["sim"]), ("sim/", ["sim", "gui"]),
    ("cad/", ["part"]), ("pipeline.py", ["part", "surface", "layers"]),
    ("models.py", ["cell"]), ("config", ["cell", "krl"]), ("job_io.py", ["cell"]), ("cell.py", ["cell"]),
    ("gui/steps/part", ["part", "gui"]), ("gui/steps/position", ["part", "gui"]),
    ("gui/steps/toolpath", ["gui", "surface", "layers"]), ("gui/steps/simulate", ["sim", "gui"]),
    ("gui/steps/export", ["krl", "gui"]), ("gui/cell_dialog", ["cell"]), ("gui/home_editor", ["cell"]),
    ("gui/job_menu", ["cell"]), ("gui/", ["gui"]),
]


def area_of(name: str) -> list[str]:
    return [a for a, pats in AREAS.items() if any(fnmatch.fnmatch(name, p) for p in pats)]


def pytest_addoption(parser):
    g = parser.getgroup("areas")
    g.addoption("--area", default=None, help=f"comma list of {', '.join(AREAS)} (or 'list')")
    g.addoption("--changed", action="store_true", help="areas touched by changes against --base")
    g.addoption("--base", default="main", help="git ref --changed compares with (default main)")
    g.addoption("--slow", action="store_true", help="with --area/--changed: include the slow end-to-end tests")


def _changed(base: str) -> tuple[set[str], set[str]]:
    """(areas, test files) touched by the branch since `base`, the working tree and untracked files."""
    root = TESTS.parent

    def git(*args):
        r = subprocess.run(["git", *args], cwd=root, capture_output=True, text=True)
        return r.stdout.split() if r.returncode == 0 else []
    paths = set(git("diff", "--name-only", f"{base}...HEAD")) | set(git("diff", "--name-only", "HEAD"))
    paths |= set(git("ls-files", "--others", "--exclude-standard"))
    areas, files = set(), set()
    for p in paths:
        if p.startswith("tests/") and p.endswith(".py"):
            files.add(Path(p).name)
        elif p.startswith("src/coldspray/"):
            rel = p[len("src/coldspray/"):]
            areas.update(next((a for pre, a in SOURCE_AREAS if rel.startswith(pre)), ["gui"]))
    return areas, files


def pytest_configure(config):
    if config.getoption("--area") == "list":
        print("\n".join(f"{a}: {', '.join(p)}" for a, p in AREAS.items()))
        pytest.exit("areas listed", returncode=0)


def pytest_collection_modifyitems(config, items):
    for item in items:
        if item.path.name in SLOW_FILES:
            item.add_marker(pytest.mark.slow)
    area, changed = config.getoption("--area"), config.getoption("--changed")
    if not area and not changed:
        return
    areas, files = set(), set()
    if area:
        areas = {a.strip() for a in area.split(",") if a.strip()}
        unknown = areas - set(AREAS)
        if unknown:
            raise pytest.UsageError(f"unknown area {', '.join(sorted(unknown))}: use {', '.join(AREAS)}")
    if changed:
        a, files = _changed(config.getoption("--base"))
        areas |= a
    slow = config.getoption("--slow")
    keep, drop = [], []
    for item in items:
        name = item.path.name
        wanted = name in GUARD or name in files or bool(set(area_of(name)) & areas)
        if wanted and (slow or name in files or not item.get_closest_marker("slow")):
            keep.append(item)
        else:
            drop.append(item)
    if drop:
        config.hook.pytest_deselected(items=drop)
        items[:] = keep
    config._areas_summary = f"areas: {', '.join(sorted(areas)) or '(none: guard set only)'}"


def pytest_report_header(config):
    if config.getoption("--area") or config.getoption("--changed"):
        return "test areas on (tests/conftest.py)"


def pytest_terminal_summary(terminalreporter, config):
    if hasattr(config, "_areas_summary"):
        terminalreporter.write_line(config._areas_summary)
