"""Job files (*.csjob, YAML): everything needed to reopen a job and export the same KRL.

A job is a plain, versioned dict. Loading is tolerant: missing keys take their defaults,
unknown keys are ignored, so older and newer job files still open.
"""
import os
from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml

JOB_VERSION = 1
SUFFIX = ".csjob"

DEFAULTS = {
    "version": JOB_VERSION,
    "part": {
        "source": None,            # STEP path (absolute) or a built-in part name; None: the test block
        "source_rel": None,        # the STEP path relative to the job file (tried if `source` is gone)
        "roles": [],               # per body: build / fixture / ignore
        "build_body": 0,
        "start_face": None,
        "base_face": None,
        "coat_faces": [],          # Surface mode face picks
        "krl_base": None,          # 4x4 part-CAD <- KRL base, or None (faceplate BASE)
        "krl_base_face": None,
    },
    "position": {
        "fixed_in_cell": False,
        "placement": None,         # 4x4 part-CAD -> faceplate BASE; None: the start placement
        "keep_on_plate": True,
        "substrate": {"enabled": True, "length": 200.0, "width": 200.0, "thickness": 10.0},
        "nozzle": None,
        "tilt_deg": 0.0,
        "rotation_deg": 0.0,
        "e1_mm": None,             # gantry E1 (incl. a solver-applied value); None: the cell default
        "roll_deg": 0.0,           # gun hold: turned about its spray axis
        "constraints": [],         # rows: kind, face, face2, ref, value, suppressed, pose
    },
    "toolpath": {"mode": "layers", "coats": 1, "process": {}},
    "purge": None,                 # PurgeConfig dict (incl. a solver-applied point); None: keep the cell's
    "export": {"preflight": False, "spray_output": True, "split": False},
}


def plain(obj):
    """numpy arrays / scalars, tuples and sets -> YAML-safe lists and Python numbers."""
    if isinstance(obj, np.ndarray):
        return plain(obj.tolist())
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, dict):
        return {str(k): plain(v) for k, v in obj.items()}
    if isinstance(obj, set):
        return sorted(plain(v) for v in obj)
    if isinstance(obj, (list, tuple)):
        return [plain(v) for v in obj]
    return obj


def _merge(default, given):
    """`given` laid over `default`: known keys only, nested dicts merged, wrong-shaped values dropped."""
    if not isinstance(default, dict) or not default:
        return deepcopy(given)
    out = deepcopy(default)
    if not isinstance(given, dict):
        return out
    for key, value in given.items():
        if key not in default:
            continue                       # unknown key (newer file): ignored
        d = default[key]
        if isinstance(d, dict) and d:
            out[key] = _merge(d, value)
        elif isinstance(d, list) and not isinstance(value, list):
            continue
        else:
            out[key] = deepcopy(value)
    return out


def normalize(data) -> dict:
    """A complete job dict from whatever was read (missing keys -> defaults)."""
    job = _merge(DEFAULTS, data if isinstance(data, dict) else {})
    job["version"] = JOB_VERSION
    return job


def save_job(job: dict, path) -> Path:
    """Write `job` as YAML; a STEP source also gets its path relative to the job file."""
    path = Path(path)
    if path.suffix == "":
        path = path.with_suffix(SUFFIX)
    job = normalize(plain(job))
    src = job["part"]["source"]
    if src and Path(src).suffix:
        try:
            rel = os.path.relpath(Path(src).resolve(), path.parent.resolve())
            job["part"]["source_rel"] = Path(rel).as_posix()
        except ValueError:                 # on another drive
            job["part"]["source_rel"] = None
    path.write_text("# Cold spray job\n" + yaml.safe_dump(job, sort_keys=False), encoding="utf-8")
    return path


def load_job(path) -> dict:
    """Read a job file: a complete dict, with `part.source` pointing at a file that exists if
    it can be found (absolute path, else relative to the job file)."""
    path = Path(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    job = normalize(data)
    job["part"]["source"] = resolve_source(job["part"], path)
    return job


def resolve_source(part: dict, job_path) -> str | None:
    src, rel = part.get("source"), part.get("source_rel")
    if not src or not Path(src).suffix:      # built-in part name (or the default test block)
        return src
    if Path(src).exists():
        return src
    if rel:
        cand = (Path(job_path).parent / rel).resolve()
        if cand.exists():
            return str(cand)
    return src
