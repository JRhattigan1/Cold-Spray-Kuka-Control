"""Job files: round trip, tolerance of missing / unknown keys, STEP path found next to the job."""

import numpy as np

from coldspray import job_io


def _job(source):
    return {"part": {"source": source, "roles": ["build", "fixture"], "build_body": 0, "start_face": 3,
                     "base_face": 5, "coat_faces": {7, 2}, "krl_base": np.eye(4), "krl_base_face": 5},
            "position": {"fixed_in_cell": True, "placement": np.diag([1.0, -1.0, -1.0, 1.0]),
                         "keep_on_plate": False, "e1_mm": np.float64(1234.5), "roll_deg": 15.0,
                         "constraints": [{"kind": "fix", "pose": np.eye(4)}]},
            "toolpath": {"mode": "surface", "coats": 3, "process": {"layer_height_mm": 0.4}},
            "purge": {"enabled": True, "point": (1.0, 2.0, 3.0)}, "export": {"preflight": True}}


def test_round_trip(tmp_path):
    path = job_io.save_job(_job("test_block"), tmp_path / "a")
    assert path.suffix == job_io.SUFFIX
    job = job_io.load_job(path)
    assert job["version"] == job_io.JOB_VERSION
    assert job["part"]["coat_faces"] == [2, 7] and job["part"]["start_face"] == 3
    assert np.allclose(job["part"]["krl_base"], np.eye(4))
    assert np.allclose(job["position"]["placement"], np.diag([1.0, -1.0, -1.0, 1.0]))
    assert job["position"]["e1_mm"] == 1234.5 and job["position"]["fixed_in_cell"]
    assert job["position"]["substrate"]["thickness"] == 10.0          # default filled in
    assert job["toolpath"] == {"mode": "surface", "coats": 3, "process": {"layer_height_mm": 0.4}}
    assert job["purge"]["point"] == [1.0, 2.0, 3.0] and job["export"]["preflight"]
    assert job_io.load_job(job_io.save_job(job, tmp_path / "b.csjob")) == job      # stable


def test_missing_and_unknown_keys(tmp_path):
    p = tmp_path / "old.csjob"
    p.write_text("version: 99\nfuture: 1\npart: {source: test_tube, extra: x}\nposition: 5\n")
    job = job_io.load_job(p)
    assert job["part"]["source"] == "test_tube" and "extra" not in job["part"] and "future" not in job
    assert job["position"] == job_io.DEFAULTS["position"]
    assert job["toolpath"]["mode"] == "layers"
    assert job_io.normalize(None) == job_io.DEFAULTS


def test_step_found_relative_to_job(tmp_path):
    """The STEP file and the job moved together: the path relative to the job finds it."""
    import shutil
    old, new = tmp_path / "old", tmp_path / "new"
    (old / "cad").mkdir(parents=True)
    (old / "jobs").mkdir()
    step = old / "cad" / "p.step"
    step.write_text("x")
    job_io.save_job(_job(str(step)), old / "jobs" / "j.csjob")
    assert job_io.load_job(old / "jobs" / "j.csjob")["part"]["source_rel"] == "../cad/p.step"
    shutil.move(str(old), str(new))
    job = job_io.load_job(new / "jobs" / "j.csjob")
    assert job["part"]["source"] == str((new / "cad" / "p.step").resolve())
