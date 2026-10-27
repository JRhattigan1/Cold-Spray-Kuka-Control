"""Acceptance: Surface mode on built-in parts plans over the picked faces,
auto-positions to full reach and exports KRL. Collision checks: not run here."""

import numpy as np

from coldspray.models import load_process
from coldspray.pipeline import load_part
from coldspray.post.krl import to_krl
from test_patterns_e2e import plan_positioned, unreachable

TILTS = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]


def repair_patch_faces(part):
    """Patch (build body) top + 4 sides: its planar faces except the one facing down."""
    faces = {f: p for f, p in part.planar.items() if part.body_of_face(f) == part.build_body}
    bottom = min(faces, key=lambda f: faces[f].normal[2])
    return set(faces) - {bottom}


def tube_outer(part):
    out = {}
    for f in set(part.face_ids.tolist()) - set(part.planar):
        v = part.vertices[np.unique(part.faces[part.face_ids == f])]
        out[f] = np.hypot(v[:, 0] - v[:, 0].mean(), v[:, 1] - v[:, 1].mean()).mean()
    r = max(out.values())
    return {f for f, d in out.items() if abs(d - r) < 1}


def run(name, pick):
    part = load_part(name)
    part.coat_faces = pick(part)
    proc = load_process()
    pos, path, robot, cell = plan_positioned(part, proc, "surface", TILTS)
    assert any(m.gun_on for m in path.moves)
    assert pos.reach == 1.0
    assert unreachable(path, robot, cell, pos.e1_mm) == 0
    assert to_krl(path, robot, cell, proc).strip()


def test_repair_patch_top_and_sides():
    part = load_part("test_repair")
    assert len(repair_patch_faces(part)) == 5
    run("test_repair", repair_patch_faces)


def test_tube_outer_cylinder():
    run("test_tube", tube_outer)
