import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from coldspray.cad.part import (Part, Substrate, build_frame, build_mesh, centre_on_faceplate, constrain, drop_on_plate,
                                lower_to_floor, other_mesh, placed_vertices, rotation_between, snap_face_down)
from coldspray.models import load_process
from coldspray.pipeline import load_part, plan_part
from coldspray.transforms import frame

TOP = Substrate().top


@pytest.fixture(scope="module")
def block():
    return load_part("test_block")   # 40 x 30 x 5 box at (10, 10, 0) with a 10 mm hole


def face_with_normal(part, n):
    return next(i for i, f in part.planar.items() if np.allclose(f.normal, n, atol=1e-9))


def test_face_ids(block):
    assert len(block.face_ids) == len(block.faces)
    assert set(block.face_ids) == set(range(7))           # 6 box faces + the hole wall
    normals = sorted(tuple(np.round(f.normal, 9) + 0.0) for f in block.planar.values())
    assert normals == sorted([(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)])
    hole = (set(block.face_ids) - set(block.planar)).pop()
    assert np.ptp(block.vertices[np.unique(block.faces[block.face_ids == hole])][:, 2]) == pytest.approx(5)
    top = block.planar[face_with_normal(block, (0, 0, 1))]
    assert top.point[2] == pytest.approx(5) and top.area == pytest.approx(40 * 30 - np.pi * 25, rel=1e-6)


def test_load_part_drops_on_plate(block):
    v = placed_vertices(block)
    assert v[:, 2].min() == pytest.approx(TOP)
    assert np.allclose(v[:, :2].min(axis=0), -v[:, :2].max(axis=0))   # centred on the table axis


def check_face_down(part, face_id, placement):
    face = part.planar[face_id]
    R, t = placement[:3, :3], placement[:3, 3]
    assert np.allclose(R @ face.normal, [0, 0, -1])
    assert np.isclose((R @ face.point + t)[2], TOP)
    assert np.isclose(np.linalg.det(R), 1)
    v = part.vertices @ R.T + t
    assert v[:, 2].min() == pytest.approx(TOP)


@pytest.mark.parametrize("normal", [(0, 0, 1), (0, 0, -1), (1, 0, 0), (0, -1, 0)])
def test_snap_face_down(block, normal):
    fid = face_with_normal(block, normal)
    placement, before = block.placement.copy(), placed_vertices(block)
    t = snap_face_down(block, fid, TOP)
    check_face_down(block, fid, t)
    after = block.vertices @ t[:3, :3].T + t[:3, 3]
    centre = lambda v: (v[:, :2].min(axis=0) + v[:, :2].max(axis=0)) / 2
    assert np.allclose(centre(after), centre(before))
    assert np.array_equal(block.placement, placement)   # not mutated


def test_snap_rejects_curved_face(block):
    hole = (set(block.face_ids) - set(block.planar)).pop()
    with pytest.raises(ValueError):
        snap_face_down(block, hole, TOP)


def test_rotation_between_opposite():
    assert np.allclose(rotation_between((0, 0, 1), (0, 0, -1)), np.diag([1, -1, -1]))
    for a in [(1, 0, 0), (0, 1, 0), (0.6, 0, 0.8)]:
        R = rotation_between(a, np.negative(a))
        assert np.allclose(R @ a, np.negative(a)) and np.isclose(np.linalg.det(R), 1)


def test_constrain_keeps_yaw_and_xy(block):
    part = Part(block.name, block.vertices, block.faces, block.face_ids, block.planar)
    part.base_face = face_with_normal(part, (0, 0, 1))
    part.placement = snap_face_down(part, part.base_face, TOP)
    good = frame(15, -7, 0, a=30) @ part.placement
    good[2, 3] = part.placement[2, 3]
    bad = good.copy()
    bad[:3, :3] = Rotation.from_euler("xy", [10, -5], degrees=True).as_matrix() @ bad[:3, :3]
    bad[2, 3] += 20
    fixed = constrain(part, bad, TOP)
    check_face_down(part, part.base_face, fixed)
    assert np.allclose(fixed[:2, 3], bad[:2, 3])
    yaw = lambda t: np.degrees(np.arctan2(t[1, 0], t[0, 0]))
    assert yaw(fixed) == pytest.approx(yaw(good), abs=0.5)
    assert np.allclose(constrain(part, good, TOP), good)
    part.base_face = None
    assert constrain(part, bad, TOP) is bad


def test_centre_on_faceplate(block):
    part = Part(block.name, block.vertices, block.faces, block.face_ids, block.planar)
    t = centre_on_faceplate(part)
    v = part.vertices @ t[:3, :3].T + t[:3, 3]
    assert np.allclose(v[:, :2].min(axis=0), -v[:, :2].max(axis=0))
    assert np.allclose(v[:, 2], part.vertices[:, 2])


def test_substrate_mesh():
    s = Substrate(100, 60, 8)
    v, f = s.mesh()
    assert np.allclose(v.min(axis=0), [-50, -30, 0]) and np.allclose(v.max(axis=0), [50, 30, 8])
    assert s.top == 8 and len(f) == 12


def test_plan_part_on_plate(block):
    proc = load_process()
    part = Part(block.name, block.vertices, block.faces, block.face_ids, block.planar)
    part.base_face = face_with_normal(part, (0, 0, -1))
    part.placement = snap_face_down(part, part.base_face, TOP)
    summary, path = plan_part(part, proc)
    assert summary == "10 layers"
    spray = np.array([m.pose[:3, 3] for m in path.moves if m.gun_on and m.speed == proc.traverse_speed_mm_s])
    assert spray[:, 2].min() == pytest.approx(TOP + proc.standoff_mm)
    assert np.all(np.array([m.pose[2, 3] for m in path.moves]) > TOP)
    # the hole (CAD centre (30, 25)) is now on the table axis: no spray there
    assert np.linalg.norm(spray[:, :2], axis=1).min() > 5


def test_plan_part_rejects_bad_placement(block):
    proc = load_process()
    part = Part(block.name, block.vertices, block.faces, block.face_ids, block.planar,
                placement=frame(z=TOP - 2) @ drop_on_plate(block, 0.0))
    with pytest.raises(ValueError, match="into the substrate plate by 2.0 mm"):
        plan_part(part, proc)
    part.placement = drop_on_plate(block, TOP + 5)
    assert plan_part(part, proc)[0] == "10 layers"    # fine: held off the plate (fixture, riser)
    plan_part(part, proc, mode="revolve")
    part.placement = frame(x=600) @ drop_on_plate(block, TOP)
    with pytest.raises(ValueError, match="faceplate"):
        plan_part(part, proc)



# --- assemblies: several bodies, one of them built --------------------------------------------------

@pytest.fixture(scope="module")
def repair():
    return load_part("test_repair")   # 120 x 80 x 30 component, 40 x 20 x 6 patch on its top


def test_assembly_bodies(repair):
    assert repair.n_bodies == 2
    assert repair.build_body == 1                          # the smaller body: the patch
    assert len(repair.planar) == 12                        # two boxes, faces numbered on
    v, f = build_mesh(repair)
    assert np.allclose(np.ptp(v, axis=0), [40, 20, 6])
    rest, _ = other_mesh(repair)
    assert np.allclose(np.ptp(rest, axis=0), [120, 80, 30])
    top_of_component = next(i for i, p in repair.planar.items()
                            if np.allclose(p.normal, [0, 0, 1]) and np.isclose(p.point[2], 30))
    assert repair.body_of_face(top_of_component) == 0


def test_single_body_has_no_others(block):
    assert block.n_bodies == 1 and other_mesh(block) is None


def test_repair_patch_layers_start_on_the_component(repair):
    proc = load_process()
    part = Part(repair.name, repair.vertices, repair.faces, repair.face_ids, repair.planar,
                body_ids=repair.body_ids, build_body=1)
    part.placement = drop_on_plate(part, 0.0)              # no plate: on the faceplate
    summary, path = plan_part(part, proc, substrate=None)
    assert summary == f"{round(6 / proc.layer_thickness_mm)} layers"
    spray = np.array([m.pose[2, 3] for m in path.moves if m.gun_on and m.speed == proc.traverse_speed_mm_s])
    assert spray.min() == pytest.approx(30 + proc.standoff_mm)
    part.build_body = 0                                    # build the component instead
    assert plan_part(part, proc, substrate=None)[0] == f"{round(30 / proc.layer_thickness_mm)} layers"


def test_no_plate_placement_checks(repair):
    part = Part(repair.name, repair.vertices, repair.faces, repair.face_ids, repair.planar,
                body_ids=repair.body_ids, build_body=1, placement=drop_on_plate(repair, 5.0))
    plan_part(part, load_process(), substrate=None)            # held 5 mm up: fine
    part.placement = drop_on_plate(repair, -1.0)
    with pytest.raises(ValueError, match="into the faceplate by 1.0 mm"):
        plan_part(part, load_process(), substrate=None)
    part.placement = frame(x=600) @ drop_on_plate(repair, 0.0)
    with pytest.raises(ValueError, match="from the table axis"):
        plan_part(part, load_process(), substrate=None)


def test_lower_to_floor_keeps_orientation_and_xy(repair):
    part = Part(repair.name, repair.vertices, repair.faces, repair.face_ids, repair.planar,
                body_ids=repair.body_ids, build_body=1,
                placement=frame(x=12, y=-7, z=80, a=20, b=30, c=10))
    t = lower_to_floor(part, 4.0)
    assert np.allclose(t[:3, :3], part.placement[:3, :3])
    assert np.allclose(t[:2, 3], part.placement[:2, 3])
    part.placement = t
    assert placed_vertices(part)[:, 2].min() == pytest.approx(4.0)


def test_snap_any_body_face_stands_assembly_on_floor(repair):
    part = Part(repair.name, repair.vertices, repair.faces, repair.face_ids, repair.planar,
                body_ids=repair.body_ids, build_body=1)
    side = next(i for i, p in part.planar.items() if np.allclose(p.normal, [1, 0, 0]) and np.isclose(p.point[0], 120))
    t = snap_face_down(part, side, 0.0)
    v = part.vertices @ t[:3, :3].T + t[:3, 3]
    assert np.allclose(t[:3, :3] @ part.planar[side].normal, [0, 0, -1])
    assert v[:, 2].min() == pytest.approx(0.0)


def test_without_drops_a_body(repair):
    part = Part(repair.name, repair.vertices, repair.faces, repair.face_ids, repair.planar,
                body_ids=repair.body_ids, build_body=1)
    part.start_face = face_with_normal_on(part, (0, 0, -1), body=1)
    part.base_face = face_with_normal_on(part, (0, 0, -1), body=0)
    alone = part.without([0])
    assert alone.n_bodies == 1 and alone.body_ids is None and alone.build_body == 0
    assert len(alone.planar) == 6 and alone.start_face == part.start_face and alone.base_face is None
    assert np.allclose(np.ptp(alone.vertices, axis=0), [40, 20, 6])
    assert part.without([]).n_bodies == 2
    with pytest.raises(ValueError):
        part.without([1])


def face_with_normal_on(part, n, body):
    return next(i for i, f in part.planar.items()
                if np.allclose(f.normal, n, atol=1e-9) and part.body_of_face(i) == body)


def test_patch_on_a_side_wall_builds_sideways(repair):
    """Assembly turned so the patch sticks out sideways (-Y): with the patch's
    underside as start face the layers stack along -Y, starting on the wall."""
    proc = load_process()
    part = Part(repair.name, repair.vertices, repair.faces, repair.face_ids, repair.planar,
                body_ids=repair.body_ids, build_body=1)
    part.start_face = face_with_normal_on(part, (0, 0, -1), body=1)
    part.placement = frame(c=90)
    part.placement = drop_on_plate(part, 0.0)
    to_base = build_frame(part)
    assert np.allclose(to_base[:3, 2], [0, -1, 0])                 # build direction
    summary, path = plan_part(part, proc, substrate=None)
    assert summary.startswith(f"{round(6 / proc.layer_thickness_mm)} layers")
    spray = [m.pose for m in path.moves if m.gun_on and m.speed == proc.traverse_speed_mm_s]
    height = np.array([(p[:3, 3] - to_base[:3, 3]) @ to_base[:3, 2] for p in spray])
    assert height.min() == pytest.approx(proc.standoff_mm)          # first layer: standoff off the wall
    assert np.allclose([p[:3, 0] for p in spray], [0, 1, 0])        # gun sprays towards the wall


def test_build_frame_without_start_face(block):
    assert np.allclose(build_frame(block), frame(z=TOP))


def test_tube_cylinders():
    tube = load_part("test_tube")
    radii = sorted({round(c.radius, 3) for c in tube.cylinders.values()})
    assert radii == [40.0, 60.0]
    for c in tube.cylinders.values():
        assert np.allclose(np.abs(c.axis), [0, 0, 1])
        assert np.allclose(c.point[:2], 0) and c.area > 0
    assert not set(tube.cylinders) & set(tube.planar)
    assert set(tube.cylinders) <= set(tube.face_ids.tolist())
    assert tube.without([]).cylinders == tube.cylinders
    assert [round(c.radius, 3) for c in load_part("test_block").cylinders.values()] == [5.0]   # its hole
