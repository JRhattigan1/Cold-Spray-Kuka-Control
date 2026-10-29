"""Coat the outside of a rotational part by spinning the turntable.

The part sits on the faceplate with its axis on the rotation axis (base Z).
The nozzle stays on the robot side of the table, pointing at the axis, while
the table turns one revolution per pass; between passes it steps
`step_over_mm` along the axis. The rotation axis has limited travel, so passes
alternate direction (-180 -> +180, then back) instead of spinning
continuously. Each coat is one sweep over the whole height; every coat the
nozzle backs off by `layer_thickness_mm` as the deposit builds up.

The surface radius at each height is taken from the part mesh, so stepped or
tapered parts are followed at constant standoff.
"""

import numpy as np

from coldspray.models import ProcessConfig
from coldspray.toolpath.model import Move, Toolpath, spray_frame

SIDE_DEG = 90.0      # nozzle position around the part at rotation 0: base +Y = robot side
STEP_DEG = 5.0       # points every 5 degrees of table rotation


def radius_profile(vertices, faces, heights) -> np.ndarray:
    """Largest distance from the base Z axis of the part's cross-section at
    each height (NaN where the part has no material)."""
    import trimesh
    mesh = trimesh.Trimesh(vertices, faces, process=True)
    radii = []
    for z in heights:
        section = mesh.section(plane_origin=(0, 0, z), plane_normal=(0, 0, 1))
        radii.append(np.linalg.norm(section.vertices[:, :2], axis=1).max() if section is not None else np.nan)
    return np.array(radii)


def merge_bands(bands, gap=1e-6) -> list[tuple[float, float]]:
    """(zmin, zmax) bands with overlapping or touching ones joined, low to high."""
    merged = []
    for lo, hi in sorted(bands):
        if merged and lo <= merged[-1][1] + gap:
            merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
        else:
            merged.append((lo, hi))
    return merged


def band_heights(lo, hi, proc: ProcessConfig) -> np.ndarray:
    """Pass heights covering lo..hi, half a track in from each end."""
    half = min(proc.track_width_mm / 2, (hi - lo) / 2)
    heights = np.arange(lo + half, hi - half + 1e-9, proc.step_over_mm)
    return heights if len(heights) else np.array([(lo + hi) / 2])


def plan_revolve(vertices, faces, proc: ProcessConfig, name="part", coats=1, tilt_deg=0.0,
                 tilt_axis="E2", rotation_axis="E5", bands=None, helical=False) -> Toolpath:
    """Toolpath with the turntable rotation (and fixed tilt) on every move.
    `bands`: (zmin, zmax) ranges to coat, each a separate region (the gun lifts
    off between them); default the whole height of the part. `helical` (an
    endless rotation axis): the table always turns the same way and each band
    is one helix of pitch `step_over_mm`, with a full flat turn at each end."""
    if bands is None:
        bands = [(vertices[:, 2].min(), vertices[:, 2].max())]
    regions = []
    for lo, hi in merge_bands(bands):
        heights = band_heights(lo, hi, proc)
        radii = radius_profile(vertices, faces, heights)
        keep = ~np.isnan(radii)
        if keep.any():
            regions.append((heights[keep], radii[keep]))
    if not regions:
        return Toolpath(name)

    def pose(radius, z, part_angle_deg):
        """Nozzle at `radius` from the axis at `part_angle_deg` (part frame), spraying inwards."""
        a = np.radians(part_angle_deg)
        radial = np.array([np.cos(a), np.sin(a), 0.0])
        tangent = np.array([-np.sin(a), np.cos(a), 0.0])
        return spray_frame(radial * radius + [0, 0, z], -radial, tangent)

    def nozzle(rotation_deg, radius, z, extra=0.0):
        # With the table turned by `rotation_deg`, the spot facing the robot is
        # at SIDE_DEG - rotation_deg on the part.
        return pose(radius + proc.standoff_mm + extra, z, SIDE_DEG - rotation_deg)

    path = Toolpath(name)
    fixed = {tilt_axis: tilt_deg}
    if helical:
        return _helix(path, regions, proc, coats, fixed, rotation_axis, nozzle)
    sweep = np.arange(-180.0, 180.0 + 1e-9, STEP_DEG)
    for coat in range(coats):
        for i, (heights, radii) in enumerate(regions):
            grown = radii + coat * proc.layer_thickness_mm
            if i == 0:
                start = sweep[0] if coat % 2 == 0 else sweep[-1]
            else:  # the next region starts where the table stopped
                start = path.moves[-1].axes.get(rotation_axis, sweep[0])
            first = fixed | {rotation_axis: start}
            kind = "PTP" if not path.moves else "LIN"
            path.moves.append(Move(nozzle(start, grown[0], heights[0], proc.approach_height_mm),
                                   proc.rapid_speed_mm_s, kind=kind, axes=first))
            path.moves.append(Move(nozzle(start, grown[0], heights[0]), proc.rapid_speed_mm_s))
            direction = 1 if start == sweep[0] else -1
            for k, (z, r) in enumerate(zip(heights, grown)):
                angles = sweep if direction == 1 else sweep[::-1]
                # Move speeds are measured at the nozzle (TCP); scale so the spot
                # crosses the part surface at the traverse speed.
                speed = proc.traverse_speed_mm_s * (r + proc.standoff_mm) / r
                if k > 0:  # step along the axis at the end of the previous turn
                    path.moves.append(Move(nozzle(angles[0], r, z), proc.traverse_speed_mm_s, gun_on=True,
                                           axes={rotation_axis: angles[0]}))
                for ang in angles[1:]:
                    path.moves.append(Move(nozzle(ang, r, z), speed, gun_on=True,
                                           axes={rotation_axis: float(ang)}))
                direction = -direction
            end = path.moves[-1].axes.get(rotation_axis, start)
            path.moves.append(Move(nozzle(end, grown[-1], heights[-1], proc.approach_height_mm),
                                   proc.rapid_speed_mm_s))
    return path


def _helix(path, regions, proc: ProcessConfig, coats, fixed, rotation_axis, nozzle) -> Toolpath:
    """Helical passes for an endless rotation axis: the table angle only grows
    (it carries on from one band and coat to the next); odd coats go back down."""
    angle = 0.0
    for coat in range(coats):
        for heights, radii in regions:
            grown = radii + coat * proc.layer_thickness_mm
            lo, hi = heights[0], heights[-1]
            climb = (hi - lo) / proc.step_over_mm * 360.0            # degrees of helix
            s = np.arange(0.0, climb + 720.0 + 1e-9, STEP_DEG)    # flat turn, helix, flat turn
            z = lo + np.clip(s - 360.0, 0.0, climb) / 360.0 * proc.step_over_mm
            if coat % 2:
                z = lo + hi - z
            r = np.interp(z, heights, grown)
            kind = "PTP" if not path.moves else "LIN"
            path.moves.append(Move(nozzle(angle, r[0], z[0], proc.approach_height_mm), proc.rapid_speed_mm_s,
                                   kind=kind, axes=fixed | {rotation_axis: angle}))
            path.moves.append(Move(nozzle(angle, r[0], z[0]), proc.rapid_speed_mm_s))
            for ds, zk, rk in zip(s[1:], z[1:], r[1:]):
                # TCP speed: scaled so the spot crosses the surface at the traverse speed
                speed = proc.traverse_speed_mm_s * (rk + proc.standoff_mm) / rk
                path.moves.append(Move(nozzle(angle + ds, rk, zk), speed, gun_on=True,
                                       axes={rotation_axis: float(angle + ds)}))
            angle += s[-1]
            path.moves.append(Move(nozzle(angle, r[-1], z[-1], proc.approach_height_mm), proc.rapid_speed_mm_s))
    return path
