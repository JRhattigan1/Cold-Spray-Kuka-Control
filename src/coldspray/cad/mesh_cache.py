"""Cache STEP -> mesh conversions next to the CAD files (assets/cad/cache)."""

from pathlib import Path

import numpy as np

from coldspray.models import PROJECT_ROOT


def cached_mesh(step_file, deflection=0.5, angle=0.3):
    """(vertices, faces) for a STEP file, re-tessellated only when it changes."""
    step = Path(step_file)
    if not step.is_absolute():
        step = PROJECT_ROOT / step
    cache = step.parent / "cache" / f"{step.stem}_{deflection:g}.npz"
    if cache.exists() and cache.stat().st_mtime >= step.stat().st_mtime:
        d = np.load(cache)
        return d["v"], d["f"]

    from coldspray.cad.step_io import load_step, tessellate  # OpenCascade is slow to import

    v, f = tessellate(load_step(step), deflection, angle)
    cache.parent.mkdir(exist_ok=True)
    np.savez_compressed(cache, v=v.astype(np.float32), f=f.astype(np.int32))
    return v, f
