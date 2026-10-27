"""Planning off the GUI thread (Generate, Auto position), and a small cache of plans.

    plan_jobs.run(win, "Planning...", work, done)   # work() on win.plan_pool, done(out, err) on the GUI thread

`work` must not touch widgets: read every value it needs on the GUI thread first.
`done` is skipped when the set-up changed meanwhile (`win.check_id` moved on, as
`_enter_setup` does). Hosts without a `plan_pool` (bare test hosts) run `work` inline.

The cache maps a plan key (part, placement, Part choices, process, mode, tilt,
rotation, coats, cell without E1, substrate, gun angle, wrist) to plan_part's
(summary, path), so Generate after Auto position doesn't plan again.
"""

import copy
import time
from collections import OrderedDict

import numpy as np
from PySide6.QtWidgets import QApplication

CACHE_SIZE = 8


def snapshot(part):
    """Shallow copy with its own placement and coat faces: a drag on the GUI thread
    can't change what the worker is planning."""
    p = copy.copy(part)
    if hasattr(p, "placement"):
        p.placement = np.array(p.placement, copy=True)
    if hasattr(p, "coat_faces"):
        p.coat_faces = set(p.coat_faces)
    return p


def _num(x):
    return None if x is None else round(float(x), 6)


def plan_key(ident, part, proc, kw: dict) -> tuple:
    """Cache key for plan_part(part, proc, **kw); `ident` is id() of the GUI's part.
    The gantry position is left out: plans don't depend on E1 (Auto position relies on it)."""
    cell = kw.get("cell")
    if cell is not None:
        cell = cell.model_copy(deep=True)
        if getattr(cell, "gantry", None) is not None:
            cell.gantry.position_mm = 0.0
        cell = cell.model_dump_json()
    wrist = kw.get("wrist")
    return (ident, part.name, np.asarray(part.placement, float).round(9).tobytes(),
            tuple(sorted(part.coat_faces)), part.start_face, part.base_face, part.build_body,
            bool(part.fixed_in_cell), proc.model_dump_json(), kw.get("mode"), _num(kw.get("tilt_deg")),
            _num(kw.get("rotation_deg")), kw.get("coats"), cell, repr(kw.get("substrate")),
            _num(kw.get("roll_deg")), None if wrist is None else np.round(wrist, 6).tobytes())


def cached_plan(cache, key, plan):
    """plan() through `cache` (an OrderedDict, or None for no cache)."""
    if cache is None:
        return plan()
    if key in cache:
        cache.move_to_end(key)
        return cache[key]
    out = cache[key] = plan()
    while len(cache) > CACHE_SIZE:
        cache.popitem(last=False)
    return out


def new_cache() -> OrderedDict:
    return OrderedDict()


def _buttons(win, on: bool):
    for name in ("generate_btn", "auto_btn"):
        btn = getattr(win, name, None)
        if btn is not None:
            btn.setEnabled(on)


def run(win, text, work, done, abandon_on_cancel=False):
    """Run `work()` on the plan worker; `done(out, err)` on the GUI thread when it ends.
    `abandon_on_cancel`: work can't stop itself, so Cancel drops it at once (it finishes unseen)."""
    win.plan_lines = []
    win.job.start(text)
    pool = getattr(win, "plan_pool", None)
    if pool is None:                       # no worker: run here (bare test hosts)
        try:
            out, err = work(), None
        except Exception as e:             # noqa: BLE001 - handed to done()
            out, err = None, e
        done(out, err)
        return
    win.plan_job = (pool.submit(work), done, win.check_id, abandon_on_cancel, text)
    _buttons(win, False)
    win.plan_timer.start()


def busy(win) -> bool:
    return getattr(win, "plan_job", None) is not None


def poll(win):
    """Plan timer (GUI thread): show progress; once the work ends, hand it to done()."""
    future, done, token, abandon, text = win.plan_job
    cancelled = win.job.is_cancelled()
    if not future.done():
        if abandon and cancelled:
            _end(win)
            win.job.finish(text.rstrip(".") + " cancelled")
            return
        if win.plan_lines and not cancelled:
            win.job.label.setText(str(win.plan_lines[-1]))
        if token != win.check_id:          # the set-up changed: nobody wants this result
            _end(win)
            win.job.finish()
            return
        win.job.poll()
        return
    _end(win)
    if token != win.check_id:
        win.job.finish()
        return
    try:
        out, err = future.result(), None
    except Exception as e:                 # noqa: BLE001 - handed to done()
        out, err = None, e
    done(out, err)


def _end(win):
    win.plan_job = None
    win.plan_timer.stop()
    _buttons(win, True)


def wait(win, timeout=600.0):
    """Block (pumping events) until the plan job has ended: for tests and scripts."""
    t_end = time.time() + timeout
    while busy(win):
        if time.time() > t_end:
            raise TimeoutError("plan job still running")
        QApplication.processEvents()
        time.sleep(0.01)
