"""Step 4 Simulate: reach, gun angle and collision checks (in the background), playback."""

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QSlider, QVBoxLayout, QWidget

from coldspray.gui.checks import CheckList, export_checks, simulate_checks
from coldspray.gui.stepper import EXPORT, SIMULATE
from coldspray.gui.timeline import PlaybackControls, Timeline, sample_times
from coldspray.gui import widgets
from coldspray.cad.part import build_mesh, other_mesh
from coldspray.sim.scene import make_gun
from coldspray.gui import style
from coldspray.gui.progress import fit_panel
from coldspray.gui.widgets import Chip, Section, primary_button, wrap_label

CHECK_EVERY = 3        # collision-check every 3rd sampled point (~6 mm of travel)


class _SolveCancelled(Exception):
    """Raised from the solver's log callback on the worker after Cancel."""


def _tile(caption: str):
    """Glass stat tile (display only): big number over a caption. Returns (frame, value label)."""
    t, sp = style.tokens(), style.SPACE
    frame = QFrame()
    frame.setObjectName("statTile")
    frame.setStyleSheet(f"QFrame#statTile {{ background: {t['panel']}; border: 1px solid {t['line']};"
                        f" border-radius: 14px; }} QFrame#statTile QLabel {{ background: transparent; border: none; }}")
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(sp[2], sp[1], sp[2], sp[1])
    lay.setSpacing(sp[0] // 2)
    value = QLabel("–")
    value.setStyleSheet(f"font-size: 22px; font-weight: 700; color: {t['accent']};")
    cap = QLabel(caption)
    cap.setStyleSheet(f"font-size: 11px; color: {t['muted']};")
    lay.addWidget(value)
    lay.addWidget(cap)
    frame.setMinimumWidth(0)
    frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    return frame, value


def capsule(btn):
    """Capsule (pill) look for a button, accent-outlined."""
    t = style.tokens()
    btn.setMinimumHeight(34)
    btn.setStyleSheet(f"QPushButton {{ border-radius: 17px; padding: 4px 18px; border: 1px solid {t['accent']};"
                      f" color: {t['accent']}; background: {t['accent_soft']}; font-weight: 600; }}"
                      f" QPushButton:disabled {{ color: {t['muted']}; border-color: {t['line']}; background: {t['soft']}; }}")
    return btn


class SimulateStep:
    solve_future = None      # background "Solve problems" run
    solve_report = None      # its SolveReport (None until run; cleared by Generate)
    solve_lines = ()         # solver log lines (appended by the worker)

    def _build_simulate_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.addWidget(self._toolpath_notice())
        self.times = None               # seconds into the program at each sample
        self.collision_failed = None    # error text if the background check failed

        checks = self.checks_section = Section("Checks")
        self.check_box = QCheckBox("Check collisions")
        self.check_box.setChecked(True)
        self.check_box.setToolTip("After Generate, check robot, gun and table against the cell and the part\n"
                                  "(runs in the background)")
        # Advanced only, but kept in Simple (with a "changed" chip) once switched off
        row = self.collision_row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.addWidget(self.check_box)
        self.collision_chip = Chip()
        row_layout.addWidget(self.collision_chip)
        row_layout.addStretch(1)
        widgets.advanced(row, changed=lambda: not self.check_box.isChecked(), chip=self.collision_chip)
        self.check_box.toggled.connect(widgets.refresh)
        checks.add(row)
        self.solve_btn = QPushButton("Solve problems")
        self.solve_btn.setToolTip("Try to fix unreachable and colliding points: arm configuration, gun angle,\n"
                                  "approach/jump/purge routes and gantry E1. What can't be fixed is listed.")
        self.solve_btn.clicked.connect(self._solve)
        self.solve_btn.setEnabled(False)
        capsule(self.solve_btn)
        checks.add(self.solve_btn)
        self.apply_suggested_btn = QPushButton("Apply suggested set-up")
        self.apply_suggested_btn.setToolTip("Set the solver's suggested gantry E1 / purge point in the cell")
        self.apply_suggested_btn.clicked.connect(self._apply_suggested)
        self.apply_suggested_btn.setVisible(False)
        checks.add(self.apply_suggested_btn)
        self.sim_checks = CheckList()
        self.sim_checks.goTo.connect(self._goto)
        checks.add(self.sim_checks)
        tiles = QWidget()
        tiles_lay = QGridLayout(tiles)     # 2 x 2: never wider than the panel
        tiles_lay.setContentsMargins(0, 0, 0, 0)
        tiles_lay.setSpacing(style.SPACE[1])
        self.stat_values = {}
        for n, (key, cap) in enumerate((("points", "points"), ("reach", "reachable"), ("collisions", "collisions"), ("time", "time"))):
            frame, self.stat_values[key] = _tile(cap)
            tiles_lay.addWidget(frame, n // 2, n % 2)
        tiles_lay.setColumnStretch(0, 1)
        tiles_lay.setColumnStretch(1, 1)
        checks.add(tiles)
        self.info = wrap_label("")      # toolpath summary
        checks.add(self.info)
        layout.addWidget(checks)

        playback = self.playback_section = Section("Playback")
        self.timeline = Timeline()
        self.timeline.seek.connect(self._goto)
        self.controls = PlaybackControls()
        self.controls.playToggled.connect(self._play)
        self.controls.stepProblem.connect(self._step_problem)
        self.controls.followToggled.connect(self._follow)
        self.play_btn = self.controls.play_btn
        self.slider = QSlider(Qt.Horizontal)   # the current sample; the timeline is its face
        self.slider.valueChanged.connect(self._on_slider)
        self.slider.setVisible(False)
        self.point_label = wrap_label("")
        self.point_label.setProperty("note", True)
        playback.add(self.timeline)
        playback.add(self.controls)
        playback.add(self.point_label)
        layout.addWidget(playback)

        layout.addWidget(primary_button("Next: Export  →", lambda: self.go_to(EXPORT)))
        layout.addStretch(1)
        fit_panel(inner)
        return inner

    def _report(self):
        """The solve report, only while it still belongs to the shown toolpath (Generate replaces it)."""
        ok = self.result is not None and self.result[0] is getattr(self, "_solve_path", None)
        return self.solve_report if ok else None

    def _collision_state(self):
        if not self.check_box.isChecked() or self.assets.meshes is None:
            return "off"
        if self.collision_failed:
            return f"failed: {self.collision_failed}"
        if self.check_future is not None:
            return "running"
        return self.scene.collisions

    def _update_info(self):
        """Check rows, timeline and summary for the current toolpath."""
        if self.result is None:
            return
        path, proc, cell, _ = self.result
        scene, state = self.scene, self._collision_state()
        if self.times is None or len(self.times) != scene.n_points:
            self.times = sample_times(path, scene.samples, proc)
            self.timeline.set_data(scene.n_points, spray=scene.gun_on, unreachable=~scene.reachable,
                                   collision=None, steep=scene.steep, times=self.times)
        self.timeline.set_collisions(scene.collisions)
        checks = simulate_checks(scene, cell, state, self._report())
        self.sim_checks.set_checks(checks)
        problems = sum(c.state not in ("ok", "todo") for c in checks)
        self.checks_section.set_chip("running" if state == "running" else
                                     f"{problems} to look at" if problems else "all clear",
                                     "neutral" if state == "running" else "crash" if problems else "good")
        self.export_list.set_checks(export_checks(scene, cell, self.assets.tool, state, self._report(),
                                                  part_base=self._part_base() is not None))
        n_open = self.export_list.open_count()
        self.robot_section.set_chip(f"{n_open} open" if n_open else "", "warn")
        self.info.setText(self.summary_head)
        n = scene.n_points
        sv = self.stat_values
        sv["points"].setText(str(n))
        sv["reach"].setText(f"{100 * float(scene.reachable.mean()):.0f}%" if n else "–")
        sv["collisions"].setText("–" if state in ("off", "running") or str(state).startswith("failed")
                                 else str(len(scene.collisions)))
        sv["time"].setText(f"{self.times[-1]:.0f} s" if self.times is not None and len(self.times) else "–")
        busy = self.check_future is not None or self.solve_future is not None
        self.solve_btn.setEnabled(not busy)
        report = self._report()
        self.apply_suggested_btn.setVisible(bool(report is not None and getattr(report, "suggested", None)))
        unfixed = bool(report is not None and report.unfixed)
        self.save_krl_btn.setEnabled(getattr(self, "unreachable", 0) == 0 and not unfixed)
        self._show_time()

    # --- collision checking (background) -------------------------------------------
    def _home_first_leg(self, robot, cell, joints):
        """Joints along HOME -> first point as the KRL moves it (a LIN when move 0 is a LIN),
        or None (the PTP joint sweep) when move 0 is a PTP or the scene has no path."""
        sc = self.scene
        kinds = getattr(sc, "move_kind", None)
        if not kinds or kinds[0] == "PTP" or not len(joints) or getattr(sc, "root_frames", None) is None:
            return None
        import numpy as np
        from coldspray.planner.solver import home_first_leg
        target0 = np.linalg.inv(sc.root_frames[0]) @ sc.base_frames[0] @ sc.samples.poses[0]
        return home_first_leg(robot, cell.home_deg, target0, joints[0], False)

    def _start_collision_check(self, robot, gun, cell, part, substrate, fixture, joints):
        from coldspray.sim.collision import CollisionChecker, part_for_mode

        part = part_for_mode(getattr(self, "result_mode", None), part)   # Layers: the part isn't built yet
        self.check_id += 1
        self.collision_failed = None
        job, a = self.check_id, self.assets
        axes_at = self.scene.samples.axes_at
        e1 = cell.gantry.position_mm if cell.gantry else None
        first_leg = self._home_first_leg(robot, cell, joints)

        def work():
            if job != self.check_id:
                return None                       # superseded by a newer Generate
            if self.checker is None or self.checker_e1 != e1:
                self.checker = CollisionChecker(robot, cell, a.meshes, gun, part, cell.robot_cad_pose_deg,
                                                substrate=substrate, fixture=fixture)
                self.checker_e1 = e1
                allowed = getattr(self.checker, "allowed_text", None)   # K3; absent before its merge
                if allowed:
                    print(allowed())
            else:
                self.checker.set_gun(robot, gun)
                self.checker.set_workpiece(part, substrate, fixture)
                self.checker.cell = cell
            return job, self.checker.check_path(joints, axes_at, every=CHECK_EVERY, home=cell.home_deg,
                                                first_leg=first_leg,
                                                progress=self.job.set_fraction, cancel=self.job.is_cancelled)

        self.job.start("Collision check...")
        self.check_future = self.pool.submit(work)
        self.poll_timer.start()
        self._update_info()

    # --- solve problems (background, same worker) -----------------------------------
    def _solve(self):
        """Run planner.solver.solve on the collision worker (FCL objects live there);
        the poll timer picks up the result. Nothing here touches widgets from the worker."""
        if self.result is None or self.check_future is not None or self.solve_future is not None:
            return
        from coldspray.planner import solver

        path, _, cell, robot = self.result
        self.check_id += 1
        job, use_checker, cancelled = self.check_id, self.check_box.isChecked(), self.job.is_cancelled
        lines = []                     # worker appends only (thread-safe); the GUI thread reads it

        def log(msg):
            lines.append(msg)
            if cancelled():
                raise _SolveCancelled  # stop between levers; the old path is kept

        def work():
            if job != self.check_id:
                return None            # superseded (new Generate / set-up) before it started
            checker = self.checker if use_checker else None    # read on the worker, where it is built
            ctx = solver.Context(robot, cell, checker, log=log)
            try:
                return (job, *solver.solve(path, ctx))
            except _SolveCancelled:
                return None

        self.solve_lines = lines
        self.job.start("Solving problems...")
        self.solve_future = self.pool.submit(work)
        self.solve_btn.setEnabled(False)
        self.poll_timer.start()

    def _apply_suggested(self):
        """Put the solver's suggestions (report.suggested) into the cell set-up: gantry E1
        (the Position step's E1 box) and purge point (its X/Y/Z boxes, spray direction kept)."""
        report = self._report()
        sug = getattr(report, "suggested", None) or {}
        done = []
        if "gantry_e1" in sug and getattr(self, "e1", None) is not None:
            self.e1.setValue(float(sug["gantry_e1"]))
            done.append(f"gantry E1 = {sug['gantry_e1']:.0f} mm")
        if "purge_point" in sug:
            boxes = getattr(self, "purge_xyz", None)
            if boxes:
                for box, v in zip(boxes, sug["purge_point"]):
                    box.setValue(float(v))
            cell = getattr(getattr(self, "assets", None), "cell", None)
            if getattr(cell, "purge", None) is not None:
                cell.purge.point = tuple(float(v) for v in sug["purge_point"])
            done.append("purge point = (" + ", ".join(f"{v:.0f}" for v in sug["purge_point"]) + ") mm")
        if done:
            self._status("Applied " + "; ".join(done))

    def _poll_solve(self):
        """GUI thread: show progress lines; once done, apply or drop the result."""
        if not self.solve_future.done():
            if self.solve_lines and not self.job.is_cancelled():
                self.job.label.setText(self.solve_lines[-1])
            self.job.poll()
            return
        future, self.solve_future = self.solve_future, None
        cancelled = self.job.is_cancelled()
        try:
            out = future.result()
        except Exception as e:
            self.job.finish("Solve failed")
            self._status(f"Solve failed: {e}")
            self._update_info()
            return
        if out is None or out[0] != self.check_id or self.result is None:
            if self.check_future is None:      # else the job bar belongs to a newer check
                self.job.finish("Solve cancelled: toolpath unchanged" if cancelled else "")
            self._update_info()
            return
        if cancelled:                           # finished anyway: still honour Cancel
            self.job.finish("Solve cancelled: toolpath unchanged")
            self._update_info()
            return
        _, new, report = out
        self.job.finish(f"Solve: {len(report.fixed)} fixes, {len(report.unfixed)} problem points left")
        self.solve_report, self._solve_path = report, new
        if new is not self.result[0]:
            self._show_solved(new)
        else:
            self._update_info()

    def _show_solved(self, path):
        """Replace the toolpath with the solver's and redo what Generate does after planning:
        scene + IK (seeded from the path's joint hints), checks, program, collision check."""
        _, proc, cell, robot = self.result
        part_mesh, fixture_mesh = build_mesh(self.part), other_mesh(self.part)
        substrate_mesh = self._substrate_mesh()
        joints = self.scene.set_path(path, cell, part=part_mesh, substrate=substrate_mesh,
                                     fixture=fixture_mesh, step=2.0, home=cell.home_deg, angle_step=5.0,
                                     gun_limit_deg=cell.gun_max_down_deg)
        self.result = (path, proc, cell, robot)
        self.slider.blockSignals(True)
        self.slider.setRange(0, self.scene.n_points - 1)
        self.slider.setValue(min(self.slider.value(), self.scene.n_points - 1))
        self.slider.blockSignals(False)
        self.unreachable = int(np.isnan(joints[:, 0]).sum())
        self.times, self.collision_failed, self.collisions = None, None, []
        self._update_info()
        self._show_program()
        self.plotter.render()
        self._invalidate(SIMULATE)              # a new toolpath: check and save it again
        if self.check_box.isChecked() and self.assets.meshes is not None:
            gun = make_gun(robot.tool, self.assets.gun_mesh)
            self._start_collision_check(robot, gun, cell, part_mesh, substrate_mesh, fixture_mesh, joints)
        else:
            self._mark_done(SIMULATE)

    def _poll_check(self):
        """Poll timer: the solve job and the collision check (either may be running)."""
        if self.solve_future is not None:
            self._poll_solve()
        if self.solve_future is None and self.check_future is None:
            self.poll_timer.stop()
        if self.check_future is None:
            return
        if not self.check_future.done():
            self.job.poll()
            return
        if self.solve_future is None:
            self.poll_timer.stop()
        future, self.check_future = self.check_future, None
        cancelled = self.job.is_cancelled()
        try:
            out = future.result()
        except Exception as e:
            self.job.finish("Collision check failed")
            self.collision_failed = str(e)
            self._update_info()
            return
        if out is None or out[0] != self.check_id:
            self.job.finish()
            return
        hits = out[1]
        if cancelled:          # partial: only the points checked before Cancel
            self.job.finish(f"Collision check cancelled: {len(hits)} collision points so far")
            self.collision_failed = "cancelled, only part of the path was checked"
        else:
            self.job.finish(f"Collision check: {len(hits)} collision points")
        self.scene.set_collisions(hits)
        self.plotter.render()
        self.collisions = hits
        self._update_info()
        self._mark_done(SIMULATE)

    # --- playback -------------------------------------------------------------
    def _goto(self, i):
        """Show sample `i` (from the timeline, a check's Go to, or a problem step)."""
        if self.result is not None:
            self.slider.setValue(int(i))

    def _on_slider(self, i):
        if self.scene and self.scene.mode == "preview":
            self.scene.show_point(i)
            self.timeline.set_current(i)
            self._show_time()
            self.plotter.render()

    def _show_time(self):
        scene = self.scene
        if self.result is None or not scene.n_points:
            self.point_label.setText("")
            return
        i = int(np.clip(self.slider.value(), 0, scene.n_points - 1))
        spray = "spray on" if scene.gun_on[i] else "spray off"
        self.point_label.setText(f"Point {i} / {scene.n_points - 1} · {spray} · "
                                 f"gun {round(scene.gun_down[i])}° down · {self.timeline.time_text()}")

    def _step_problem(self, direction):
        j = self.timeline.next_problem() if direction > 0 else self.timeline.prev_problem()
        if j is None:
            self._status("No further problems in that direction")
        else:
            self._goto(j)

    def _follow(self, on):
        follow = getattr(self.scene, "set_follow", None)
        if follow:
            follow(on)

    def _play(self, on):
        if on and self.result is not None and self.scene.n_points:
            if self.slider.value() >= self.slider.maximum():
                self._goto(0)
            self.play_timer.start()
        else:
            self.play_timer.stop()
            self.controls.set_playing(False)

    def _toggle_play(self):
        self._play(not self.play_timer.isActive())
        self.controls.set_playing(self.play_timer.isActive())

    def _tick(self):
        """Advance by real time x speed (one sample at least)."""
        i = self.slider.value()
        if self.times is not None and len(self.times) == self.scene.n_points:
            t = self.times[i] + self.play_timer.interval() / 1000 * self.controls.speed_factor()
            j = max(i + 1, int(np.searchsorted(self.times, t)))
        else:
            j = i + 1
        if j > self.slider.maximum():
            self.slider.setValue(self.slider.maximum())
            self._play(False)
            return
        self.slider.setValue(j)
