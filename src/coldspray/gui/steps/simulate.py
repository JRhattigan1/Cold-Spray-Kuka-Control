"""Step 4 Simulate: reach, gun angle and collision checks (in the background), playback."""

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QHBoxLayout, QSlider, QVBoxLayout, QWidget

from coldspray.gui.checks import CheckList, export_checks, simulate_checks
from coldspray.gui.stepper import EXPORT, SIMULATE
from coldspray.gui.timeline import PlaybackControls, Timeline, sample_times
from coldspray.gui import widgets
from coldspray.gui.widgets import Chip, Section, primary_button, wrap_label

CHECK_EVERY = 3        # collision-check every 3rd sampled point (~6 mm of travel)


class SimulateStep:
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
        self.sim_checks = CheckList()
        self.sim_checks.goTo.connect(self._goto)
        checks.add(self.sim_checks)
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
        return inner

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
        checks = simulate_checks(scene, cell, state)
        self.sim_checks.set_checks(checks)
        problems = sum(c.state not in ("ok", "todo") for c in checks)
        self.checks_section.set_chip("running" if state == "running" else
                                     f"{problems} to look at" if problems else "all clear",
                                     "neutral" if state == "running" else "crash" if problems else "good")
        self.export_list.set_checks(export_checks(scene, cell, self.assets.tool, state))
        n_open = self.export_list.open_count()
        self.robot_section.set_chip(f"{n_open} open" if n_open else "", "warn")
        self.info.setText(self.summary_head)
        self._show_time()

    # --- collision checking (background) -------------------------------------------
    def _start_collision_check(self, robot, gun, cell, part, substrate, fixture, joints):
        from coldspray.sim.collision import CollisionChecker

        self.check_id += 1
        self.collision_failed = None
        job, a = self.check_id, self.assets
        axes_at = self.scene.samples.axes_at
        e1 = cell.gantry.position_mm if cell.gantry else None

        def work():
            if job != self.check_id:
                return None                       # superseded by a newer Generate
            if self.checker is None or self.checker_e1 != e1:
                self.checker = CollisionChecker(robot, cell, a.meshes, gun, part, cell.robot_cad_pose_deg,
                                                substrate=substrate, fixture=fixture)
                self.checker_e1 = e1
            else:
                self.checker.set_gun(robot, gun)
                self.checker.set_workpiece(part, substrate, fixture)
                self.checker.cell = cell
            return job, self.checker.check_path(joints, axes_at, every=CHECK_EVERY,
                                                progress=self.job.set_fraction, cancel=self.job.is_cancelled)

        self.job.start("Collision check...")
        self.check_future = self.pool.submit(work)
        self.poll_timer.start()
        self._update_info()

    def _poll_check(self):
        if self.check_future is None:
            return
        if not self.check_future.done():
            self.job.poll()
            return
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
