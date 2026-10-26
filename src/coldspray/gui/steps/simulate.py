"""Step 4 Simulate: reach, gun angle and collision checks (in the background), playback."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QHBoxLayout, QPushButton, QSlider, QVBoxLayout, QWidget

from coldspray.gui.stepper import EXPORT, SIMULATE
from coldspray.gui.widgets import Section, primary_button, wrap_label

CHECK_EVERY = 3        # collision-check every 3rd sampled point (~6 mm of travel)


class SimulateStep:
    def _build_simulate_panel(self) -> QWidget:
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.addWidget(self._toolpath_notice())

        checks = self.checks_section = Section("Checks")
        self.check_box = QCheckBox("Check collisions")
        self.check_box.setChecked(True)
        self.check_box.setToolTip("After Generate, check robot, gun and table against the cell and the part\n"
                                  "(runs in the background)")
        checks.add(self.check_box)
        self.info = wrap_label("")
        checks.add(self.info)
        layout.addWidget(checks)

        playback = self.playback_section = Section("Playback")
        row = QHBoxLayout()
        self.play_btn = QPushButton("Play")
        self.play_btn.clicked.connect(self._toggle_play)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self._on_slider)
        row.addWidget(self.play_btn)
        row.addWidget(self.slider, 1)
        playback.add(row)
        layout.addWidget(playback)

        layout.addWidget(primary_button("Next: Export  →", lambda: self.go_to(EXPORT)))
        layout.addStretch(1)
        return inner

    def _update_info(self):
        reach = (f"<span style='color:#d93025'>{self.unreachable} unreachable points</span>" if self.unreachable
                 else "<span style='color:green'>All points reachable</span>")
        steep, limit = int(self.scene.steep.sum()), self.assets.cell.gun_max_down_deg
        gun = (f"<span style='color:#b38600'>{steep} points with the gun more than {limit:.0f}° down "
               f"(max {self.scene.gun_down.max():.0f}°, yellow)</span>" if steep
               else f"<span style='color:green'>Gun within {limit:.0f}° of level "
                    f"(max {max(self.scene.gun_down.max(), 0):.0f}° down)</span>")
        self.info.setText(self.summary_head + reach + "<br>" + gun + "<br>" + self.collision_text)

    # --- collision checking (background) -------------------------------------------
    def _start_collision_check(self, robot, gun, cell, part, substrate, fixture, joints):
        from coldspray.sim.collision import CollisionChecker

        self.check_id += 1
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
            return job, self.checker.check_path(joints, axes_at, every=CHECK_EVERY)

        self.check_future = self.pool.submit(work)
        self.poll_timer.start()

    def _poll_check(self):
        if self.check_future is None or not self.check_future.done():
            return
        self.poll_timer.stop()
        future, self.check_future = self.check_future, None
        try:
            out = future.result()
        except Exception as e:
            self.collision_text = f"<span style='color:#d93025'>collision check failed: {e}</span>"
            self._update_info()
            return
        if out is None or out[0] != self.check_id:
            return
        hits = out[1]
        self.scene.set_collisions(hits)
        self.plotter.render()
        if hits:
            pairs = sorted({p for h in hits for p in h.pairs})
            shown = ", ".join(f"{a}/{b}" for a, b in pairs[:4]) + (" ..." if len(pairs) > 4 else "")
            self.collision_text = (f"<span style='color:#c2185b'>{len(hits)} collision points</span> "
                                   f"(magenta): {shown}")
        else:
            self.collision_text = "<span style='color:green'>No collisions found</span>"
        self.collisions = hits
        self._update_info()
        self._mark_done(SIMULATE)

    # --- playback -------------------------------------------------------------
    def _on_slider(self, i):
        if self.scene and self.scene.mode == "preview":
            self.scene.show_point(i)
            self.plotter.render()

    def _toggle_play(self):
        if self.play_timer.isActive():
            self.play_timer.stop()
            self.play_btn.setText("Play")
        elif self.scene and self.scene.n_points:
            self.play_timer.start()
            self.play_btn.setText("Pause")

    def _tick(self):
        i = self.slider.value() + 1
        if i > self.slider.maximum():
            self._toggle_play()
            return
        self.slider.setValue(i)
