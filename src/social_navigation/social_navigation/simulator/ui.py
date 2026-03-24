from __future__ import annotations

import math
from pathlib import Path
import time

import numpy as np

from social_navigation.social_navigation.rendering.simulation_renderer import draw_simulation

from .constants import WALL
from ..rendering.pygame_threaded import (
    InputSnapshot,
    ThreadedPygameRuntime,
)
from .simulation import NavigationSimulation


class SimulatorUI:
    def __init__(self, recording_path: str | Path | None = None, recording_fps: int = 30) -> None:
        self.simulation = NavigationSimulation(control_mode="ROBOT_AI")
        self.cell_px = 24
        self.runtime = ThreadedPygameRuntime(
            window_size=(self.simulation.scenario.width * self.cell_px, self.simulation.scenario.height * self.cell_px),
            title="Crowded Navigation Simulator",
            font_name="consolas",
            font_size=18,
            recording_path=recording_path,
            recording_fps=recording_fps,
        )
        self.input_snapshot = InputSnapshot()

    def run(self) -> None:
        self.runtime.start()
        try:
            base_dt = 1.0 / 30.0
            self.simulation.warmup(6.0, dt=base_dt)

            running = True
            frame_period = 1.0 / 30.0
            prev_time = time.perf_counter()
            while running:
                now = time.perf_counter()
                elapsed = now - prev_time
                if elapsed < frame_period:
                    time.sleep(frame_period - elapsed)
                    now = time.perf_counter()
                    elapsed = now - prev_time
                dt = min(elapsed, frame_period)
                prev_time = now
                self.input_snapshot = self.runtime.poll_input()
                running = self._handle_events(self.input_snapshot)
                self._update(dt)
                commands = draw_simulation(self.simulation, self.cell_px)
                self.runtime.submit_frame(commands)
        finally:
            self.runtime.stop()

    def _handle_events(self, snapshot: InputSnapshot) -> bool:
        if snapshot.quit_requested:
            return False
        if snapshot.tab_pressed:
            self.simulation.cycle_control_mode()
        if self.simulation.control_mode != "MANUAL":
            for click in snapshot.mouse_clicks:
                clicked = self._px_to_cell(click.pos)
                if click.button == 1 and self.simulation.scenario.is_free(clicked):
                    self.simulation.set_goal(clicked)
                if click.button == 3:
                    self.simulation.clear_goal()
        return True

    def _update(self, dt: float) -> None:
        if self.simulation.control_mode == "MANUAL":
            v = 0.0
            w = 0.0
            if self.input_snapshot.up_pressed:
                v += 2.0
            if self.input_snapshot.down_pressed:
                v -= 1.0
            if self.input_snapshot.left_pressed:
                w -= 2.5
            if self.input_snapshot.right_pressed:
                w += 2.5
            self.simulation.update(dt, manual_command=(v, w))
            return
        self.simulation.update(dt)


    def _px_to_cell(self, px_pos: tuple[int, int]) -> tuple[int, int]:
        x = int(math.floor(px_pos[0] / self.cell_px))
        y = int(math.floor(px_pos[1] / self.cell_px))
        x = int(np.clip(x, 0, self.simulation.scenario.width - 1))
        y = int(np.clip(y, 0, self.simulation.scenario.height - 1))
        return x, y