from __future__ import annotations

import math
import time

import numpy as np

from .constants import WALL
from .pygame_threaded import (
    Circle,
    DrawCommand,
    Fill,
    InputSnapshot,
    Line,
    Lines,
    Present,
    Rect,
    Text,
    ThreadedPygameRuntime,
)
from .simulation import NavigationSimulation


class Simulator:
    def __init__(self) -> None:
        self.simulation = NavigationSimulation(control_mode="ROBOT_AI")
        self.cell_px = 24
        self.runtime = ThreadedPygameRuntime(
            window_size=(self.simulation.scenario.width * self.cell_px, self.simulation.scenario.height * self.cell_px),
            title="Crowded Navigation Simulator",
            font_name="consolas",
            font_size=18,
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
                self._draw()
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

    def _draw(self) -> None:
        scenario = self.simulation.scenario
        crowd = self.simulation.crowd
        robot = self.simulation.robot

        commands = [Fill((237, 242, 245))]
        for y in range(scenario.height):
            for x in range(scenario.width):
                if scenario.grid[y, x] == WALL:
                    commands.append(
                        Rect(
                            x=x * self.cell_px,
                            y=y * self.cell_px,
                            w=self.cell_px,
                            h=self.cell_px,
                            color=(37, 41, 44),
                        )
                    )
                else:
                    commands.append(
                        Rect(
                            x=x * self.cell_px,
                            y=y * self.cell_px,
                            w=self.cell_px,
                            h=self.cell_px,
                            color=(220, 226, 230),
                            width=1,
                        )
                    )

        self._draw_markers(commands, scenario.human_starts, (44, 120, 230))
        self._draw_markers(commands, scenario.human_ends, (61, 184, 112))
        if self.simulation.goal is not None:
            self._draw_markers(commands, [self.simulation.goal], (235, 120, 50))

        active_idxs = np.flatnonzero(crowd.active)
        for idx in active_idxs:
            self._draw_circle(commands, crowd.positions[idx], float(crowd.radius[idx]), (58, 138, 246))

        if robot.path and len(robot.path) >= 2:
            points = [self._to_px(scenario.cell_to_world(c)) for c in robot.path]
            commands.append(Lines(points=points, color=(201, 85, 73), closed=False, width=2))

        self._draw_circle(commands, robot.position, robot.radius, (212, 63, 44))
        heading = robot.position + robot.forward() * (robot.radius + 0.45)
        commands.append(Line(start=self._to_px(robot.position), end=self._to_px(heading), color=(24, 27, 28), width=3))

        mode = self.simulation.control_mode
        manual_goal = "None" if self.simulation.goal is None else f"{self.simulation.goal[0]},{self.simulation.goal[1]}"
        commands.append(
            Text(
                text=f"Mode: {mode} | TAB cycle | LMB set AI goal | RMB clear | goal: {manual_goal}",
                pos=(10, 8),
                color=(12, 12, 12),
            )
        )
        commands.append(Present())
        self.runtime.submit_frame(commands)

    def _draw_markers(
        self,
        commands: list[DrawCommand],
        cells: list[tuple[int, int]],
        color: tuple[int, int, int],
    ) -> None:
        for cell in cells:
            center = self._to_px(self.simulation.scenario.cell_to_world(cell))
            commands.append(Circle(center=center, radius=self.cell_px // 4, color=color, width=2))

    def _draw_circle(
        self,
        commands: list[DrawCommand],
        world_pos: np.ndarray,
        world_radius: float,
        color: tuple[int, int, int],
    ) -> None:
        commands.append(
            Circle(
                center=self._to_px(world_pos),
                radius=max(2, int(round(world_radius * self.cell_px))),
                color=color,
            )
        )

    def _to_px(self, world_pos: np.ndarray) -> tuple[int, int]:
        return int(round(world_pos[0] * self.cell_px)), int(round(world_pos[1] * self.cell_px))

    def _px_to_cell(self, px_pos: tuple[int, int]) -> tuple[int, int]:
        x = int(math.floor(px_pos[0] / self.cell_px))
        y = int(math.floor(px_pos[1] / self.cell_px))
        x = int(np.clip(x, 0, self.simulation.scenario.width - 1))
        y = int(np.clip(y, 0, self.simulation.scenario.height - 1))
        return x, y
