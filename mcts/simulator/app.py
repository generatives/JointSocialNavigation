from __future__ import annotations

import math
import time

import numpy as np

from .agents import Crowd, MCTSRobotAI, Robot, RobotAI
from .constants import WALL
from .map import ScenarioMap
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


class Simulator:
    def __init__(self) -> None:
        self.scenario = ScenarioMap.build_default()
        robot_start = self.scenario.cell_to_world(self.scenario.robot_start)
        self.robot = Robot(position=robot_start.copy(), theta=0.0)
        self.robot_ai = RobotAI(self.scenario)
        self.crowd = Crowd(max_humans=220, scenario=self.scenario)
        self.mcts_robot_ai = MCTSRobotAI(self.scenario, self.crowd)
        self.control_modes = ("MANUAL", "ROBOT_AI", "MCTS_ROBOT_AI")
        self.control_mode_idx = 1

        self.cell_px = 24
        self.runtime = ThreadedPygameRuntime(
            window_size=(self.scenario.width * self.cell_px, self.scenario.height * self.cell_px),
            title="Crowded Navigation Simulator",
            font_name="consolas",
            font_size=18,
        )
        self.input_snapshot = InputSnapshot()

    def run(self) -> None:
        self.runtime.start()
        try:
            # Run for a few seconds to get some people in the scene
            base_dt = 1.0 / 30.0
            for _ in range(int(1.0 / base_dt * 6)):
                self._update(base_dt)

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
            prev_mode = self.control_modes[self.control_mode_idx]
            self.control_mode_idx = (self.control_mode_idx + 1) % len(self.control_modes)
            next_mode = self.control_modes[self.control_mode_idx]
            if prev_mode == "ROBOT_AI" and next_mode != "ROBOT_AI":
                self.robot.path = []
                self.robot.path_ptr = 0
        if self.control_modes[self.control_mode_idx] != "MANUAL":
            for click in snapshot.mouse_clicks:
                clicked = self._px_to_cell(click.pos)
                if click.button == 1 and self.scenario.is_free(clicked):
                    self.robot_ai.set_manual_goal(clicked)
                    self.mcts_robot_ai.set_manual_goal(clicked)
                    self.robot.path = []
                    self.robot.path_ptr = 0
                if click.button == 3:
                    self.robot_ai.clear_manual_goal()
                    self.mcts_robot_ai.clear_manual_goal()
                    self.robot.path = []
                    self.robot.path_ptr = 0
        return True

    def _update(self, dt: float) -> None:
        mode = self.control_modes[self.control_mode_idx]
        if mode == "ROBOT_AI":
            self.robot_ai.update(self.robot, dt)
        elif mode == "MCTS_ROBOT_AI":
            self.mcts_robot_ai.update(self.robot, dt)
        else:
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
            self.robot.command_v = v
            self.robot.command_w = w

        self.robot.step(dt, self.scenario)
        self.crowd.update(dt, self.robot)

    def _draw(self) -> None:
        commands = [Fill((237, 242, 245))]
        for y in range(self.scenario.height):
            for x in range(self.scenario.width):
                if self.scenario.grid[y, x] == WALL:
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

        self._draw_markers(commands, self.scenario.human_starts, (44, 120, 230))
        self._draw_markers(commands, self.scenario.human_ends, (61, 184, 112))
        goal = self.robot_ai.manual_goal if self.robot_ai.manual_goal is not None else self.mcts_robot_ai.manual_goal
        if goal is not None:
            self._draw_markers(commands, [goal], (235, 120, 50))

        active_idxs = np.flatnonzero(self.crowd.active)
        for idx in active_idxs:
            self._draw_circle(commands, self.crowd.positions[idx], float(self.crowd.radius[idx]), (58, 138, 246))

        if self.robot.path and len(self.robot.path) >= 2:
            points = [self._to_px(self.scenario.cell_to_world(c)) for c in self.robot.path]
            commands.append(Lines(points=points, color=(201, 85, 73), closed=False, width=2))

        self._draw_circle(commands, self.robot.position, self.robot.radius, (212, 63, 44))
        heading = self.robot.position + self.robot.forward() * (self.robot.radius + 0.45)
        commands.append(
            Line(start=self._to_px(self.robot.position), end=self._to_px(heading), color=(24, 27, 28), width=3)
        )

        mode = self.control_modes[self.control_mode_idx]
        manual_goal = "None" if goal is None else f"{goal[0]},{goal[1]}"
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
            center = self._to_px(self.scenario.cell_to_world(cell))
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
        x = int(np.clip(x, 0, self.scenario.width - 1))
        y = int(np.clip(y, 0, self.scenario.height - 1))
        return x, y
