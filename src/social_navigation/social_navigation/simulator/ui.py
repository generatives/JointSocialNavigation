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
from .config import load_config
from .simulation import NavigationSimulation


class SimulatorUI:
    def __init__(self) -> None:
        config = load_config()
        self.simulation = NavigationSimulation(config=config)
        self.cell_px = config.cell_px
        self._warmup_seconds = config.warmup_seconds
        self.runtime = ThreadedPygameRuntime(
            window_size=(self.simulation.scenario.width * self.cell_px, self.simulation.scenario.height * self.cell_px),
            title="Crowded Navigation Simulator",
            font_name="consolas",
            font_size=18,
        )
        self.input_snapshot = InputSnapshot()
        self.show_beliefs = True
        # Running metrics for HUD display.
        self._cumulative_social_force = 0.0
        self._min_human_robot_distance = float("inf")
        self._robot_idle_time = 0.0

    def run(self) -> None:
        self.runtime.start()
        try:
            base_dt = 1.0 / 30.0
            self.simulation.warmup(self._warmup_seconds, dt=base_dt)

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
        if snapshot.b_pressed:
            self.show_beliefs = not self.show_beliefs
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
            metrics = self.simulation.update(dt, manual_command=(v, w))
        else:
            metrics = self.simulation.update(dt)
        # Accumulate metrics for HUD.
        self._cumulative_social_force += metrics.robot_social_force_generated
        if metrics.min_human_robot_distance < self._min_human_robot_distance:
            self._min_human_robot_distance = metrics.min_human_robot_distance
        if abs(self.simulation.robot.command_v) < 0.05:
            self._robot_idle_time += dt

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

        if self.show_beliefs:
            self._draw_beliefs(commands, crowd, active_idxs)

        for idx in active_idxs:
            self._draw_circle(commands, crowd.positions[idx], float(crowd.radius[idx]), (58, 138, 246))

        if robot.path and len(robot.path) >= 2:
            points = [self._to_px(scenario.cell_to_world(c)) for c in robot.path]
            commands.append(Lines(points=points, color=(201, 85, 73), closed=False, width=2))

        self._draw_circle(commands, robot.position, robot.radius, (212, 63, 44))
        heading = robot.position + robot.forward() * (robot.radius + 0.45)
        commands.append(Line(start=self._to_px(robot.position), end=self._to_px(heading), color=(24, 27, 28), width=3))

        # Draw yield target markers (green diamond) for yielding humans.
        for idx in active_idxs:
            if crowd.yielding[idx] and crowd.yield_target[idx] is not None:
                yt = crowd.yield_target[idx]
                center = self._to_px(scenario.cell_to_world(yt))
                commands.append(Circle(center=center, radius=self.cell_px // 4, color=(40, 180, 80), width=3))
                commands.append(Text(text="yield", pos=(center[0] + 5, center[1] - 18), color=(40, 150, 60)))

        mode = self.simulation.control_mode
        manual_goal = "None" if self.simulation.goal is None else f"{self.simulation.goal[0]},{self.simulation.goal[1]}"
        belief_indicator = "[B]eliefs ON" if self.show_beliefs else "[B]eliefs OFF"
        commands.append(
            Text(
                text=f"Mode: {mode} | TAB cycle | LMB set AI goal | RMB clear | goal: {manual_goal} | {belief_indicator}",
                pos=(10, 8),
                color=(12, 12, 12),
            )
        )
        # Metrics HUD (second line).
        min_d = self._min_human_robot_distance
        min_d_str = f"{min_d:.2f}" if min_d < 1e6 else "--"
        commands.append(
            Text(
                text=f"SFM:{self._cumulative_social_force:.1f}  minDist:{min_d_str}  robotIdle:{self._robot_idle_time:.1f}s",
                pos=(10, 30),
                color=(80, 80, 80),
            )
        )

        if self.simulation.control_mode == "JOINT_ASTAR_ROBOT_AI":
            ai = self.simulation.joint_astar_robot_ai
            if ai.planned_robot_path and len(ai.planned_robot_path) >= 2:
                points = [self._to_px(self.simulation.scenario.cell_to_world(c)) for c in ai.planned_robot_path]
                commands.append(Lines(points=points, color=(201, 85, 73), closed=False, width=2))
            if ai.planned_human_path and len(ai.planned_human_path) >= 2:
                points = [self._to_px(self.simulation.scenario.cell_to_world(c)) for c in ai.planned_human_path]
                commands.append(Lines(points=points, color=(58, 138, 246), closed=False, width=2))

        if self.simulation.control_mode == "MCTS_ROBOT_AI":
            if self.simulation.mcts_robot_ai.planned_robot_trajectory:
                trajectory_length = len(self.simulation.mcts_robot_ai.planned_robot_trajectory)
                for i in range(trajectory_length - 1):
                    start = self.simulation.mcts_robot_ai.planned_robot_trajectory[i]
                    end = self.simulation.mcts_robot_ai.planned_robot_trajectory[i+1]
                    commands.append(Line(start=self._to_px(start), end=self._to_px(end), color=(201, 85, 73), width=3))

            if self.simulation.mcts_robot_ai.planned_human_trajectories:
                for trajectory in self.simulation.mcts_robot_ai.planned_human_trajectories.values():
                    trajectory_length = len(self.simulation.mcts_robot_ai.planned_robot_trajectory)
                    for i in range(trajectory_length - 1):
                        start = trajectory[i]
                        end = trajectory[i+1]
                        commands.append(Line(start=self._to_px(start), end=self._to_px(end), color=(58, 138, 246), width=3))

            if self.simulation.mcts_robot_ai.planned_human_goal_estimates:
                for goal_pos in self.simulation.mcts_robot_ai.planned_human_goal_estimates.values():
                    self._draw_circle(commands, goal_pos, 0.18, (255, 165, 0))


        commands.append(Present())
        self.runtime.submit_frame(commands)

    def _draw_beliefs(
        self,
        commands: list[DrawCommand],
        crowd,
        active_idxs: np.ndarray,
    ) -> None:
        """Visualize each human's Bayesian belief about the robot's goal.

        For every active human, draw a line from the human to each candidate
        goal.  The line colour and width are proportional to the belief
        probability: low probability → faint grey, high probability → vivid
        amber.  A text label near the human shows the dominant belief
        probability as a percentage.
        """
        candidates = crowd._robot_goal_candidates
        num_c = candidates.shape[0]
        if num_c == 0 or active_idxs.size == 0:
            return

        # Colour endpoints for lerp: grey (low) → amber (high).
        bg = (200, 200, 200)
        fg = (220, 110, 20)

        for idx in active_idxs:
            beliefs = crowd.robot_goal_beliefs[idx]
            human_px = self._to_px(crowd.positions[idx])

            for c_idx in range(num_c):
                p = float(beliefs[c_idx])
                if p < 0.05:
                    continue  # skip nearly-zero beliefs to reduce clutter
                goal_px = self._to_px(candidates[c_idx])
                color = (
                    int(bg[0] + p * (fg[0] - bg[0])),
                    int(bg[1] + p * (fg[1] - bg[1])),
                    int(bg[2] + p * (fg[2] - bg[2])),
                )
                width = max(1, round(p * 4))
                commands.append(Line(start=human_px, end=goal_px, color=color, width=width))

            # Show the best belief percentage as text above the human.
            best_p = float(beliefs.max())
            best_idx = int(np.argmax(beliefs))
            label = f"g{best_idx}:{best_p:.0%}"
            label_pos = (human_px[0] + 8, human_px[1] - 20)
            commands.append(Text(text=label, pos=label_pos, color=(160, 60, 0)))

        # Label each candidate goal with its index so the human labels match.
        for c_idx in range(num_c):
            goal_px = self._to_px(candidates[c_idx])
            commands.append(Text(text=f"g{c_idx}", pos=(goal_px[0] + 5, goal_px[1] - 18), color=(160, 60, 0)))

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
