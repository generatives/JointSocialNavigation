from __future__ import annotations

import argparse
import random
import time

import numpy as np

from social_navigation.mcts.decoupled_mcts import MCTSConfig
from social_navigation.simulator.constants import WALL
from social_navigation.simulator.mcts_game_state import MCTSGameState, MCTSGameStateConfig
from social_navigation.social_navigation.rendering.pygame_threaded import Circle, Fill, InputSnapshot, Line, Present, Rect, Text, ThreadedPygameRuntime
from social_navigation.simulator.scenario_map import ScenarioMap


class MCTSActionExplorer:
    def __init__(self) -> None:
        self.cell_px = 28
        self.sidebar_px = 460
        self.maps = [
            ("Empty", ScenarioMap.build_empty()),
            ("Default", ScenarioMap.build_default()),
            ("Crossing", ScenarioMap.build_hallway_crossing()),
            ("Collision", ScenarioMap.build_hallway_collision()),
            ("Tradeoff", ScenarioMap.build_hallway_tradeoff()),
        ]
        self.map_index = 0
        self.map_origin_px = (24, 24)
        self.max_map_width = max(scenario.width for _, scenario in self.maps)
        self.max_map_height = max(scenario.height for _, scenario in self.maps)

        self.runtime = ThreadedPygameRuntime(
            window_size=(
                self.map_origin_px[0] * 2 + self.max_map_width * self.cell_px + self.sidebar_px,
                self.map_origin_px[1] * 2 + self.max_map_height * self.cell_px,
            ),
            title="MCTS Action Probability Explorer",
            font_name="consolas",
            font_size=18,
        )
        self.input_snapshot = InputSnapshot()
        self.robot_heading = 0.0
        self.robot_position = np.zeros(2, dtype=np.float32)
        self.goal_position = np.zeros(2, dtype=np.float32)
        self.progress_weight = 1.0
        self.heading_weight = 0.3
        self.weight_step = 0.1
        self._set_positions_from_map()

    @property
    def scenario(self) -> ScenarioMap:
        return self.maps[self.map_index][1]

    @property
    def map_name(self) -> str:
        return self.maps[self.map_index][0]

    def run(self) -> None:
        self.runtime.start()
        try:
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
                prev_time = now
                self.input_snapshot = self.runtime.poll_input()
                running = self._handle_events(min(elapsed, frame_period))
                self._draw()
        finally:
            self.runtime.stop()

    def _set_positions_from_map(self) -> None:
        scenario = self.scenario
        self.robot_position = scenario.cell_to_world(scenario.robot_start)
        if scenario.robot_goals:
            self.goal_position = scenario.cell_to_world(scenario.robot_goals[0])
        else:
            fallback_goal = scenario.nearest_free((scenario.width - 2, scenario.height // 2))
            if fallback_goal is None:
                fallback_goal = scenario.robot_start
            self.goal_position = scenario.cell_to_world(fallback_goal)

        goal_vector = self.goal_position - self.robot_position
        goal_norm = np.linalg.norm(goal_vector)
        self.robot_heading = float(np.arctan2(goal_vector[1], goal_vector[0])) if goal_norm > 1e-9 else 0.0

    def _handle_events(self, dt: float) -> bool:
        if self.input_snapshot.quit_requested:
            return False
        if self.input_snapshot.tab_pressed:
            self.map_index = (self.map_index + 1) % len(self.maps)
            self._set_positions_from_map()

        rotation_speed = 1.8
        if self.input_snapshot.left_pressed:
            self.robot_heading -= rotation_speed * dt
        if self.input_snapshot.right_pressed:
            self.robot_heading += rotation_speed * dt

        for click in self.input_snapshot.mouse_clicks:
            if self._handle_sidebar_click(click.pos, click.button):
                continue
            cell = self._px_to_cell(click.pos)
            if cell is None:
                continue
            free_cell = self.scenario.nearest_free(cell)
            if free_cell is None:
                continue
            world = self.scenario.cell_to_world(free_cell)
            if click.button == 1:
                self.robot_position = world
            elif click.button == 3:
                self.goal_position = world
        return True

    def _build_state(self) -> MCTSGameState:
        direction = np.array([np.cos(self.robot_heading), np.sin(self.robot_heading)], dtype=np.float32)
        robot_speed = 1.0
        positions = np.array([self.robot_position], dtype=np.float32)
        velocities = np.array([direction * robot_speed], dtype=np.float32)
        goals = np.array([self.goal_position], dtype=np.float32)
        starting_distances = np.linalg.norm(goals - positions, axis=1)

        config = MCTSGameStateConfig(
            mcts_config=MCTSConfig(
                num_actors=1,
                max_actions=(4,),
                rng=random.Random(7),
                max_depth=6,
            ),
            robot_speed=robot_speed,
            dt=0.8,
            robot_angular_velocity=1.1,
            uncomfortable_distance=1.5,
            map=self.scenario,
            robot_radius=0.35,
            human_radius=0.35,
            starting_distances=starting_distances,
            action_progress_weight=self.progress_weight,
            action_heading_weight=self.heading_weight,
        )
        return MCTSGameState(
            positions=positions,
            velocities=velocities,
            agent_goal_positions=goals,
            accumulated_value=None,
            config=config,
            depth=0,
        )

    def _draw(self) -> None:
        state = self._build_state()
        action_probabilities = list(state.legal_actions())[0]
        commands = [Fill((244, 242, 237))]
        commands.extend(self._draw_map())
        commands.extend(self._draw_sidebar(state, action_probabilities))

        self._draw_world_circle(commands, self.goal_position, 0.22, (58, 138, 246), width=2)
        self._draw_world_circle(commands, self.robot_position, state.config.robot_radius, (212, 63, 44))

        heading_end = self.robot_position + np.array(
            [np.cos(self.robot_heading), np.sin(self.robot_heading)],
            dtype=np.float32,
        ) * 0.9
        commands.append(Line(start=self._to_px(self.robot_position), end=self._to_px(heading_end), color=(24, 27, 28), width=3))

        action_colors = [
            (224, 122, 95),
            (61, 64, 91),
            (129, 178, 154),
            (242, 204, 143),
        ]
        for action_idx, (action, probability) in enumerate(action_probabilities):
            command_speed, command_omega = state.get_command_velocities(action)
            x_new, y_new, _ = state._propagate_unicycle(
                self.robot_position[0],
                self.robot_position[1],
                self.robot_heading,
                command_speed,
                command_omega,
                state.config.dt,
            )
            projected = np.array([x_new, y_new], dtype=np.float32)
            color = action_colors[action_idx]
            commands.append(Line(start=self._to_px(self.robot_position), end=self._to_px(projected), color=color, width=3))
            self._draw_world_circle(commands, projected, 0.14, color)
            label_anchor = projected + np.array([0.12, -0.2], dtype=np.float32)
            commands.append(Text(text=f"{probability:.2f}", pos=self._to_px(label_anchor), color=color))

        commands.append(Present())
        self.runtime.submit_frame(commands)

    def _draw_map(self) -> list:
        scenario = self.scenario
        commands = []
        ox, oy = self.map_origin_px
        map_w_px = scenario.width * self.cell_px
        map_h_px = scenario.height * self.cell_px
        commands.append(Rect(x=ox - 2, y=oy - 2, w=map_w_px + 4, h=map_h_px + 4, color=(70, 69, 66), width=2))
        for y in range(scenario.height):
            for x in range(scenario.width):
                if scenario.grid[y, x] == WALL:
                    commands.append(
                        Rect(
                            x=ox + x * self.cell_px,
                            y=oy + y * self.cell_px,
                            w=self.cell_px,
                            h=self.cell_px,
                            color=(63, 68, 73),
                        )
                    )
                else:
                    commands.append(
                        Rect(
                            x=ox + x * self.cell_px,
                            y=oy + y * self.cell_px,
                            w=self.cell_px,
                            h=self.cell_px,
                            color=(206, 201, 191),
                            width=1,
                        )
                    )
        return commands

    def _weight_control_rects(self, x0: int, y0: int) -> dict[str, tuple[int, int, int, int]]:
        button_size = 28
        value_x = x0 + 118
        plus_x = x0 + 208
        second_row_y = y0 + 40
        return {
            "progress_minus": (x0, y0, button_size, button_size),
            "progress_plus": (plus_x, y0, button_size, button_size),
            "heading_minus": (x0, second_row_y, button_size, button_size),
            "heading_plus": (plus_x, second_row_y, button_size, button_size),
            "progress_value": (value_x, y0, 72, button_size),
            "heading_value": (value_x, second_row_y, 72, button_size),
        }

    def _handle_sidebar_click(self, pos: tuple[int, int], button: int) -> bool:
        if button != 1:
            return False
        x0 = self.map_origin_px[0] + self.max_map_width * self.cell_px + 28
        y0 = self.map_origin_px[1] + 244
        rects = self._weight_control_rects(x0, y0)
        for key, rect in rects.items():
            if not key.endswith(("minus", "plus")):
                continue
            if self._point_in_rect(pos, rect):
                if key == "progress_minus":
                    self.progress_weight = max(0.0, round(self.progress_weight - self.weight_step, 2))
                elif key == "progress_plus":
                    self.progress_weight = min(3.0, round(self.progress_weight + self.weight_step, 2))
                elif key == "heading_minus":
                    self.heading_weight = max(0.0, round(self.heading_weight - self.weight_step, 2))
                elif key == "heading_plus":
                    self.heading_weight = min(3.0, round(self.heading_weight + self.weight_step, 2))
                return True
        return False

    def _draw_weight_controls(self, x0: int, y0: int, sidebar_width: int) -> list:
        rects = self._weight_control_rects(x0, y0)
        commands = [
            Text(text="Weights", pos=(x0, y0 - 26), color=(20, 20, 20)),
            Text(text="Progress", pos=(x0 + 40, y0 + 4), color=(20, 20, 20)),
            Text(text="Heading", pos=(x0 + 40, y0 + 44), color=(20, 20, 20)),
            Text(text="Higher heading weight prefers ending the step facing the goal", pos=(x0, y0 + 84), color=(60, 60, 60)),
        ]
        for key in ("progress_minus", "progress_plus", "heading_minus", "heading_plus"):
            x, y, w, h = rects[key]
            commands.append(Rect(x=x, y=y, w=w, h=h, color=(212, 208, 198), width=1))
            commands.append(Text(text='-' if key.endswith('minus') else '+', pos=(x + 9, y + 2), color=(20, 20, 20)))

        for key, value in (("progress_value", self.progress_weight), ("heading_value", self.heading_weight)):
            x, y, w, h = rects[key]
            commands.append(Rect(x=x, y=y, w=w, h=h, color=(212, 208, 198), width=1))
            commands.append(Text(text=f"{value:.1f}", pos=(x + 18, y + 2), color=(20, 20, 20)))

        return commands

    def _point_in_rect(self, pos: tuple[int, int], rect: tuple[int, int, int, int]) -> bool:
        x, y = pos
        rx, ry, rw, rh = rect
        return rx <= x < rx + rw and ry <= y < ry + rh

    def _draw_sidebar(self, state: MCTSGameState, action_probabilities: list[tuple[int, float]]) -> list:
        map_px_width = self.max_map_width * self.cell_px
        x0 = self.map_origin_px[0] + map_px_width + 28
        y = self.map_origin_px[1]
        sidebar_width = self.sidebar_px - 48
        bar_label_width = 72
        commands = [
            Text(text="MCTS Action Priors", pos=(x0, y), color=(20, 20, 20)),
            Text(text=f"Map: {self.map_name} (TAB to cycle)", pos=(x0, y + 32), color=(40, 40, 40)),
            Text(text="LMB move robot | RMB move goal", pos=(x0, y + 60), color=(40, 40, 40)),
            Text(text="Left/Right arrows rotate heading", pos=(x0, y + 88), color=(40, 40, 40)),
            Text(text="Use +/- buttons to tune heuristic weights", pos=(x0, y + 116), color=(40, 40, 40)),
        ]

        robot_cell = self.scenario.world_to_cell(self.robot_position)
        goal_cell = self.scenario.world_to_cell(self.goal_position)
        goal_distance = float(np.linalg.norm(self.goal_position - self.robot_position))
        commands.extend(
            [
                Text(text=f"Robot: {robot_cell}  heading: {np.degrees(self.robot_heading):.1f} deg", pos=(x0, y + 150), color=(20, 20, 20)),
                Text(text=f"Goal:  {goal_cell}  distance: {goal_distance:.2f} m", pos=(x0, y + 178), color=(20, 20, 20)),
                Text(text=f"dt: {state.config.dt:.2f}  speed: {state.config.robot_speed:.2f}  omega: {state.config.robot_angular_velocity:.2f}", pos=(x0, y + 206), color=(20, 20, 20)),
            ]
        )

        commands.extend(self._draw_weight_controls(x0, y + 244, sidebar_width))

        labels = ["Turn Left", "Straight", "Turn Right", "Stop"]
        colors = [
            (224, 122, 95),
            (61, 64, 91),
            (129, 178, 154),
            (242, 204, 143),
        ]
        bar_y = y + 360
        bar_w = sidebar_width - bar_label_width
        bar_h = 26
        for idx, ((action, probability), label) in enumerate(zip(action_probabilities, labels)):
            row_y = bar_y + idx * 62
            commands.append(Text(text=f"{action}: {label}", pos=(x0, row_y), color=(20, 20, 20)))
            commands.append(Rect(x=x0, y=row_y + 24, w=bar_w, h=bar_h, color=(212, 208, 198), width=1))
            commands.append(Rect(x=x0, y=row_y + 24, w=max(1, int(round(bar_w * probability))), h=bar_h, color=colors[idx]))
            commands.append(Text(text=f"{probability:.3f}", pos=(x0 + bar_w + 16, row_y + 24), color=(20, 20, 20)))

        return commands

    def _draw_world_circle(
        self,
        commands: list,
        world_pos: np.ndarray,
        world_radius: float,
        color: tuple[int, int, int],
        width: int = 0,
    ) -> None:
        commands.append(
            Circle(
                center=self._to_px(world_pos),
                radius=max(2, int(round(world_radius * self.cell_px))),
                color=color,
                width=width,
            )
        )

    def _to_px(self, world_pos: np.ndarray) -> tuple[int, int]:
        ox, oy = self.map_origin_px
        return int(round(ox + world_pos[0] * self.cell_px)), int(round(oy + world_pos[1] * self.cell_px))

    def _px_to_cell(self, px_pos: tuple[int, int]) -> tuple[int, int] | None:
        ox, oy = self.map_origin_px
        rel_x = px_pos[0] - ox
        rel_y = px_pos[1] - oy
        if rel_x < 0 or rel_y < 0:
            return None
        x = rel_x // self.cell_px
        y = rel_y // self.cell_px
        if x >= self.scenario.width or y >= self.scenario.height:
            return None
        return int(x), int(y)


def parse_args() -> argparse.Namespace:
    return argparse.ArgumentParser(description="Interactive explorer for MCTS action priors").parse_args()


def main() -> None:
    parse_args()
    random.seed(7)
    np.random.seed(7)
    MCTSActionExplorer().run()


if __name__ == "__main__":
    main()
