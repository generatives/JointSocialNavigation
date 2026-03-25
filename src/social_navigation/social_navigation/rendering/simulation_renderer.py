import math
from typing import List

import numpy as np

from social_navigation.rendering.pygame_threaded import Circle, DrawCommand, Fill, Line, Lines, Present, Rect, Text
from social_navigation.simulator.constants import WALL
from social_navigation.simulator.simulation import NavigationSimulation


def draw_simulation(simulation: NavigationSimulation, cell_px: int) -> List[DrawCommand]:
    scenario = simulation.scenario
    crowd = simulation.crowd
    robot = simulation.robot

    commands = [Fill((237, 242, 245))]
    for y in range(scenario.height):
        for x in range(scenario.width):
            if scenario.grid[y, x] == WALL:
                commands.append(
                    Rect(
                        x=x * cell_px,
                        y=y * cell_px,
                        w=cell_px,
                        h=cell_px,
                        color=(37, 41, 44),
                    )
                )
            else:
                commands.append(
                    Rect(
                        x=x * cell_px,
                        y=y * cell_px,
                        w=cell_px,
                        h=cell_px,
                        color=(220, 226, 230),
                        width=1,
                    )
                )

    _draw_markers(simulation, commands, scenario.human_starts, (44, 120, 230), cell_px)
    _draw_markers(simulation, commands, scenario.human_ends, (61, 184, 112), cell_px)
    if simulation.goal is not None:
        _draw_markers(simulation, commands, [simulation.goal], (235, 120, 50), cell_px)

    active_idxs = np.flatnonzero(crowd.active)
    for idx in active_idxs:
        _draw_circle(simulation, commands, crowd.positions[idx], float(crowd.radius[idx]), (58, 138, 246), cell_px)

    if robot.path and len(robot.path) >= 2:
        points = [_to_px(scenario.cell_to_world(c), cell_px) for c in robot.path]
        commands.append(Lines(points=points, color=(201, 85, 73), closed=False, width=2))

    _draw_circle(simulation, commands, robot.position, robot.radius, (212, 63, 44), cell_px)
    heading = robot.position + robot.forward() * (robot.radius + 0.45)
    commands.append(Line(start=_to_px(robot.position, cell_px), end=_to_px(heading, cell_px), color=(24, 27, 28), width=3))

    mode = simulation.control_mode
    manual_goal = "None" if simulation.goal is None else f"{simulation.goal[0]},{simulation.goal[1]}"
    commands.append(
        Text(
            text=f"Mode: {mode} | TAB cycle | LMB set AI goal | RMB clear | goal: {manual_goal}",
            pos=(10, 8),
            color=(12, 12, 12),
        )
    )

    if simulation.control_mode == "MCTS_ROBOT_AI":
        if simulation.mcts_robot_ai.planned_robot_trajectory:
            trajectory_length = len(simulation.mcts_robot_ai.planned_robot_trajectory)
            for i in range(0, trajectory_length - 1, 2):
                start = simulation.mcts_robot_ai.planned_robot_trajectory[i]
                end = simulation.mcts_robot_ai.planned_robot_trajectory[i+1]
                commands.append(Line(start=_to_px(start, cell_px), end=_to_px(end, cell_px), color=(201, 85, 73), width=3))

        if simulation.mcts_robot_ai.planned_human_trajectories:
            for trajectory in simulation.mcts_robot_ai.planned_human_trajectories.values():
                trajectory_length = len(simulation.mcts_robot_ai.planned_robot_trajectory)
                for i in range(0, trajectory_length - 1, 2):
                    start = trajectory[i]
                    end = trajectory[i+1]
                    commands.append(Line(start=_to_px(start, cell_px), end=_to_px(end, cell_px), color=(58, 138, 246), width=3))

        if simulation.mcts_robot_ai.planned_human_goal_estimates:
            for goal_pos in simulation.mcts_robot_ai.planned_human_goal_estimates.values():
                _draw_circle(simulation, commands, goal_pos, 0.18, (255, 165, 0), cell_px)

    commands.append(Present())
    return commands

def _draw_markers(
    simulation: NavigationSimulation,
    commands: list[DrawCommand],
    cells: list[tuple[int, int]],
    color: tuple[int, int, int],
    cell_px: int,
) -> None:
    for cell in cells:
        center = _to_px(simulation.scenario.cell_to_world(cell), cell_px)
        commands.append(Circle(center=center, radius=cell_px // 4, color=color, width=2))

def _draw_circle(
    simulation: NavigationSimulation,
    commands: list[DrawCommand],
    world_pos: np.ndarray,
    world_radius: float,
    color: tuple[int, int, int],
    cell_px: int,
) -> None:
    commands.append(
        Circle(
            center=_to_px(world_pos, cell_px),
            radius=max(2, int(round(world_radius * cell_px))),
            color=color,
        )
    )

def _to_px(world_pos: np.ndarray, cell_px: int) -> tuple[int, int]:
    return int(round(world_pos[0] * cell_px)), int(round(world_pos[1] * cell_px))