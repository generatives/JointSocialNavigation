from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .agents import Crowd, MCTSRobotAI, Robot, RobotAI
from .map import ScenarioMap

CONTROL_MODES = ("MANUAL", "ROBOT_AI", "MCTS_ROBOT_AI")


@dataclass(slots=True)
class StepMetrics:
    collided_with_wall: bool
    robot_human_collisions: int
    robot_social_force_generated: float
    distance_travelled: float


class NavigationSimulation:
    def __init__(
        self,
        *,
        control_mode: str = "ROBOT_AI",
        max_humans: int = 220,
        scenario: ScenarioMap | None = None,
    ) -> None:
        self.scenario = scenario or ScenarioMap.build_hallway_crossing()
        robot_start = self.scenario.cell_to_world(self.scenario.robot_start)
        self.robot = Robot(position=robot_start.copy(), theta=0.0)
        self.robot_ai = RobotAI(self.scenario)
        self.crowd = Crowd(max_humans=max_humans, scenario=self.scenario)
        self.mcts_robot_ai = MCTSRobotAI(self.scenario, self.crowd)
        self.control_modes = CONTROL_MODES
        self.control_mode_idx = self.control_modes.index(control_mode)
        self.sim_time = 0.0

    @property
    def control_mode(self) -> str:
        return self.control_modes[self.control_mode_idx]

    @property
    def goal(self) -> tuple[int, int] | None:
        goal = self.robot_ai.manual_goal
        if goal is not None:
            return goal
        return self.mcts_robot_ai.manual_goal

    def cycle_control_mode(self) -> tuple[str, str]:
        prev_mode = self.control_mode
        self.control_mode_idx = (self.control_mode_idx + 1) % len(self.control_modes)
        next_mode = self.control_mode
        if prev_mode == "ROBOT_AI" and next_mode != "ROBOT_AI":
            self.robot.path = []
            self.robot.path_ptr = 0
        return prev_mode, next_mode

    def set_goal(self, cell: tuple[int, int]) -> None:
        self.robot_ai.set_manual_goal(cell)
        self.mcts_robot_ai.set_manual_goal(cell)
        self.robot.path = []
        self.robot.path_ptr = 0

    def clear_goal(self) -> None:
        self.robot_ai.clear_manual_goal()
        self.mcts_robot_ai.clear_manual_goal()
        self.robot.path = []
        self.robot.path_ptr = 0

    def goal_reached(self, goal: tuple[int, int] | None = None, tolerance: float = 0.6) -> bool:
        active_goal = goal if goal is not None else self.goal
        if active_goal is None:
            return False
        return np.linalg.norm(self.robot.position - self.scenario.cell_to_world(active_goal)) < tolerance

    def warmup(self, seconds: float, *, dt: float = 1.0 / 30.0) -> None:
        steps = max(0, int(seconds / dt))
        for _ in range(steps):
            self.update(dt)

    def update(self, dt: float, manual_command: tuple[float, float] = (0.0, 0.0)) -> StepMetrics:
        mode = self.control_mode
        if mode == "ROBOT_AI":
            self.robot_ai.update(self.robot, dt)
        elif mode == "MCTS_ROBOT_AI":
            self.mcts_robot_ai.update(self.robot, dt)
        else:
            self.robot.command_v = manual_command[0]
            self.robot.command_w = manual_command[1]

        robot_metrics = self.robot.step(dt, self.scenario)
        crowd_metrics = self.crowd.update(dt, self.robot)
        self.sim_time += dt
        return StepMetrics(
            collided_with_wall=robot_metrics.collided_with_wall,
            robot_human_collisions=crowd_metrics.robot_human_collisions,
            robot_social_force_generated=crowd_metrics.robot_social_force_generated,
            distance_travelled=robot_metrics.distance_travelled,
        )
