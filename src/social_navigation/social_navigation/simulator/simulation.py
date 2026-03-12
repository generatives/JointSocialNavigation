from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .agents import Crowd, MCTSRobotAI, Robot, RobotAI, JointAStarRobotAI
from .config import SimConfig, load_config
from .scenario_map import ScenarioMap

CONTROL_MODES = ("MANUAL", "ROBOT_AI", "MCTS_ROBOT_AI", "JOINT_ASTAR_ROBOT_AI")

_MAP_BUILDERS = {
    "empty": ScenarioMap.build_empty,
    "default": ScenarioMap.build_default,
    "hallway_crossing": ScenarioMap.build_hallway_crossing,
    "hallway_collision": ScenarioMap.build_hallway_collision,
    "narrow_hallway": ScenarioMap.build_narrow_hallway,
    "narrow_hallway2": ScenarioMap.build_narrow_hallway2,
    "narrow_hallway3": ScenarioMap.build_narrow_hallway3,
}


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
        config: SimConfig | None = None,
        scenario: ScenarioMap | None = None,
    ) -> None:
        if config is None:
            config = load_config()
        self.config = config

        if scenario is not None:
            self.scenario = scenario
        else:
            builder = _MAP_BUILDERS.get(config.map, ScenarioMap.build_hallway_crossing)
            self.scenario = builder()

        rx, ry, theta = self.scenario.robot_start
        robot_start = self.scenario.cell_to_world((rx, ry))
        self.robot = Robot(position=robot_start.copy(), theta=theta)
        self.robot_ai = RobotAI(self.scenario)
        self.crowd = Crowd(
            max_humans=config.max_humans,
            scenario=self.scenario,
            max_spawns=config.max_spawns,
            spawn_rate_per_sec=config.spawn_rate_per_sec,
            pref_speed_min=config.pref_speed_min,
            pref_speed_max=config.pref_speed_max,
            human_human_amplitude=config.human_human_amplitude,
            human_human_decay=config.human_human_decay,
            robot_human_amplitude=config.robot_human_amplitude,
            robot_human_decay=config.robot_human_decay,
            obstacle_amplitude=config.obstacle_amplitude,
            obstacle_decay=config.obstacle_decay,
            belief_sigma=config.belief_sigma,
            belief_yield_threshold=config.belief_yield_threshold,
            belief_lookahead=config.belief_lookahead,
        )
        self.mcts_robot_ai = MCTSRobotAI(self.scenario, self.crowd)
        self.joint_astar_robot_ai = JointAStarRobotAI(
            self.scenario,
            self.crowd,
            replan_period=config.replan_period,
            robot_move_penalty=config.robot_move_penalty,
            human_detour_penalty=config.human_detour_penalty,
            blocking_penalty=config.blocking_penalty,
            proximity_penalty=config.proximity_penalty,
            proximity_threshold=config.proximity_threshold,
            belief_discount_blocking=config.belief_discount_blocking,
        )
        self.control_modes = CONTROL_MODES
        self.control_mode_idx = self.control_modes.index(config.default_mode)
        self.sim_time = 0.0

    @property
    def control_mode(self) -> str:
        return self.control_modes[self.control_mode_idx]

    @property
    def goal(self) -> tuple[int, int] | None:
        for ai in (self.robot_ai, self.mcts_robot_ai, self.joint_astar_robot_ai):
            if ai.manual_goal is not None:
                return ai.manual_goal
        return None

    def cycle_control_mode(self) -> tuple[str, str]:
        prev_mode = self.control_mode
        self.control_mode_idx = (self.control_mode_idx + 1) % len(self.control_modes)
        next_mode = self.control_mode
        if prev_mode != next_mode:
            self.robot.path = []
            self.robot.path_ptr = 0
        return prev_mode, next_mode

    def set_goal(self, cell: tuple[int, int]) -> None:
        self.robot_ai.set_manual_goal(cell)
        self.mcts_robot_ai.set_manual_goal(cell)
        self.joint_astar_robot_ai.set_manual_goal(cell)
        self.robot.path = []
        self.robot.path_ptr = 0

    def clear_goal(self) -> None:
        self.robot_ai.clear_manual_goal()
        self.mcts_robot_ai.clear_manual_goal()
        self.joint_astar_robot_ai.clear_manual_goal()
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
        elif mode == "JOINT_ASTAR_ROBOT_AI":
            self.joint_astar_robot_ai.update(self.robot, dt)
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
