import math
import random
from dataclasses import dataclass, field
from typing import Iterable, List

from mcts.decoupled_mcts import Action, GameStateProtocol, MCTSConfig, ValueMap
from simulator.constants import WALL
import numpy as np

from simulator.map import ScenarioMap


@dataclass(frozen=True, slots=True)
class MCTSGameStateConfig:
    mcts_config: MCTSConfig
    robot_speed: float
    dt: float
    angle: float
    uncomfortable_distance: float
    map: ScenarioMap
    robot_radius: float
    human_radius: float
    orientation_changes: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "orientation_changes",
            np.array([-self.angle, 0.0, self.angle], dtype=np.float32),
        )


class MCTSGameState(GameStateProtocol):

    __slots__ = (
        "config",
        "positions",
        "velocities",
        "agent_goal_positions",
        "depth",
        "_accumulated_value",
        "_is_terminal_cache"
    )

    def __init__(self,
                 positions: np.ndarray,
                 velocities: np.ndarray,
                 agent_goal_positions: np.ndarray,
                 accumulated_value: np.ndarray | None,
                 config: MCTSGameStateConfig,
                 depth: int):
        super().__init__()
        self.config = config

        self.positions = positions
        self.velocities = velocities
        self.agent_goal_positions = agent_goal_positions
        self._is_terminal_cache = None
        self.depth = depth
        
        if accumulated_value is None:
            accumulated_value = np.zeros((positions.shape[0],))
        self._accumulated_value = self._accumulate_value(accumulated_value)


    def legal_actions(self) -> Iterable[Iterable[Action]]:
        return self.config.mcts_config.legal_actions

    def apply_actions(self, actions: List[int]) -> "GameStateProtocol":

        human_velocities = self._calculate_human_velocities()

        robot_velocity = self.velocities[0, :]
        robot_orientation = np.arctan2(robot_velocity[1], robot_velocity[0])
        robot_new_orientation = robot_orientation + self.config.orientation_changes[actions[0]]

        new_velocities = np.empty_like(self.velocities)
        new_velocities[0, 0] = self.config.robot_speed * np.cos(robot_new_orientation)
        new_velocities[0, 1] = self.config.robot_speed * np.sin(robot_new_orientation)
        new_velocities[1:] = human_velocities

        new_positions = self.positions + self.config.dt * new_velocities

        return MCTSGameState(
            new_positions,
            new_velocities,
            self.agent_goal_positions,
            self._accumulated_value,
            self.config,
            self.depth + 1
        )

    def is_terminal(self) -> bool:
        if self._is_terminal_cache is None:
            collided_with_wall = any(
                (not self.config.map.position_is_free(self.positions[i, :]))
                for i in range(self.config.mcts_config.num_actors)
            )
            reached_depth = self.depth >= self.config.mcts_config.max_depth
            self._is_terminal_cache = collided_with_wall or reached_depth

        return self._is_terminal_cache
    
    def _uncomfortable_distance(self) -> np.ndarray:
        robot_position = self.positions[0, :]
        other_positions = self.positions[1:, :]
        distances = np.linalg.norm(other_positions - robot_position, axis=1)
        uncomfortable_distances = np.clip(distances, 0, self.config.uncomfortable_distance)
        return np.sum(uncomfortable_distances)
    
    def _goal_distance(self) -> np.ndarray:
        return np.linalg.norm(self.agent_goal_positions - self.positions, axis=1)
    
    def _accumulate_value(self, value_accumulator) -> np.ndarray:
        value_accumulator = value_accumulator.copy()
        value_accumulator[0] += self._uncomfortable_distance()
        if self.is_terminal():
            value_accumulator += -self._goal_distance()
        return value_accumulator

    def terminal_values(self) -> ValueMap:
        return self._accumulated_value.tolist()
    
    def _calculate_human_velocities(self):
        human_preferred_speed = 1.7
        dt = self.config.dt

        human_positions = self.positions[1:, :]
        human_velocities = self.velocities[1:, :].copy()
        human_goal_positions = self.agent_goal_positions[1:, :]
        num_humans = human_positions.shape[0]

        desired = np.zeros((num_humans, 2), dtype=np.float32)
        for i in range(num_humans):
            to_target = human_goal_positions[i] - human_positions[i]
            dist = np.linalg.norm(to_target)
            if dist > 1e-6:
                desired[i] = (to_target / dist) * human_preferred_speed

        relaxation_time = 0.45
        accel = (desired - human_velocities) / relaxation_time
        social_forces, force_generated = self._social_forces(human_positions)
        accel += social_forces
        human_velocities += accel * dt

        speed = np.linalg.norm(human_velocities, axis=1)
        max_speed = human_preferred_speed * 1.7
        too_fast = speed > max_speed
        if np.any(too_fast):
            human_velocities *= (max_speed / speed[too_fast])[:, None]

        return human_velocities

    def _social_forces(self, human_positions) -> tuple[np.ndarray, float]:
        n = human_positions.shape[0]
        forces = np.zeros((n, 2), dtype=np.float32)
        robot_social_force_generated = 0.0

        a_h = 6.0
        b_h = 0.7
        a_obs = 3.2
        b_obs = 0.7

        for i in range(n):
            for j in range(i + 1, n):
                diff = human_positions[i] - human_positions[j]
                dist = np.linalg.norm(diff)
                if dist < 1e-4:
                    continue
                direction = diff / dist
                penetration = self.config.human_radius + self.config.human_radius - dist
                mag = a_h * math.exp((self.config.human_radius + self.config.human_radius - dist) / b_h)
                if penetration > 0.0:
                    mag += penetration * 25.0
                force = direction * mag
                forces[i] += force
                forces[j] -= force

        robot_pos = self.positions[0, :]
        for i in range(n):
            diff = human_positions[i] - robot_pos
            dist = np.linalg.norm(diff)
            if dist < 1e-4:
                continue
            direction = diff / dist
            combined = self.config.human_radius + self.config.robot_radius
            penetration = combined - dist
            mag = 10.0 * math.exp((combined - dist) / 0.6)
            if penetration > 0.0:
                mag += penetration * 30.0
            force = direction * mag
            forces[i] += force
            robot_social_force_generated += float(np.linalg.norm(force))

        for i in range(n):
            cell = self.config.map.world_to_cell(human_positions[i])
            for oy in range(-2, 3):
                for ox in range(-2, 3):
                    cx, cy = cell[0] + ox, cell[1] + oy
                    if cx < 0 or cx >= self.config.map.width or cy < 0 or cy >= self.config.map.height:
                        continue
                    if self.config.map.grid[cy, cx] != WALL:
                        continue
                    obstacle_pos = np.array([float(cx) + 0.5, float(cy) + 0.5], dtype=np.float32)
                    diff = human_positions[i] - obstacle_pos
                    dist = np.linalg.norm(diff)
                    if dist < 1e-4:
                        continue
                    direction = diff / dist
                    mag = a_obs * math.exp((self.config.human_radius + 0.5 - dist) / b_obs)
                    forces[i] += direction * mag

        return forces, robot_social_force_generated
    

def navigation_rollout(state: MCTSGameState):
    while not state.is_terminal():
        legal_actions = state.legal_actions()
        random_actions = [actions[random.randint(0, len(actions) - 1)] for actions in legal_actions]
        state = state.apply_actions(random_actions)
    return state.terminal_values()
