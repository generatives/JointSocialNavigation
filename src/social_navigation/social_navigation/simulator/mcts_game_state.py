import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Iterable
import numpy as np

from social_navigation.mcts.decoupled_mcts import Action, GameStateProtocol, MCTSConfig, ValueMap
from social_navigation.simulator.constants import WALL
from social_navigation.simulator.scenario_map import ScenarioMap

@dataclass(frozen=True, slots=True)
class MCTSGameStateConfig:
    mcts_config: MCTSConfig
    robot_speed: float
    dt: float
    robot_angular_velocity: float
    uncomfortable_distance: float
    map: ScenarioMap
    robot_radius: float
    human_radius: float
    starting_distances: float
    action_progress_weight: float = 0.3
    action_heading_weight: float = 1.0
    robot_angular_velocity_actions: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "robot_angular_velocity_actions",
            np.array([-self.robot_angular_velocity, 0.0, self.robot_angular_velocity], dtype=np.float32),
        )


class MCTSGameState(GameStateProtocol):
    __slots__ = (
        "config", "positions", "velocities", "agent_goal_positions",
        "depth", "_accumulated_value", "_collision_mask", "_collision_occured"
    )

    def __init__(self, positions: np.ndarray, velocities: np.ndarray,
                 agent_goal_positions: np.ndarray, accumulated_value: np.ndarray | None,
                 config: MCTSGameStateConfig, depth: int):
        self.config = config
        self.positions = positions
        self.velocities = velocities
        self.agent_goal_positions = agent_goal_positions
        self._collision_mask = None
        self._collision_occured = None
        self.depth = depth
        
        if accumulated_value is None:
            accumulated_value = np.zeros((positions.shape[0],))
        self._accumulated_value = self._accumulate_value(accumulated_value)

   
    def _score_robot_action(self, linear_velocity, rotational_velocity) -> float:
        robot_position = self.positions[0, :]
        robot_velocity = self.velocities[0, :]
        robot_orientation = np.arctan2(robot_velocity[1], robot_velocity[0])

        x_new, y_new, robot_new_orientation = self._propagate_unicycle(
            robot_position[0],
            robot_position[1],
            robot_orientation,
            linear_velocity,
            rotational_velocity,
            self.config.dt,
        )

        goal_vector = self.agent_goal_positions[0, :] - self.positions[0, :]
        goal_norm = np.linalg.norm(goal_vector)
        goal_direction = goal_vector / goal_norm if goal_norm > 1e-9 else np.zeros_like(goal_vector)
        goal_heading = math.atan2(goal_vector[1], goal_vector[0]) if goal_norm > 1e-9 else 0.0

        action_vector = np.array([x_new, y_new]) - self.positions[0, :]
        action_norm = np.linalg.norm(action_vector)
        if action_norm > 1e-9 and goal_norm > 1e-9:
            progress_alignment = float(np.dot(goal_direction, action_vector / action_norm))
        elif action_norm <= 1e-9 and goal_norm > 1e-9:
            progress_alignment = 0.0
        else:
            progress_alignment = 1.0

        heading_error = math.atan2(
            math.sin(goal_heading - robot_new_orientation),
            math.cos(goal_heading - robot_new_orientation),
        )
        heading_alignment = math.cos(heading_error) if goal_norm > 1e-9 else 1.0

        combined_alignment = (
            self.config.action_progress_weight * progress_alignment
            + self.config.action_heading_weight * heading_alignment
        )
        score = float(np.exp(combined_alignment - 1.0))
        return score

    def sample_action(self, actor_idx: int, rng: np.random.Generator, existing_actions: Optional[List[Tuple[Action, float]]] = None) -> Tuple[Action, float]:
        """Samples a new action for the actor, ensuring diversity within the pool and against existing expansions."""
        
        # To modify later  based on specific heuristics 
        v_mean, v_std =  0.9 * self.config.robot_speed, 0.5
        omega_mean, omega_std = 0.0, self.config.robot_angular_velocity * 0.5

        # We must normalize the axes because v and omega have different maximum scales
        v_scale = self.config.robot_speed if self.config.robot_speed > 0 else 1.0
        omega_scale = self.config.robot_angular_velocity if self.config.robot_angular_velocity > 0 else 1.0

        best_v, best_omega, score = 0.0, 0.0, 1.0

        if actor_idx == 0:
            # Robot (Actor 0)
            if not existing_actions:
                # First action has nothing to be diverse against
                best_v = float(np.clip(rng.normal(v_mean, v_std), 0.0, self.config.robot_speed))
                best_omega = float(np.clip(rng.normal(omega_mean, omega_std), -self.config.robot_angular_velocity, self.config.robot_angular_velocity))
            else:
                max_min_dist = -1.0
                num_candidates = 5 # Number of samples to draw from the distribution
                existing_arr = np.array([action for action, _ in existing_actions]) # Shape: (N, 2)
                
                for _ in range(num_candidates):
                    v_cand = float(np.clip(rng.normal(v_mean, v_std), 0.0, self.config.robot_speed))
                    omega_cand = float(np.clip(rng.normal(omega_mean, omega_std), -self.config.robot_angular_velocity, self.config.robot_angular_velocity))
                    
                    # Calculate scaled distance to all known actions
                    v_diffs = (existing_arr[:, 0] - v_cand) / v_scale
                    omega_diffs = (existing_arr[:, 1] - omega_cand) / omega_scale
                    dists = np.sqrt(v_diffs**2 + omega_diffs**2) # Eucledian distance
                    
                    min_tree_dist = float(np.min(dists))
                    
                    # Keep the sample that is furthest from already-explored actions
                    if min_tree_dist > max_min_dist:
                        max_min_dist = min_tree_dist
                        best_v, best_omega = v_cand, omega_cand

                # calculate heuristic score
                score = self._score_robot_action(best_v, best_omega)
    
        return (best_v, best_omega), score


    def _propagate_unicycle(self, x, y, theta, v, omega, dt, eps=1e-9):
        if abs(omega) < eps:
            x_new = x + v * dt * math.cos(theta)
            y_new = y + v * dt * math.sin(theta)
            theta_new = theta
        else:
            theta_new = theta + omega * dt
            x_new = x + (v / omega) * (math.sin(theta_new) - math.sin(theta))
            y_new = y - (v / omega) * (math.cos(theta_new) - math.cos(theta))

        return x_new, y_new, theta_new

    def apply_actions(self, actions: List[Tuple[Action, float]]) -> "GameStateProtocol":
        substeps = 2
        substep_dt = self.config.dt / substeps
        positions = self.positions.copy()
        velocities = self.velocities.copy()
        # The robot's continuous action is directly used
        robot_v, robot_omega = actions[0][0]

        for i in range(substeps):
            # TODO: change human velocities to acommodate multiple potential actions to humans
            human_velocities = self._calculate_human_velocities(positions, velocities)

            robot_position = positions[0, :]
            robot_velocity = velocities[0, :]
            robot_orientation = np.arctan2(robot_velocity[1], robot_velocity[0])

            x_new, y_new, robot_new_orientation = self._propagate_unicycle(
                robot_position[0], robot_position[1], robot_orientation,
                robot_v, robot_omega, substep_dt
            )

            velocities = np.empty_like(velocities)
            velocities[0, 0] = robot_v * np.cos(robot_new_orientation)
            velocities[0, 1] = robot_v * np.sin(robot_new_orientation)
            velocities[1:] = human_velocities

            positions = positions + substep_dt * velocities
            positions[0, 0] = x_new
            positions[0, 1] = y_new

        return MCTSGameState(
            positions, velocities, self.agent_goal_positions,
            self._accumulated_value, self.config, self.depth + 1
        )
    
    def _get_invalid_state(self) -> bool:
        if self._collision_occured is None:
            self._collision_mask = np.array([
                not self.config.map.position_is_free(self.positions[i, :])
                for i in range(self.config.mcts_config.num_actors)
            ])
            self._collision_occured = any(self._collision_mask)

        return self._collision_mask, self._collision_occured


    def is_terminal(self) -> bool:
        _, is_invalid = self._get_invalid_state()
        reached_depth = self.depth >= self.config.mcts_config.max_depth
        goal_distance = np.linalg.norm(self.agent_goal_positions[0] - self.positions[0])
        reached_goal = goal_distance < 0.25
        return is_invalid or reached_depth or reached_goal
    
    def _uncomfortable_distance(self) -> np.ndarray:
        robot_position = self.positions[0, :]
        other_positions = self.positions[1:, :]
        distances = np.linalg.norm(other_positions - robot_position, axis=1)
        uncomfortable_distances = np.clip(distances, 0, self.config.uncomfortable_distance)
        total_distance = np.sum(uncomfortable_distances)

        # We should scale the distances so that if the robot stays at least 1.5m away
        # from all actors throughout the plan the total score will be 1.0.
        # This makes it easier to trade this score against the goal reaching score
        total_possible_distance = self.config.uncomfortable_distance * \
            (self.config.mcts_config.num_actors - 1) * \
            self.config.mcts_config.max_depth
        score = total_distance / total_possible_distance
        return score
    
    def _uncomfortable_distance_meter_score(self) -> np.ndarray:
        robot_position = self.positions[0, :]
        other_positions = self.positions[1:, :]
        distances = np.linalg.norm(other_positions - robot_position, axis=1)
        is_uncomfortable = distances < self.config.uncomfortable_distance
        costs = np.zeros(other_positions.shape[0])
        costs[is_uncomfortable] = self.config.robot_speed * self.config.dt
        total_cost = np.sum(costs)

        return -total_cost
    
    def _sfm_force_score(self):
        num_humans = self.positions.shape[0] - 1
        if num_humans > 0:
            human_positions = self.positions[1:, :]
            _, robot_force_generated = self._social_forces(human_positions)

            approx_max_force_per_human = 30.0
            normalizing_factor = (approx_max_force_per_human * num_humans * self.config.mcts_config.max_depth) + 1e-6

            return -robot_force_generated / normalizing_factor
        else:
            return 0.0
    
    def _goal_distance(self) -> np.ndarray:
        distances = np.linalg.norm(self.agent_goal_positions - self.positions, axis=1)

        # small nudge to encourage getting to the final goal earlier
        if distances[0] < 0.25:
            distances[0] = -(self.config.mcts_config.max_depth - self.depth) * self.config.dt * self.config.robot_speed

        distances = -distances

        ## scale by starting distances so that staying at the start is worth -1.0 and
        ## reaching the destination is worth 0.0
        #safe_starting = np.where(self.config.starting_distances > 0, self.config.starting_distances, 1.0)
        #scores = np.where(self.config.starting_distances > 0, -distances / safe_starting, 0.0)

        return distances
    
    def _accumulate_value(self, value_accumulator) -> np.ndarray:
        value_accumulator = value_accumulator.copy()
        #value_accumulator[0] += 0.6 * self._sfm_force_score()
        #value_accumulator[0] += self._uncomfortable_distance()
        value_accumulator[0] += 1.5 * self._uncomfortable_distance_meter_score()
        if self.is_terminal():
            value_accumulator += 1.0 * self._goal_distance()
        
        invalid_state_mask, _ = self._get_invalid_state()
        value_accumulator[invalid_state_mask] = -self.config.starting_distances[0]

        return value_accumulator

    def terminal_values(self) -> ValueMap:
        return self._accumulated_value.tolist()

    # All good here
    def _calculate_human_velocities(self, positions, velocities):
        human_preferred_speed = 1.35
        dt = self.config.dt

        human_positions = positions[1:, :]
        human_velocities = velocities[1:, :].copy()
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
            human_velocities[too_fast] *= (max_speed / speed[too_fast])[:, None]

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
    rng = state.config.mcts_config.rng
    while not state.is_terminal():
        action_definitions = [
            state.sample_action(actor_idx, rng)
            for actor_idx
            in range(state.config.mcts_config.num_actors)
        ]
        state = state.apply_actions(action_definitions)
    return state.terminal_values()