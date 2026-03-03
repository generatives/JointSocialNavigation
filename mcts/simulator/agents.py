from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np

from mcts.decoupled_mcts import MCTS, MCTSConfig
from simulator.mcts_game_state import MCTSGameState, MCTSGameStateConfig, navigation_rollout

from .constants import WALL
from .map import ScenarioMap
from .pathfinding import a_star
from .physics import collides_with_walls


@dataclass
class Robot:
    position: np.ndarray
    theta: float
    radius: float = 0.35
    max_speed: float = 2.1
    max_omega: float = 2.6
    command_v: float = 0.0
    command_w: float = 0.0
    goal_idx: int = 0
    path: list[tuple[int, int]] = field(default_factory=list)
    path_ptr: int = 0

    def forward(self) -> np.ndarray:
        return np.array([math.cos(self.theta), math.sin(self.theta)], dtype=np.float32)

    def step(self, dt: float, world: ScenarioMap) -> "RobotStepMetrics":
        v = float(np.clip(self.command_v, -self.max_speed, self.max_speed))
        w = float(np.clip(self.command_w, -self.max_omega, self.max_omega))
        self.theta = (self.theta + w * dt + math.pi) % (2.0 * math.pi) - math.pi

        delta = self.forward() * v * dt
        new_pos = self.position + delta
        resolved, collided_with_wall = self._resolve_world_collision(new_pos, world)
        distance_travelled = float(np.linalg.norm(resolved - self.position))
        self.position[:] = resolved
        return RobotStepMetrics(
            distance_travelled=distance_travelled,
            collided_with_wall=collided_with_wall,
        )

    def _resolve_world_collision(self, proposal: np.ndarray, world: ScenarioMap) -> tuple[np.ndarray, bool]:
        out = self.position.copy()
        collided_with_wall = False
        for axis in (0, 1):
            test = out.copy()
            test[axis] = proposal[axis]
            if not collides_with_walls(test, self.radius, world):
                out[axis] = test[axis]
            else:
                collided_with_wall = True
        return out, collided_with_wall


@dataclass(slots=True)
class RobotStepMetrics:
    distance_travelled: float
    collided_with_wall: bool


@dataclass(slots=True)
class CrowdStepMetrics:
    robot_human_collisions: int
    robot_social_force_generated: float


class RobotAI:
    def __init__(self, scenario: ScenarioMap, replan_period: float = 0.75):
        self.scenario = scenario
        self.replan_period = replan_period
        self.replan_timer = 0.0
        self.manual_goal: tuple[int, int] | None = None

    def set_manual_goal(self, cell: tuple[int, int]) -> None:
        self.manual_goal = cell
        self.replan_timer = 0.0

    def clear_manual_goal(self) -> None:
        self.manual_goal = None
        self.replan_timer = 0.0

    def update(self, robot: Robot, dt: float) -> None:
        self.replan_timer -= dt
        if self.manual_goal is None:
            robot.command_v = 0.0
            robot.command_w = 0.0
            robot.path = []
            robot.path_ptr = 0
            return

        goal = self.manual_goal
        robot_cell = self.scenario.world_to_cell(robot.position)
        free_robot_cell = self.scenario.nearest_free(robot_cell)
        if free_robot_cell is None:
            robot.command_v = 0.0
            robot.command_w = 0.0
            return

        if np.linalg.norm(robot.position - self.scenario.cell_to_world(goal)) < 0.6:
            robot.command_v = 0.0
            robot.command_w = 0.0
            return

        if self.replan_timer <= 0.0 or not robot.path:
            self.replan_timer = self.replan_period
            new_path = a_star(self.scenario.grid, free_robot_cell, goal)
            if new_path:
                robot.path = new_path
                robot.path_ptr = 1 if len(new_path) > 1 else 0

        if not robot.path or robot.path_ptr >= len(robot.path):
            robot.command_v = 0.0
            robot.command_w = 0.0
            return

        target_world = self.scenario.cell_to_world(robot.path[robot.path_ptr])
        to_target = target_world - robot.position
        if np.linalg.norm(to_target) < 0.35 and robot.path_ptr < len(robot.path) - 1:
            robot.path_ptr += 1
            target_world = self.scenario.cell_to_world(robot.path[robot.path_ptr])
            to_target = target_world - robot.position

        desired_heading = math.atan2(float(to_target[1]), float(to_target[0]))
        heading_error = (desired_heading - robot.theta + math.pi) % (2.0 * math.pi) - math.pi
        distance = np.linalg.norm(to_target)

        robot.command_w = float(np.clip(2.3 * heading_error, -robot.max_omega, robot.max_omega))
        speed_scale = max(0.0, 1.0 - abs(heading_error) / math.pi)
        robot.command_v = float(np.clip(1.8 * distance * speed_scale, 0.0, robot.max_speed))

class MCTSRobotAI:
    def __init__(self, scenario: ScenarioMap, crowd: Crowd, replan_period: float = 0.50):
        self.scenario = scenario
        self.crowd = crowd
        self.replan_period = replan_period
        self.replan_timer = 0.0
        self.manual_goal: tuple[int, int] | None = None
        self.intermediate_goal: np.ndarray | None = None

    def set_manual_goal(self, cell: tuple[int, int]) -> None:
        self.manual_goal = cell
        self.replan_timer = 0.0
        self.intermediate_goal = None

    def clear_manual_goal(self) -> None:
        self.manual_goal = None
        self.replan_timer = 0.0
        self.intermediate_goal = None

    def _plan_intermediate_goal(self, robot: Robot) -> np.ndarray:
        num_humans = 5
        num_actions = 3
        tree_depth = 6
        dt = 0.5
        human_speed = 1.7

        if self.manual_goal is None:
            return robot.position.copy()

        active_idxs = np.flatnonzero(self.crowd.active)
        if active_idxs.size == 0:
            return self.scenario.cell_to_world(self.manual_goal)

        distances = np.linalg.norm(self.crowd.positions[active_idxs] - robot.position, axis=1)
        sorted_active = active_idxs[np.argsort(distances)]
        closest_humans = sorted_active[:num_humans]

        human_positions = self.crowd.positions[closest_humans]
        human_velocities = self.crowd.velocities[closest_humans]
        human_orientations = np.arctan2(human_velocities[:, 1], human_velocities[:, 0])
        human_goals = human_positions + human_velocities * (human_speed * dt * tree_depth)

        positions = np.vstack((robot.position[None, :], human_positions))
        orientations = np.concatenate((np.array([robot.theta], dtype=np.float32), human_orientations))
        robot_goal = self.scenario.cell_to_world(self.manual_goal)[None, :]
        goal_positions = np.vstack((robot_goal, human_goals))

        num_agents = positions.shape[0]
        mcts_config = MCTSConfig(num_actors=num_agents, num_actions=num_actions, max_depth=tree_depth)
        state_config = MCTSGameStateConfig(
            mcts_config=mcts_config,
            movement_distance=human_speed * dt,
            angle=np.pi / 4.0,
            uncomfortable_distance=1.5,
            map=self.scenario,
        )
        mcts = MCTS(mcts_config, navigation_rollout, rng=random.Random(random.randint(0, 2**31 - 1)))

        root_state = MCTSGameState(
            positions=positions,
            orientations=orientations,
            agent_goal_positions=goal_positions,
            value_accumulator=None,
            config=state_config,
            depth=0
        )

        _, child_state, _ = mcts.search(root_state, num_simulations=100)
        return child_state.positions[0].copy()

    def update(self, robot: Robot, dt: float) -> None:
        self.replan_timer -= dt
        if self.manual_goal is None:
            robot.command_v = 0.0
            robot.command_w = 0.0
            robot.path = []
            robot.path_ptr = 0
            return

        goal = self.manual_goal
        robot_cell = self.scenario.world_to_cell(robot.position)
        free_robot_cell = self.scenario.nearest_free(robot_cell)
        if free_robot_cell is None:
            robot.command_v = 0.0
            robot.command_w = 0.0
            return

        if np.linalg.norm(robot.position - self.scenario.cell_to_world(goal)) < 0.6:
            robot.command_v = 0.0
            robot.command_w = 0.0
            return
        
        if self.replan_timer <= 0.0 or self.intermediate_goal is None or np.linalg.norm(robot.position - self.intermediate_goal) < 0.35:
            self.replan_timer = self.replan_period
            self.intermediate_goal = self._plan_intermediate_goal(robot)

        target_disp = self.intermediate_goal - robot.position

        desired_heading = math.atan2(float(target_disp[1]), float(target_disp[0]))
        heading_error = (desired_heading - robot.theta + math.pi) % (2.0 * math.pi) - math.pi
        distance = np.linalg.norm(target_disp)

        robot.command_w = float(np.clip(2.3 * heading_error, -robot.max_omega, robot.max_omega))
        speed_scale = max(0.0, 1.0 - abs(heading_error) / math.pi)
        robot.command_v = float(np.clip(1.8 * distance * speed_scale, 0.0, robot.max_speed))


class Crowd:
    def __init__(self, max_humans: int, scenario: ScenarioMap) -> None:
        self.max_humans = max_humans
        self.scenario = scenario
        self.active = np.zeros(max_humans, dtype=bool)
        self.positions = np.zeros((max_humans, 2), dtype=np.float32)
        self.velocities = np.zeros((max_humans, 2), dtype=np.float32)
        self.goals = np.zeros((max_humans, 2), dtype=np.float32)
        self.radius = np.full(max_humans, 0.28, dtype=np.float32)
        self.pref_speed = np.random.uniform(1.0, 1.7, size=max_humans).astype(np.float32)
        self.paths: list[list[tuple[int, int]]] = [[] for _ in range(max_humans)]
        self.path_ptr = np.zeros(max_humans, dtype=np.int32)
        self.replan_timer = np.random.uniform(0.2, 1.1, size=max_humans).astype(np.float32)
        self.spawn_rate_per_sec = 0.5
        self.spawn_accumulator = 0.0

    def spawn(self) -> bool:
        idxs = np.flatnonzero(~self.active)
        if idxs.size == 0:
            return False
        idx = int(idxs[0])
        start_cell = random.choice(self.scenario.human_starts)
        goal_cell = random.choice(self.scenario.human_ends)
        path = a_star(self.scenario.grid, start_cell, goal_cell)
        if not path:
            return False

        p = self.scenario.cell_to_world(start_cell)
        self.active[idx] = True
        self.positions[idx] = p
        self.velocities[idx] = 0.0
        self.goals[idx] = self.scenario.cell_to_world(goal_cell)
        self.paths[idx] = path
        self.path_ptr[idx] = 1 if len(path) > 1 else 0
        self.replan_timer[idx] = random.uniform(0.4, 1.2)
        return True

    def despawn(self, idx: int) -> None:
        self.active[idx] = False
        self.velocities[idx] = 0.0
        self.paths[idx] = []
        self.path_ptr[idx] = 0
        self.replan_timer[idx] = random.uniform(0.4, 1.2)

    def update(self, dt: float, robot: Robot) -> CrowdStepMetrics:
        robot_social_force_generated = 0.0
        robot_human_collisions = 0
        self.spawn_accumulator += dt * self.spawn_rate_per_sec
        while self.spawn_accumulator >= 1.0:
            self.spawn()
            self.spawn_accumulator -= 1.0

        active_idxs = np.flatnonzero(self.active)
        if active_idxs.size == 0:
            return CrowdStepMetrics(
                robot_human_collisions=robot_human_collisions,
                robot_social_force_generated=robot_social_force_generated,
            )

        self.replan_timer[active_idxs] -= dt
        for i in active_idxs:
            self._replan_if_needed(int(i))

        for i in active_idxs:
            self._update_path_target(int(i))

        desired = np.zeros((active_idxs.size, 2), dtype=np.float32)
        for j, i in enumerate(active_idxs):
            waypoint = self._current_waypoint(int(i))
            to_target = waypoint - self.positions[i]
            dist = np.linalg.norm(to_target)
            if dist > 1e-6:
                desired[j] = (to_target / dist) * self.pref_speed[i]

        relaxation_time = 0.45
        accel = (desired - self.velocities[active_idxs]) / relaxation_time
        social_forces, force_generated = self._social_forces(active_idxs, robot)
        robot_social_force_generated += force_generated
        accel += social_forces
        self.velocities[active_idxs] += accel * dt

        speed = np.linalg.norm(self.velocities[active_idxs], axis=1)
        max_speed = self.pref_speed[active_idxs] * 1.7
        too_fast = speed > max_speed
        if np.any(too_fast):
            self.velocities[active_idxs[too_fast]] *= (max_speed[too_fast] / speed[too_fast])[:, None]

        proposed = self.positions[active_idxs] + self.velocities[active_idxs] * dt
        for idx_local, i in enumerate(active_idxs):
            self.positions[i] = self._resolve_world_collision(int(i), proposed[idx_local])

        self._resolve_human_collisions(active_idxs)
        robot_human_collisions += self._resolve_robot_collisions(active_idxs, robot)

        for i in active_idxs:
            if np.linalg.norm(self.positions[i] - self.goals[i]) < 0.6:
                self.despawn(int(i))
        return CrowdStepMetrics(
            robot_human_collisions=robot_human_collisions,
            robot_social_force_generated=robot_social_force_generated,
        )

    def _current_waypoint(self, idx: int) -> np.ndarray:
        path = self.paths[idx]
        if not path:
            return self.goals[idx]
        ptr = int(np.clip(self.path_ptr[idx], 0, len(path) - 1))
        return self.scenario.cell_to_world(path[ptr])

    def _update_path_target(self, idx: int) -> None:
        path = self.paths[idx]
        if not path:
            return
        ptr = int(self.path_ptr[idx])
        if ptr >= len(path):
            return
        waypoint = self.scenario.cell_to_world(path[ptr])
        if np.linalg.norm(self.positions[idx] - waypoint) < 0.35 and ptr < len(path) - 1:
            self.path_ptr[idx] += 1

    def _replan_if_needed(self, idx: int) -> None:
        if self.replan_timer[idx] > 0.0:
            return
        self.replan_timer[idx] = random.uniform(0.8, 1.5)
        start = self.scenario.nearest_free(self.scenario.world_to_cell(self.positions[idx]))
        goal = self.scenario.nearest_free(self.scenario.world_to_cell(self.goals[idx]))
        if start is None or goal is None:
            return
        path = a_star(self.scenario.grid, start, goal)
        if path:
            self.paths[idx] = path
            self.path_ptr[idx] = 1 if len(path) > 1 else 0

    def _social_forces(self, active_idxs: np.ndarray, robot: Robot) -> tuple[np.ndarray, float]:
        n = active_idxs.size
        forces = np.zeros((n, 2), dtype=np.float32)
        robot_social_force_generated = 0.0

        a_h = 6.0
        b_h = 0.7
        a_obs = 3.2
        b_obs = 0.7

        pos = self.positions[active_idxs]
        rad = self.radius[active_idxs]

        for i in range(n):
            for j in range(i + 1, n):
                diff = pos[i] - pos[j]
                dist = np.linalg.norm(diff)
                if dist < 1e-4:
                    continue
                direction = diff / dist
                penetration = rad[i] + rad[j] - dist
                mag = a_h * math.exp((rad[i] + rad[j] - dist) / b_h)
                if penetration > 0.0:
                    mag += penetration * 25.0
                force = direction * mag
                forces[i] += force
                forces[j] -= force

        robot_pos = robot.position
        for i in range(n):
            diff = pos[i] - robot_pos
            dist = np.linalg.norm(diff)
            if dist < 1e-4:
                continue
            direction = diff / dist
            combined = rad[i] + robot.radius
            penetration = combined - dist
            mag = 10.0 * math.exp((combined - dist) / 0.6)
            if penetration > 0.0:
                mag += penetration * 30.0
            force = direction * mag
            forces[i] += force
            robot_social_force_generated += float(np.linalg.norm(force))

        for i in range(n):
            cell = self.scenario.world_to_cell(pos[i])
            for oy in range(-2, 3):
                for ox in range(-2, 3):
                    cx, cy = cell[0] + ox, cell[1] + oy
                    if cx < 0 or cx >= self.scenario.width or cy < 0 or cy >= self.scenario.height:
                        continue
                    if self.scenario.grid[cy, cx] != WALL:
                        continue
                    obstacle_pos = np.array([float(cx) + 0.5, float(cy) + 0.5], dtype=np.float32)
                    diff = pos[i] - obstacle_pos
                    dist = np.linalg.norm(diff)
                    if dist < 1e-4:
                        continue
                    direction = diff / dist
                    mag = a_obs * math.exp((rad[i] + 0.5 - dist) / b_obs)
                    forces[i] += direction * mag

        return forces, robot_social_force_generated

    def _resolve_world_collision(self, idx: int, proposal: np.ndarray) -> np.ndarray:
        out = self.positions[idx].copy()
        for axis in (0, 1):
            test = out.copy()
            test[axis] = proposal[axis]
            if not collides_with_walls(test, float(self.radius[idx]), self.scenario):
                out[axis] = test[axis]
            else:
                self.velocities[idx, axis] = 0.0
        return out

    def _resolve_human_collisions(self, active_idxs: np.ndarray) -> None:
        for i_pos in range(active_idxs.size):
            i = int(active_idxs[i_pos])
            for j_pos in range(i_pos + 1, active_idxs.size):
                j = int(active_idxs[j_pos])
                delta = self.positions[i] - self.positions[j]
                dist = np.linalg.norm(delta)
                target = float(self.radius[i] + self.radius[j])
                if dist < 1e-6 or dist >= target:
                    continue
                n = delta / dist
                overlap = target - dist
                self.positions[i] += n * (overlap * 0.5)
                self.positions[j] -= n * (overlap * 0.5)
                self.velocities[i] *= 0.5
                self.velocities[j] *= 0.5

    def _resolve_robot_collisions(self, active_idxs: np.ndarray, robot: Robot) -> int:
        collision_count = 0
        for i in active_idxs:
            delta = self.positions[i] - robot.position
            dist = np.linalg.norm(delta)
            target = float(self.radius[i] + robot.radius)
            if dist < 1e-6 or dist >= target:
                continue
            n = delta / dist
            overlap = target - dist
            self.positions[i] += n * overlap
            self.velocities[i] *= 0.3
            collision_count += 1
        return collision_count
