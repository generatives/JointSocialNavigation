from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from social_navigation.mcts.decoupled_mcts import MCTS, MCTSConfig
from social_navigation.simulator.mcts_game_state import MCTSGameState, MCTSGameStateConfig, navigation_rollout

from .constants import WALL
from .scenario_map import ScenarioMap
from .pathfinding import a_star
from .physics import collides_with_walls

ROBOT_RADIUS = 0.35
MAX_ROBOT_SPEED = 2.1
MAX_ROBOT_OMEGA = 2.6

@dataclass
class Robot:
    position: np.ndarray
    theta: float
    radius: float = ROBOT_RADIUS
    max_speed: float = MAX_ROBOT_SPEED
    max_omega: float = MAX_ROBOT_OMEGA
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
    min_human_robot_distance: float


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

        final_goal_world = self.scenario.cell_to_world(goal)
        final_goal_dist = np.linalg.norm(robot.position - final_goal_world)
        if final_goal_dist < 0.6:
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

        robot.command_w = float(np.clip(2.3 * heading_error, -robot.max_omega, robot.max_omega))
        speed_scale = max(0.0, 1.0 - abs(heading_error) / math.pi)

        # Keep speed high through intermediate waypoints.
        cruise_speed = robot.max_speed * speed_scale

        # Only slow down near the final manual goal.
        slow_radius = 1.5
        if final_goal_dist < slow_radius:
            cruise_speed *= final_goal_dist / slow_radius

        robot.command_v = float(np.clip(cruise_speed, 0.0, robot.max_speed))

class MCTSRobotAI:
    def __init__(self, scenario: ScenarioMap, crowd: Crowd):
        self.scenario = scenario
        self.crowd = crowd
        self.replan_timer = 0.0
        self.manual_goal: tuple[int, int] | None = None
        self.command_v: float | None = None
        self.command_w: float | None = None
        self.planned_robot_trajectory: List[float, float] | None = None
        self.planned_human_trajectories: Dict[int, List[float, float]] | None = None
        self.planned_human_goal_estimates: Dict[int, np.ndarray] | None = None

    def set_manual_goal(self, cell: tuple[int, int]) -> None:
        self.manual_goal = cell
        self.replan_timer = 0.0
        self.command_v = None
        self.command_w = None

    def clear_manual_goal(self) -> None:
        self.manual_goal = None
        self.replan_timer = 0.0
        self.command_v = None
        self.command_w = None

    def _plan_intermediate_goal(self, robot: Robot) -> np.ndarray:
        num_humans = 5
        tree_depth = 6
        assumed_human_speed = 1.7
        robot_speed = MAX_ROBOT_SPEED * 0.9
        robot_omega = MAX_ROBOT_OMEGA * 0.9

        if self.manual_goal is None:
            return robot.position.copy()
        
        robot_goal = self.scenario.cell_to_world(self.manual_goal)
        final_goal_dist = np.linalg.norm(robot.position - robot_goal)
        time_to_goal = final_goal_dist / robot_speed
        dt = time_to_goal / tree_depth
        dt = min(0.5, max(0.2, dt))

        active_idxs = np.flatnonzero(self.crowd.active)
        if active_idxs.size == 0:
            return self.scenario.cell_to_world(self.manual_goal)

        distances = np.linalg.norm(self.crowd.positions[active_idxs] - robot.position, axis=1)
        sorted_active = active_idxs[np.argsort(distances)]
        closest_humans = sorted_active[:num_humans]

        human_positions = self.crowd.positions[closest_humans]
        human_velocities = self.crowd.velocities[closest_humans]
        horizon = assumed_human_speed * dt * tree_depth
        ema_vels = self.crowd.velocity_ema[closest_humans]
        ema_speeds = np.linalg.norm(ema_vels, axis=1, keepdims=True)
        moving = ema_speeds > 0.1
        ema_dirs = np.where(moving, ema_vels / np.where(moving, ema_speeds, 1.0), 0.0)
        human_goals = human_positions + ema_dirs * horizon

        robot_velocity = np.array([
            [np.cos(robot.theta), np.sin(robot.theta)]
        ])

        positions = np.vstack((robot.position[None, :], human_positions))
        velocities = np.concatenate((robot_velocity, human_velocities))
        goal_positions = np.vstack((robot_goal[None, :], human_goals))
        starting_distances = np.linalg.norm(goal_positions - positions, axis=1)

        num_agents = positions.shape[0]
        num_actions = [6] + [1] * (num_agents - 1)
        mcts_config = MCTSConfig(
            num_actors=num_agents, 
            max_actions=num_actions, 
            rng=random.Random(random.randint(0, 2**31 - 1)),
            max_depth=tree_depth)
        state_config = MCTSGameStateConfig(
            mcts_config=mcts_config,
            robot_speed=robot_speed,
            dt=dt,
            robot_radius=robot.radius,
            human_radius=np.mean(self.crowd.radius),
            robot_angular_velocity=robot_omega,
            uncomfortable_distance=1.5,
            starting_distances=starting_distances,
            map=self.scenario,
        )
        mcts = MCTS(mcts_config, navigation_rollout)

        root_state = MCTSGameState(
            positions=positions,
            velocities=velocities,
            agent_goal_positions=goal_positions,
            accumulated_value=None,
            config=state_config,
            depth=0
        )

        actions, child_state, state_trajectory, _ = mcts.search(root_state, num_simulations=5000)
        self.planned_robot_trajectory = [state.positions[0].copy() for state in state_trajectory]
        self.planned_human_trajectories = {
            human_index: [state.positions[actor_index+1].copy() for state in state_trajectory]
            for actor_index, human_index in enumerate(closest_humans)
        }
        self.planned_human_goal_estimates = {
            human_index: human_goals[actor_index].copy()
            for actor_index, human_index in enumerate(closest_humans)
        }

        linear_velocity, angular_velocity = root_state.get_command_velocities(actions[0])

        return linear_velocity, angular_velocity, dt

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

        if goal == robot_cell:
            self.clear_manual_goal()
            return

        free_robot_cell = self.scenario.nearest_free(robot_cell)
        if free_robot_cell is None:
            robot.command_v = 0.0
            robot.command_w = 0.0
            return
        
        if self.replan_timer <= 0.0:
            linear_velocity, angular_velocity, plan_length = self._plan_intermediate_goal(robot)
            self.command_v = linear_velocity
            self.command_w = angular_velocity
            self.replan_timer = plan_length

        robot.command_v = self.command_v
        robot.command_w = self.command_w


class JointAStarRobotAI:
    """
    Robot AI that plans via Joint State Space A*.

    At each replan tick it:
      1. Observes the robot's current grid cell and the closest active human's
         current cell (from their continuous SFM position).
      2. Calls joint_a_star, which assumes the human moves one cell per step
         toward its goal — a deliberate simplification.  The actual human is
         driven by SFM (social forces + wall avoidance), so their trajectory
         will deviate.  Replanning every ``replan_period`` seconds corrects
         this: the plan is always anchored to the human's real position.
      3. Hands the planned robot path to the same proportional path-following
         controller used by RobotAI.

    Falls back to plain A* when no humans are active.
    """

    def __init__(
        self,
        scenario: ScenarioMap,
        crowd: "Crowd",
        replan_period: float = 0.25,
        robot_move_penalty: float = 0.05,
        human_detour_penalty: float = 0.5,
        blocking_penalty: float = 2.0,
        proximity_penalty: float = 3.5,
        proximity_threshold: int = 4,
        belief_discount_blocking: float = 0.5,
    ) -> None:
        self.scenario = scenario
        self.crowd = crowd
        self.replan_period = replan_period
        self.robot_move_penalty = robot_move_penalty
        self.human_detour_penalty = human_detour_penalty
        self.blocking_penalty = blocking_penalty
        self.proximity_penalty = proximity_penalty
        self.proximity_threshold = proximity_threshold
        self.belief_discount_blocking = belief_discount_blocking
        self.replan_timer = 0.0
        self.manual_goal: tuple[int, int] | None = None
        self.planned_robot_path: list[tuple[int, int]] = []
        self.planned_human_path: list[tuple[int, int]] = []

    def set_manual_goal(self, cell: tuple[int, int]) -> None:
        self.manual_goal = cell
        self.replan_timer = 0.0

    def clear_manual_goal(self) -> None:
        self.manual_goal = None
        self.replan_timer = 0.0
        self.planned_robot_path = []
        self.planned_human_path = []

    def update(self, robot: Robot, dt: float) -> None:
        self.replan_timer -= dt
        if self.manual_goal is None:
            robot.command_v = 0.0
            robot.command_w = 0.0
            robot.path = []
            robot.path_ptr = 0
            return

        goal = self.manual_goal
        robot_cell = self.scenario.nearest_free(self.scenario.world_to_cell(robot.position))
        if robot_cell is None:
            robot.command_v = 0.0
            robot.command_w = 0.0
            return

        final_goal_world = self.scenario.cell_to_world(goal)
        final_goal_dist = np.linalg.norm(robot.position - final_goal_world)
        if final_goal_dist < 0.6:
            robot.command_v = 0.0
            robot.command_w = 0.0
            return

        path_exhausted = not robot.path or robot.path_ptr >= len(robot.path)
        if self.replan_timer <= 0.0 or path_exhausted:
            self.replan_timer = self.replan_period
            self._replan(robot, robot_cell, goal)

        if not robot.path or robot.path_ptr >= len(robot.path):
            robot.command_v = 0.0
            robot.command_w = 0.0
            return

        # Single-cell path means "hold position" — navigate to the cell
        # center so the robot is fully inside, then stop.
        holding = len(robot.path) == 1

        target_world = self.scenario.cell_to_world(robot.path[robot.path_ptr])
        to_target = target_world - robot.position
        dist_to_target = float(np.linalg.norm(to_target))

        if not holding and dist_to_target < 0.35 and robot.path_ptr < len(robot.path) - 1:
            robot.path_ptr += 1
            target_world = self.scenario.cell_to_world(robot.path[robot.path_ptr])
            to_target = target_world - robot.position
            dist_to_target = float(np.linalg.norm(to_target))

        # Close enough to hold cell center — fully stop.
        if holding and dist_to_target < 0.15:
            robot.command_v = 0.0
            robot.command_w = 0.0
            return

        desired_heading = math.atan2(float(to_target[1]), float(to_target[0]))
        heading_error = (desired_heading - robot.theta + math.pi) % (2.0 * math.pi) - math.pi

        robot.command_w = float(np.clip(2.3 * heading_error, -robot.max_omega, robot.max_omega))
        speed_scale = max(0.0, 1.0 - abs(heading_error) / math.pi)

        if holding:
            # Slow approach to cell center — proportional to distance so the
            # robot settles smoothly instead of circling at cruise speed.
            cruise_speed = float(np.clip(1.8 * dist_to_target * speed_scale, 0.0, robot.max_speed))
        else:
            # Keep speed high through intermediate waypoints.
            cruise_speed = robot.max_speed * speed_scale
            # Only slow down near the final manual goal.
            slow_radius = 1.5
            if final_goal_dist < slow_radius:
                cruise_speed *= final_goal_dist / slow_radius

            # "I see you" signal: when a human is on the corridor ahead,
            # gently reduce speed proportional to proximity.  This gives
            # the human observable evidence that the robot is aware of them,
            # allowing their awareness belief to rise before yielding.
            active = np.flatnonzero(self.crowd.active)
            if active.size > 0:
                dists = np.linalg.norm(
                    self.crowd.positions[active] - robot.position, axis=1
                )
                min_dist = float(dists.min())
                awareness_range = 15.0
                if min_dist < awareness_range:
                    # Scale from 1.0 (at range) to 0.6 (very close).
                    caution = 0.6 + 0.4 * (min_dist / awareness_range)
                    cruise_speed *= caution

        robot.command_v = float(np.clip(cruise_speed, 0.0, robot.max_speed))


    def _replan(self, robot: Robot, robot_cell: tuple[int, int], robot_goal: tuple[int, int]) -> None:
        from social_navigation.joint_astar.joint_astar import joint_a_star

        active = np.flatnonzero(self.crowd.active)
        if active.size == 0:
            path = a_star(self.scenario.grid, robot_cell, robot_goal)
            if path:
                robot.path = path
                robot.path_ptr = 1 if len(path) > 1 else 0
            self.planned_robot_path = list(robot.path)
            self.planned_human_path = []
            return

        # Use the closest active human.
        distances = np.linalg.norm(self.crowd.positions[active] - robot.position, axis=1)
        closest = int(active[int(np.argmin(distances))])

        human_cell = self.scenario.nearest_free(
            self.scenario.world_to_cell(self.crowd.positions[closest])
        )
        human_goal_cell = self.scenario.nearest_free(
            self.scenario.world_to_cell(self.crowd.goals[closest])
        )
        if human_cell is None or human_goal_cell is None or human_cell == robot_cell:
            return

        # If the human is actively yielding (in the alcove) or already
        # behind the robot (they've passed each other), the corridor is
        # clear — skip joint planning and use plain A*.
        human_to_goal = (
            abs(human_cell[0] - robot_goal[0]) + abs(human_cell[1] - robot_goal[1])
        )
        robot_to_goal = (
            abs(robot_cell[0] - robot_goal[0]) + abs(robot_cell[1] - robot_goal[1])
        )
        human_behind = human_to_goal > robot_to_goal
        if self.crowd.yielding[closest] or human_behind:
            path = a_star(self.scenario.grid, robot_cell, robot_goal)
            if path:
                robot.path = path
                robot.path_ptr = 1 if len(path) > 1 else 0
            self.planned_robot_path = list(robot.path)
            self.planned_human_path = []
            return

        # Adjust planning conservatism based on the human's two beliefs:
        #
        # 1. Goal belief (does the human know where I'm going?)
        #    High + correct → discount blocking penalty (human will yield).
        #
        # 2. Awareness belief (does the human think I see them?)
        #    Low → increase proximity penalty (human won't yield, might
        #    freeze or push back harder — robot should keep more distance).
        effective_blocking = self.blocking_penalty
        effective_proximity = self.proximity_penalty
        best_belief = float(self.crowd.robot_goal_beliefs[closest].max())
        awareness = float(self.crowd.robot_awareness_belief[closest])

        if best_belief >= self.crowd.belief_yield_threshold:
            best_idx = int(np.argmax(self.crowd.robot_goal_beliefs[closest]))
            inferred_goal = self.crowd._robot_goal_candidates[best_idx]
            actual_goal_world = self.scenario.cell_to_world(robot_goal)
            if np.linalg.norm(inferred_goal - actual_goal_world) < 2.0:
                # Only discount if human also believes robot is aware.
                # Both beliefs must be strong for cooperative yielding.
                if awareness >= self.crowd.belief_yield_threshold:
                    effective_blocking *= self.belief_discount_blocking
                else:
                    # Human knows goal but thinks robot is unaware — human
                    # will be defensive, not cooperative.  Plan conservatively.
                    effective_proximity *= 1.5

        result = joint_a_star(
            self.scenario, robot_cell, robot_goal, human_cell, human_goal_cell,
            robot_move_penalty=self.robot_move_penalty,
            human_detour_penalty=self.human_detour_penalty,
            blocking_penalty=effective_blocking,
            proximity_penalty=effective_proximity,
            proximity_threshold=self.proximity_threshold,
        )
        if result and len(result.robot_path) > 1:
            self.planned_robot_path = result.robot_path
            self.planned_human_path = result.human_path

            # result.robot_path[0] is the current cell; result.robot_path[1] is
            # the planner's intended next cell for this tick.  If they are the
            # same, the plan says "wait" — hold position.  If they differ, give
            # the robot exactly that one cell as its next waypoint and let the
            # next replan issue the following step.  This prevents the
            # path-following controller from racing through all the repeated
            # "wait" entries and driving the robot out of the alcove early.
            next_cell = result.robot_path[1]
            if next_cell == robot_cell:
                # Hold position: a single-entry path terminates immediately and
                # leaves the robot stationary until the replan timer fires.
                robot.path = [robot_cell]
                robot.path_ptr = 0
            else:
                robot.path = [robot_cell, next_cell]
                robot.path_ptr = 1
        else:
            path = a_star(self.scenario.grid, robot_cell, robot_goal)
            if path:
                robot.path = path
                robot.path_ptr = 1 if len(path) > 1 else 0
            self.planned_robot_path = list(robot.path)
            self.planned_human_path = []


class Crowd:
    def __init__(
        self,
        max_humans: int,
        scenario: ScenarioMap,
        max_spawns: int | None = None,
        spawn_rate_per_sec: float = 0.2,
        pref_speed_min: float = 1.0,
        pref_speed_max: float = 1.7,
        human_human_amplitude: float = 6.0,
        human_human_decay: float = 0.7,
        robot_human_amplitude: float = 10.0,
        robot_human_decay: float = 0.6,
        obstacle_amplitude: float = 1.0,
        obstacle_decay: float = 0.7,
        belief_sigma: float = 0.5,
        belief_yield_threshold: float = 0.5,
        belief_lookahead: float = 1.5,
        awareness_sigma: float = 0.6,
    ) -> None:
        self.max_humans = max_humans
        self.scenario = scenario
        self.max_spawns = max_spawns  # None = unlimited
        self.total_spawned = 0
        self.active = np.zeros(max_humans, dtype=bool)
        self.positions = np.zeros((max_humans, 2), dtype=np.float32)
        self.velocities = np.zeros((max_humans, 2), dtype=np.float32)
        self.goals = np.zeros((max_humans, 2), dtype=np.float32)
        self.radius = np.full(max_humans, 0.28, dtype=np.float32)
        self.pref_speed = np.random.uniform(pref_speed_min, pref_speed_max, size=max_humans).astype(np.float32)
        self.paths: list[list[tuple[int, int]]] = [[] for _ in range(max_humans)]
        self.path_ptr = np.zeros(max_humans, dtype=np.int32)
        self.replan_timer = np.random.uniform(0.2, 1.1, size=max_humans).astype(np.float32)
        self.human_human_amplitude = human_human_amplitude
        self.human_human_decay = human_human_decay
        self.robot_human_amplitude = robot_human_amplitude
        self.robot_human_decay = robot_human_decay
        self.obstacle_amplitude = obstacle_amplitude
        self.obstacle_decay = obstacle_decay
        self.spawn_rate_per_sec = spawn_rate_per_sec
        self.spawn_accumulator = 0.0
        self.velocity_ema = np.zeros((max_humans, 2), dtype=np.float32)
        self.ema_alpha = 0.02 # exponential moving average

        # --- Belief state -------------------------------------------------
        # Candidate destinations the robot might be heading to.
        # Sourced from scenario.robot_goal_candidates which prefers explicit
        # robot_goals when available, falling back to human_starts ∪ human_ends.
        _candidate_cells: list[tuple[int, int]] = scenario.robot_goal_candidates
        if _candidate_cells:
            self._robot_goal_candidates = np.array(
                [scenario.cell_to_world(c) for c in _candidate_cells],
                dtype=np.float32,
            )  # (num_candidates, 2)
        else:
            self._robot_goal_candidates = np.zeros((0, 2), dtype=np.float32)

        num_candidates = self._robot_goal_candidates.shape[0]
        # Uniform prior: each human believes any destination equally likely.
        self.robot_goal_beliefs = np.full(
            (max_humans, max(num_candidates, 1)),
            1.0 / max(num_candidates, 1),
            dtype=np.float32,
        )
        # Angular uncertainty of the Gaussian likelihood (radians).
        self.belief_sigma = belief_sigma
        # Minimum max-belief before anticipatory forces activate.
        self.belief_yield_threshold = belief_yield_threshold
        # Seconds ahead to project robot's position for anticipatory repulsion.
        self.belief_lookahead = belief_lookahead

        # --- Awareness belief state -----------------------------------------
        # Each human also estimates whether the robot is *aware* of their
        # presence.  The observation model watches the robot's lateral
        # deviation from a straight-line path to its inferred goal: deviation
        # toward the human → likely aware; straight-line heading → uncertain.
        # Scalar per human: P(robot aware of me), initialised to 0.5.
        self.robot_awareness_belief = np.full(max_humans, 0.5, dtype=np.float32)
        self.awareness_sigma = awareness_sigma
        # Store previous robot heading to compute curvature (heading rate).
        self._prev_robot_theta: float | None = None

        # --- Yield state (belief-conditioned replanning) ---
        # When a human's belief is strong enough, it reroutes through a nearby
        # alcove cell and waits there until the robot has passed.
        self.yield_target: list[tuple[int, int] | None] = [None] * max_humans
        self.yielding = np.zeros(max_humans, dtype=bool)

    def spawn(self) -> bool:
        if self.max_spawns is not None and self.total_spawned >= self.max_spawns:
            return False
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
        # Reset beliefs so new spawns don't inherit stale posteriors.
        num_candidates = self._robot_goal_candidates.shape[0]
        if num_candidates > 0:
            self.robot_goal_beliefs[idx] = 1.0 / num_candidates
        self.robot_awareness_belief[idx] = 0.5
        self.yield_target[idx] = None
        self.yielding[idx] = False
        self.total_spawned += 1
        return True

    def despawn(self, idx: int) -> None:
        self.active[idx] = False
        self.velocities[idx] = 0.0
        self.velocity_ema[idx] = 0.0
        self.paths[idx] = []
        self.path_ptr[idx] = 0
        self.replan_timer[idx] = random.uniform(0.4, 1.2)
        num_candidates = self._robot_goal_candidates.shape[0]
        if num_candidates > 0:
            self.robot_goal_beliefs[idx] = 1.0 / num_candidates
        self.robot_awareness_belief[idx] = 0.5
        self.yield_target[idx] = None
        self.yielding[idx] = False

    def update(self, dt: float, robot: Robot) -> CrowdStepMetrics:
        robot_social_force_generated = 0.0
        robot_human_collisions = 0
        min_human_robot_distance = float("inf")
        self.spawn_accumulator += dt * self.spawn_rate_per_sec
        while self.spawn_accumulator >= 1.0:
            self.spawn()
            self.spawn_accumulator -= 1.0

        active_idxs = np.flatnonzero(self.active)
        if active_idxs.size == 0:
            return CrowdStepMetrics(
                robot_human_collisions=robot_human_collisions,
                robot_social_force_generated=robot_social_force_generated,
                min_human_robot_distance=min_human_robot_distance,
            )

        self.replan_timer[active_idxs] -= dt
        # Update each human's belief about the robot's intended destination
        # before computing forces so anticipatory terms use fresh posteriors.
        self._update_robot_goal_beliefs(robot)
        # Update each human's belief about whether the robot is aware of them.
        self._update_robot_awareness_beliefs(active_idxs, robot, dt)
        # Check whether each human should start or stop yielding.
        self._update_yield_behavior(active_idxs, robot)
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
        accel += self._belief_conditioned_forces(active_idxs, robot)
        self.velocities[active_idxs] += accel * dt

        speed = np.linalg.norm(self.velocities[active_idxs], axis=1)
        max_speed = self.pref_speed[active_idxs] * 1.7
        too_fast = speed > max_speed
        if np.any(too_fast):
            self.velocities[active_idxs[too_fast]] *= (max_speed[too_fast] / speed[too_fast])[:, None]

        self.velocity_ema[active_idxs] = (
            self.ema_alpha * self.velocities[active_idxs]
            + (1 - self.ema_alpha) * self.velocity_ema[active_idxs]
        )

        proposed = self.positions[active_idxs] + self.velocities[active_idxs] * dt
        for idx_local, i in enumerate(active_idxs):
            self.positions[i] = self._resolve_world_collision(int(i), proposed[idx_local])

        self._resolve_human_collisions(active_idxs)
        robot_human_collisions += self._resolve_robot_collisions(active_idxs, robot)

        for i in active_idxs:
            d = float(np.linalg.norm(self.positions[i] - robot.position))
            if d < min_human_robot_distance:
                min_human_robot_distance = d
            if np.linalg.norm(self.positions[i] - self.goals[i]) < 0.6:
                self.despawn(int(i))
        return CrowdStepMetrics(
            robot_human_collisions=robot_human_collisions,
            robot_social_force_generated=robot_social_force_generated,
            min_human_robot_distance=min_human_robot_distance,
        )

    def _current_waypoint(self, idx: int) -> np.ndarray:
        # When yielding, always steer toward the yield cell (and stay there).
        if self.yielding[idx] and self.yield_target[idx] is not None:
            return self.scenario.cell_to_world(self.yield_target[idx])
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
        if self.yielding[idx]:
            return  # don't overwrite yield path
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

    # ------------------------------------------------------------------
    # Belief-conditioned yielding
    # ------------------------------------------------------------------

    def _update_yield_behavior(self, active_idxs: np.ndarray, robot: Robot) -> None:
        """Start or stop yielding for each active human based on belief."""
        for i in active_idxs:
            idx = int(i)

            if self.yielding[idx]:
                # Check if the robot has passed the yield cell — time to resume.
                if self._robot_has_passed(idx, robot):
                    self.yielding[idx] = False
                    self.yield_target[idx] = None
                    self.replan_timer[idx] = 0.0  # force immediate replan to goal
                continue

            # Should this human start yielding?
            # Need strong goal belief AND no evidence of unawareness.
            # Yielding only makes sense if the human knows where the robot is
            # going AND hasn't observed clear signs that the robot is unaware.
            # The gate is set low (0.3) so the neutral prior (0.5) doesn't
            # block yielding — only actively negative evidence does.
            if float(self.robot_goal_beliefs[idx].max()) < self.belief_yield_threshold:
                continue
            if float(self.robot_awareness_belief[idx]) < 0.3:
                continue  # strong evidence robot doesn't see me — stay defensive

            # Robot must be approaching (velocity toward this human).
            robot_vel = np.array(
                [robot.command_v * math.cos(robot.theta),
                 robot.command_v * math.sin(robot.theta)],
                dtype=np.float32,
            )
            to_human = self.positions[idx] - robot.position
            if np.dot(robot_vel, to_human) < 0.1:
                continue  # robot not heading toward human

            dist = float(np.linalg.norm(to_human))
            if dist < 2.0 or dist > 25.0:
                continue  # too close (reactive forces handle it) or too far

            yield_cell = self._find_yield_cell(idx, robot)
            if yield_cell is None:
                continue

            self.yield_target[idx] = yield_cell
            self.yielding[idx] = True

            # Replan path toward the yield cell.
            human_cell = self.scenario.nearest_free(
                self.scenario.world_to_cell(self.positions[idx])
            )
            if human_cell is not None:
                path = a_star(self.scenario.grid, human_cell, yield_cell)
                if path:
                    self.paths[idx] = path
                    self.path_ptr[idx] = 1 if len(path) > 1 else 0

    def _find_yield_cell(self, human_idx: int, robot: Robot) -> tuple[int, int] | None:
        """Find a free cell off the main corridor where the human can step aside."""
        human_cell = self.scenario.nearest_free(
            self.scenario.world_to_cell(self.positions[human_idx])
        )
        goal_cell = self.scenario.nearest_free(
            self.scenario.world_to_cell(self.goals[human_idx])
        )
        if human_cell is None or goal_cell is None:
            return None

        path = a_star(self.scenario.grid, human_cell, goal_cell)
        if not path:
            return None

        # Build the full corridor cell set by connecting all robot goal
        # candidates pairwise.  Any cell on this main corridor would still
        # block the robot, so it is NOT a valid hiding spot.
        corridor_set: set[tuple[int, int]] = set()
        candidates = self.scenario.robot_goal_candidates
        for i, s in enumerate(candidates):
            for e in candidates[i + 1:]:
                cp = a_star(self.scenario.grid, s, e)
                if cp:
                    corridor_set.update(cp)

        robot_cell = self.scenario.world_to_cell(robot.position)

        # Scan cells adjacent to the path for off-corridor alcove cells.
        # A valid yield cell must:
        # 1. Be free and NOT in the corridor set
        # 2. Have walls on ≥2 cardinal sides — this ensures it's a genuine
        #    pocket/alcove rather than the second row of a wide corridor.
        #    In a 2-cell-wide hallway, cells in the "other row" typically
        #    have only 1 wall (the far side) and 3 open neighbors.  A real
        #    alcove is tucked against walls on at least 2 sides.
        best_cell = None
        best_dist = float("inf")
        for cell in path:
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                if not self.scenario.is_free(neighbor):
                    continue
                if neighbor in corridor_set:
                    continue  # still on the main corridor
                # Count walls (including out-of-bounds) on cardinal sides.
                wall_count = sum(
                    1 for dx2, dy2 in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                    if not self.scenario.is_free((neighbor[0] + dx2, neighbor[1] + dy2))
                )
                if wall_count < 2:
                    continue  # too open — corridor extension, not an alcove
                # Prefer the alcove cell closest to the human that is
                # between the human and the robot (so the human reaches it
                # before they meet).
                dist_to_human = (
                    abs(neighbor[0] - human_cell[0])
                    + abs(neighbor[1] - human_cell[1])
                )
                dist_to_robot = (
                    abs(neighbor[0] - robot_cell[0])
                    + abs(neighbor[1] - robot_cell[1])
                )
                if dist_to_human < dist_to_robot and dist_to_human < best_dist:
                    best_dist = dist_to_human
                    best_cell = neighbor

        return best_cell

    def _robot_has_passed(self, human_idx: int, robot: Robot) -> bool:
        """True when the robot has moved past the yield cell toward the human's start."""
        yield_cell = self.yield_target[human_idx]
        if yield_cell is None:
            return True

        robot_cell = self.scenario.world_to_cell(robot.position)
        goal_cell = self.scenario.world_to_cell(self.goals[human_idx])

        yield_to_goal = (
            abs(yield_cell[0] - goal_cell[0]) + abs(yield_cell[1] - goal_cell[1])
        )
        robot_to_goal = (
            abs(robot_cell[0] - goal_cell[0]) + abs(robot_cell[1] - goal_cell[1])
        )
        # Robot is "past" when it's farther from the human's goal than the
        # yield cell is (i.e. it has crossed to the other side).
        return robot_to_goal > yield_to_goal

    def _update_robot_goal_beliefs(self, robot: Robot) -> None:
        """Bayesian update of each active human's belief about the robot's goal.

        Likelihood model: the robot's heading direction is consistent with
        moving toward goal g with probability proportional to a wrapped
        Gaussian centred on the angle from the robot to g.  When the robot
        is nearly stationary we skip the update — no directional information
        can be extracted from a near-zero velocity.
        """
        num_candidates = self._robot_goal_candidates.shape[0]
        if num_candidates == 0:
            return

        robot_speed = abs(robot.command_v)
        if robot_speed < 0.1:
            return

        robot_angle = math.atan2(
            robot.command_v * math.sin(robot.theta),
            robot.command_v * math.cos(robot.theta),
        )

        # Direction from robot to each candidate goal.
        diffs = self._robot_goal_candidates - robot.position  # (C, 2)
        dists = np.linalg.norm(diffs, axis=1)                 # (C,)
        valid = dists > 0.1

        goal_angles = np.where(
            valid,
            np.arctan2(diffs[:, 1], diffs[:, 0]),
            robot_angle,
        )

        # Wrapped angular difference, then Gaussian likelihood.
        angular_diffs = goal_angles - robot_angle
        angular_diffs = (angular_diffs + math.pi) % (2.0 * math.pi) - math.pi
        likelihoods = np.exp(
            -(angular_diffs ** 2) / (2.0 * self.belief_sigma ** 2)
        )
        likelihoods[~valid] = 0.0

        active_idxs = np.flatnonzero(self.active)
        for i in active_idxs:
            updated = self.robot_goal_beliefs[i] * likelihoods
            total = float(updated.sum())
            if total > 1e-10:
                self.robot_goal_beliefs[i] = updated / total

    def _update_robot_awareness_beliefs(
        self, active_idxs: np.ndarray, robot: Robot, dt: float
    ) -> None:
        """Bayesian update of each human's belief that the robot is aware of them.

        Two complementary observation channels:

        1. **Lateral avoidance** — Does the robot deviate from a straight-line
           path to its goal, away from the human?  Works in open spaces and
           junctions where the robot can steer laterally.

        2. **Speed reduction** — Does the robot slow down when approaching
           the human?  Works in head-on corridor scenarios where lateral
           deviation is impossible.  An aware robot slows or waits; an
           unaware one charges through at full speed.

        The combined likelihood is the product of both channel likelihoods
        (independent observations).
        """
        num_candidates = self._robot_goal_candidates.shape[0]
        if num_candidates == 0:
            return

        robot_speed = abs(robot.command_v)

        # Robot's actual heading direction (used for lateral channel).
        robot_heading = np.array(
            [math.cos(robot.theta), math.sin(robot.theta)], dtype=np.float32
        )

        for i in active_idxs:
            idx = int(i)

            # Skip awareness updates once the human has settled into the
            # yield cell — the robot driving at full speed through the
            # now-clear corridor is correct, not evidence of unawareness.
            # But keep updating while the human is still en route to the
            # yield cell so awareness can accumulate during the approach.
            if self.yielding[idx] and self.yield_target[idx] is not None:
                yt_pos = self.scenario.cell_to_world(self.yield_target[idx])
                if float(np.linalg.norm(self.positions[idx] - yt_pos)) < 0.6:
                    continue  # settled in alcove — skip

            # Use this human's MAP estimate of the robot's goal.
            best_goal_idx = int(np.argmax(self.robot_goal_beliefs[idx]))
            goal_pos = self._robot_goal_candidates[best_goal_idx]

            # Direction from robot to this human.
            to_human = self.positions[idx] - robot.position
            human_dist = float(np.linalg.norm(to_human))
            if human_dist < 0.5 or human_dist > 15.0:
                continue  # too close or too far for meaningful inference

            # --- Channel 1: Lateral avoidance ---
            lateral_likelihood_aware = 1.0
            lateral_likelihood_unaware = 1.0

            if robot_speed >= 0.1:
                to_goal = goal_pos - robot.position
                goal_dist = float(np.linalg.norm(to_goal))
                if goal_dist > 0.5:
                    to_goal_dir = to_goal / goal_dist
                    to_human_dir = to_human / human_dist

                    # Which side is the human on (cross product)?
                    lateral_human = float(
                        to_goal_dir[0] * to_human_dir[1]
                        - to_goal_dir[1] * to_human_dir[0]
                    )
                    # Which side is the robot heading (cross product)?
                    lateral_heading = float(
                        to_goal_dir[0] * robot_heading[1]
                        - to_goal_dir[1] * robot_heading[0]
                    )
                    # Positive avoidance_signal = robot steers away from human.
                    avoidance_signal = -lateral_human * lateral_heading

                    lateral_likelihood_aware = math.exp(
                        -(max(0.0, -avoidance_signal) ** 2)
                        / (2.0 * self.awareness_sigma ** 2)
                    )
                    lateral_likelihood_unaware = math.exp(
                        -(max(0.0, avoidance_signal) ** 2)
                        / (2.0 * self.awareness_sigma ** 2)
                    )

            # --- Channel 2: Speed reduction when approaching ---
            speed_likelihood_aware = 1.0
            speed_likelihood_unaware = 1.0

            # Only informative when robot is heading toward the human.
            robot_vel = np.array(
                [robot.command_v * math.cos(robot.theta),
                 robot.command_v * math.sin(robot.theta)],
                dtype=np.float32,
            )
            approaching = float(np.dot(robot_vel, to_human)) > 0.0

            if approaching and human_dist < 15.0:
                # Expected speed of an unaware robot: full speed.
                # An aware robot reduces speed as it gets closer.
                # speed_ratio ∈ [0, 1]: how much of max speed the robot uses.
                speed_ratio = robot_speed / max(robot.max_speed, 0.1)
                # Expected speed ratio for an aware robot at this distance:
                # aware robots slow down roughly proportional to proximity.
                # At dist=15 → expected ~1.0; at dist=2 → expected ~0.2.
                expected_aware_ratio = min(1.0, human_dist / 12.0)

                # How well does observed speed match aware vs unaware model?
                # Use a wider sigma at long distance (weak signal) and tighter
                # sigma at close range (strong signal).
                proximity_weight = max(0.0, 1.0 - human_dist / 15.0)
                sigma_s = self.awareness_sigma / (0.5 + proximity_weight)
                speed_likelihood_aware = math.exp(
                    -((speed_ratio - expected_aware_ratio) ** 2)
                    / (2.0 * sigma_s ** 2)
                )
                speed_likelihood_unaware = math.exp(
                    -((speed_ratio - 1.0) ** 2)
                    / (2.0 * sigma_s ** 2)
                )

            # --- Combined Bayesian update ---
            likelihood_aware = lateral_likelihood_aware * speed_likelihood_aware
            likelihood_unaware = lateral_likelihood_unaware * speed_likelihood_unaware

            prior_aware = float(self.robot_awareness_belief[idx])
            prior_unaware = 1.0 - prior_aware

            posterior_aware = likelihood_aware * prior_aware
            posterior_unaware = likelihood_unaware * prior_unaware
            total = posterior_aware + posterior_unaware
            if total > 1e-10:
                self.robot_awareness_belief[idx] = np.float32(
                    np.clip(posterior_aware / total, 0.01, 0.99)
                )

    def _belief_conditioned_forces(
        self, active_idxs: np.ndarray, robot: Robot
    ) -> np.ndarray:
        """Anticipatory repulsion from the robot's predicted future position.

        When a human's belief is concentrated enough (max belief ≥
        belief_yield_threshold) it knows where the robot is heading.  We add
        a repulsive force from the robot's extrapolated position
        belief_lookahead seconds in the future, pushing the human aside
        *before* the robot arrives.  This is complementary to the reactive
        SFM force (which acts on current distance) and enables anticipatory
        yielding.
        """
        forces = np.zeros((active_idxs.size, 2), dtype=np.float32)

        robot_speed = abs(robot.command_v)
        if robot_speed < 0.1:
            return forces

        predicted_robot_pos = robot.position + np.array(
            [
                robot.command_v * math.cos(robot.theta),
                robot.command_v * math.sin(robot.theta),
            ],
            dtype=np.float32,
        ) * self.belief_lookahead

        for j, i in enumerate(active_idxs):
            if float(self.robot_goal_beliefs[i].max()) < self.belief_yield_threshold:
                continue

            diff = self.positions[i] - predicted_robot_pos
            dist = float(np.linalg.norm(diff))
            if dist < 1e-4 or dist > 4.0:
                continue

            direction = diff / dist
            belief_strength = float(self.robot_goal_beliefs[i].max())
            awareness = float(self.robot_awareness_belief[i])

            # Awareness modulates force strength:
            # - High awareness (≈1): human trusts robot will avoid, moderate
            #   anticipatory nudge (1.0× base).
            # - Low awareness (≈0): human is defensive — robot might not see
            #   me, so push harder to self-protect (up to 2.0× base).
            awareness_scale = 2.0 - awareness

            mag = (
                self.robot_human_amplitude
                * math.exp(-dist / self.robot_human_decay)
                * belief_strength
                * awareness_scale
            )
            forces[j] += direction * mag

        return forces

    def _social_forces(self, active_idxs: np.ndarray, robot: Robot) -> tuple[np.ndarray, float]:
        n = active_idxs.size
        forces = np.zeros((n, 2), dtype=np.float32)
        robot_social_force_generated = 0.0

        a_h = self.human_human_amplitude
        b_h = self.human_human_decay
        a_obs = self.obstacle_amplitude
        b_obs = self.obstacle_decay

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
        robot_cell = self.scenario.world_to_cell(robot_pos)
        for i in range(n):
            diff = pos[i] - robot_pos
            dist = np.linalg.norm(diff)
            if dist < 1e-4:
                continue
            direction = diff / dist
            combined = rad[i] + robot.radius
            penetration = combined - dist

            # Only apply social distancing force when the robot is on the
            # human's optimal path (triangle inequality tight).  When the robot
            # is in a side alcove the human doesn't need to divert, so we drop
            # the exponential term and keep only the physical contact force.
            human_cell = self.scenario.world_to_cell(pos[i])
            human_goal_cell = self.scenario.world_to_cell(self.goals[active_idxs[i]])
            h_to_r = abs(human_cell[0] - robot_cell[0]) + abs(human_cell[1] - robot_cell[1])
            r_to_hg = abs(robot_cell[0] - human_goal_cell[0]) + abs(robot_cell[1] - human_goal_cell[1])
            h_to_hg = abs(human_cell[0] - human_goal_cell[0]) + abs(human_cell[1] - human_goal_cell[1])
            robot_is_blocking = h_to_r > 0 and (h_to_r + r_to_hg == h_to_hg)

            mag = self.robot_human_amplitude * math.exp((combined - dist) / self.robot_human_decay) if robot_is_blocking else 0.0
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
