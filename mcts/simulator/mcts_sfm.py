import math
import numpy as np

from simulator.constants import WALL
from simulator.mcts_game_state import MCTSGameState


def calculate_likely_velocity(mcts_game_state: MCTSGameState, dt: float):
    human_preferred_speed = 1.7

    human_positions = mcts_game_state.positions[1:, :]
    human_velocities = mcts_game_state.velocities[1:, :].copy()
    human_goal_positions = mcts_game_state.agent_goal_positions[1:, :]
    num_humans = human_positions.shape[0]

    desired = np.zeros((num_humans, 2), dtype=np.float32)
    for i in range(num_humans):
        to_target = human_goal_positions[i] - human_positions[i]
        dist = np.linalg.norm(to_target)
        if dist > 1e-6:
            desired[i] = (to_target / dist) * human_preferred_speed

    relaxation_time = 0.45
    accel = (desired - human_velocities) / relaxation_time
    social_forces, force_generated = _social_forces(human_positions, mcts_game_state)
    accel += social_forces
    human_velocities += accel * dt

    speed = np.linalg.norm(human_velocities, axis=1)
    max_speed = human_preferred_speed * 1.7
    too_fast = speed > max_speed
    if np.any(too_fast):
        human_velocities *= (max_speed[too_fast] / speed[too_fast])[:, None]

    return human_velocities

def _social_forces(human_positions, mcts_game_state: MCTSGameState) -> tuple[np.ndarray, float]:
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
            penetration = mcts_game_state.config.human_radius + mcts_game_state.config.human_radius - dist
            mag = a_h * math.exp((mcts_game_state.config.human_radius + mcts_game_state.config.human_radius - dist) / b_h)
            if penetration > 0.0:
                mag += penetration * 25.0
            force = direction * mag
            forces[i] += force
            forces[j] -= force

    robot_pos = mcts_game_state.positions[0, :]
    for i in range(n):
        diff = human_positions[i] - robot_pos
        dist = np.linalg.norm(diff)
        if dist < 1e-4:
            continue
        direction = diff / dist
        combined = mcts_game_state.config.human_radius + mcts_game_state.config.robot_radius
        penetration = combined - dist
        mag = 10.0 * math.exp((combined - dist) / 0.6)
        if penetration > 0.0:
            mag += penetration * 30.0
        force = direction * mag
        forces[i] += force
        robot_social_force_generated += float(np.linalg.norm(force))

    for i in range(n):
        cell = mcts_game_state.scenario.world_to_cell(human_positions[i])
        for oy in range(-2, 3):
            for ox in range(-2, 3):
                cx, cy = cell[0] + ox, cell[1] + oy
                if cx < 0 or cx >= mcts_game_state.scenario.width or cy < 0 or cy >= mcts_game_state.scenario.height:
                    continue
                if mcts_game_state.scenario.grid[cy, cx] != WALL:
                    continue
                obstacle_pos = np.array([float(cx) + 0.5, float(cy) + 0.5], dtype=np.float32)
                diff = human_positions[i] - obstacle_pos
                dist = np.linalg.norm(diff)
                if dist < 1e-4:
                    continue
                direction = diff / dist
                mag = a_obs * math.exp((mcts_game_state.config.human_radius + 0.5 - dist) / b_obs)
                forces[i] += direction * mag

    return forces, robot_social_force_generated