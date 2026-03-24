from __future__ import annotations

import heapq
import math

import numpy as np

from .constants import WALL


def _prediction_cells(
    human_predictions: np.ndarray | None,
    width: int,
    height: int,
) -> np.ndarray | None:
    if human_predictions is None or human_predictions.size == 0:
        return None

    cells = np.floor(human_predictions).astype(np.int32)
    cells[..., 0] = np.clip(cells[..., 0], 0, width - 1)
    cells[..., 1] = np.clip(cells[..., 1], 0, height - 1)
    return cells

def state_key(cell: tuple[int, int], timestep: int, prediction_count: int) -> tuple[tuple[int, int], int]:
    # All times after the prediction horizon are equivalent for collision pruning.
    # Assume that at prediction count the Crowd agent stands at the last position
    return cell, min(timestep, prediction_count)

def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _social_force_cost(
    robot_position: np.ndarray,
    human_positions: np.ndarray,
    robot_radius: float,
    human_radius: float,
) -> float:
    if human_positions.size == 0:
        return 0.0

    combined = robot_radius + human_radius
    total_cost = 0.0
    for human_position in human_positions:
        diff = human_position - robot_position
        dist = float(np.linalg.norm(diff))
        if dist < 1e-4:
            return math.inf

        penetration = combined - dist
        mag = 10.0 * math.exp((combined - dist) / 0.6)
        if penetration > 0.0:
            mag += penetration * 30.0
        total_cost += mag

    return total_cost


def a_star(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    human_predictions: np.ndarray | None = None,
    min_clearance: float = 0.0,
    social_force_weight: float = 0.0,
    robot_radius: float = 0.35,
    human_radius: float = 0.28,
) -> list[tuple[int, int]]:
    if start == goal:
        return [start]

    h, w = grid.shape
    if not (0 <= start[0] < w and 0 <= start[1] < h):
        return []
    if not (0 <= goal[0] < w and 0 <= goal[1] < h):
        return []
    if grid[start[1], start[0]] == WALL or grid[goal[1], goal[0]] == WALL:
        return []

    predicted_cells = _prediction_cells(human_predictions, w, h)
    prediction_count = 0 if predicted_cells is None else predicted_cells.shape[1]

    open_heap: list[tuple[float, tuple[tuple[int, int], int]]] = []
    start_state = state_key(start, 0, prediction_count)
    heapq.heappush(open_heap, (0.0, start_state))
    came_from: dict[tuple[tuple[int, int], int], tuple[tuple[int, int], int]] = {}
    g_score = {start_state: 0.0}

    neighbours = [(1, 0), (-1, 0), (0, 0), (0, 1), (0, -1)]

    while open_heap:
        _, current_state = heapq.heappop(open_heap)
        current, current_time = current_state
        if current == goal:
            path = [current]
            trace_state = current_state
            while trace_state in came_from:
                trace_state = came_from[trace_state]
                path.append(trace_state[0])
            path.reverse()
            return path

        for dx, dy in neighbours:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt[0] < 0 or nxt[0] >= w or nxt[1] < 0 or nxt[1] >= h:
                continue
            if grid[nxt[1], nxt[0]] == WALL:
                continue

            next_time = current_time + 1
            if predicted_cells is not None and next_time < prediction_count:
                occupied = np.any(
                    (predicted_cells[:, next_time, 0] == nxt[0])
                    & (predicted_cells[:, next_time, 1] == nxt[1])
                )
                if occupied:
                    continue

            if human_predictions is not None and next_time < prediction_count and min_clearance > 0.0:
                candidate_world = np.array([nxt[0] + 0.5, nxt[1] + 0.5], dtype=np.float32)
                human_distances = np.linalg.norm(human_predictions[:, next_time] - candidate_world, axis=1)
                if np.any(human_distances < min_clearance):
                    continue
            else:
                candidate_world = np.array([nxt[0] + 0.5, nxt[1] + 0.5], dtype=np.float32)

            next_state = state_key(nxt, next_time, prediction_count)
            step_cost = 1.0
            if human_predictions is not None and next_time < prediction_count and social_force_weight > 0.0:
                step_cost += social_force_weight * _social_force_cost(
                    candidate_world,
                    human_predictions[:, next_time],
                    robot_radius,
                    human_radius,
                )

            tentative = g_score[current_state] + step_cost
            if tentative < g_score.get(next_state, math.inf):
                came_from[next_state] = current_state
                g_score[next_state] = tentative
                f = tentative + heuristic(nxt, goal)
                heapq.heappush(open_heap, (f, next_state))

    return []
